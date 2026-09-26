# StarMount: ESP32-S3 mount controller

Firmware that turns an ESP32-S3 into the complete controller for the Star Adventurer
GoTo mod. It replaces the Raspberry Pi running `lx200.py` and the separate DEC
controller:

- **RA**: the Star Adventurer's own motor, over USB (the ESP is the USB host)
- **DEC**: the NEMA17 stepper on an EasyDriver, driven directly
- **Clients**: OnStep protocol over Wi-Fi, a superset of Meade LX200. INDI/Ekos and ASIAIR use their
  OnStep driver (pier side, meridian limits, backlash), NINA the ASCOM OnStep driver, and
  Stellarium, SkySafari and generic LX200 drivers work too. Plus a web dashboard.

## Hardware

| What | Details |
|---|---|
| Board | ESP32-S3 N16R8 (16 MB flash, 8 MB octal PSRAM) |
| RA link | Native USB port (OTG, GPIO19/20) ↔ mount's USB (Prolific PL2303 HXD, `067b:2303`), 115200 8N1 |
| VBUS | The ESP must supply 5 V on the OTG port: board powered from the 5V pin, "USB-OTG" pad bridged |
| DEC | EasyDriver: STEP GPIO6, DIR GPIO5, EN GPIO38 (active low, disabled while idle) |
| Ground | ESP, EasyDriver/12 V supply and mount share ground |

The OTG port belongs to the mount, so after the first flash **everything goes over Wi-Fi**:
updates, logs, control. The CH343 "UART" port shows the UART0 console if you can attach it.

## Build, flash, update

Needs PlatformIO ≥ 6.2 (the pioarduino platform, Arduino core 3.x / ESP-IDF 5.5, is pulled
in automatically).

```sh
cd esp32
# Wi-Fi credentials (git-ignored); without them the ESP opens AP "StarMount" / "starmount"
cat > secrets.ini <<'EOF'
[secrets]
build_flags = -DWIFI_SSID=\"MySSID\" -DWIFI_PASS=\"MyPassword\"
EOF

pio run -e usb -t upload            # first flash over USB (BOOT+RESET for download mode)
pio run -e ota -t upload            # later: ArduinoOTA to starmount.local
curl -F fw=@.pio/build/usb/firmware.bin http://starmount.local/update   # or HTTP upload
pio test -e native                  # host unit tests (no hardware)
```

**Recovery.** A new image that crashes before Wi-Fi comes up is meant to roll back to the previous
one, but it hasn't been checked that the bootloader actually has rollback enabled.
The USB host starts 10 s after boot, so OTA always has a window. Holding BOOT during that
window skips the USB host for one boot. Worst case: unplug the mount, BOOT+RESET, flash over USB.

**Tailscale / VPN.** If a VPN advertises the LAN subnet (Tailscale `--accept-routes`), traffic
to the ESP goes into the tunnel and it looks dead. Disable accept-routes, or bind to the
Wi-Fi interface (`curl --interface wlo1`, `STARMOUNT_IFACE=wlo1` for the tests).

## Interfaces

| Port / path | What |
|---|---|
| TCP 5001, 9999 | OnStepX-compatible commands (LX200 superset): see [docs/onstep-protocol.md](docs/onstep-protocol.md). 5001 is the old `lx200.py` port, 9999 OnStep's usual one |
| `http://starmount.local/` | Dashboard: RA/DEC, state, LST, clock, settings, DEC test buttons |
| `/api/status` | JSON state (used by the dashboard and the tests) |
| `/api/settings`, `/api/time`, `/api/home`, `/api/stop` | Settings, set clock from the browser, set RA home, stop |
| `/api/lx200?c=:GR` | Run one OnStep/LX200 command over HTTP |
| `/api/register`, `/api/goto_ha`, `/api/dec` | Debug: redefine the RA register, slew to a register angle, DEC test moves |
| TCP 10110 | NMEA over TCP: the GPS sentences, for INDI's "GPS NMEA" driver (KStars/Ekos time and location) |
| `/api/tasks`, `/api/gps_nmea` | Per-task CPU/stack/heap; inject NMEA sentences (debug, no GPS needed) |
| `/log`, telnet 23 | Log (kept across resets: after a crash `/log` still shows the previous boot) |
| `/sys`, `/cmd?c=:e1`, `/update`, `/wifi` | System status, raw mount command, firmware upload, Wi-Fi setup |
| UDP 11880 | SynScan Wi-Fi-dongle compatible bridge to the mount (debugging only: it bypasses the RA controller) |

## Architecture

```
 clients:  INDI / NINA / ASIAIR / Stellarium / PHD2           browser
               │ OnStep (LX200 superset), TCP 5001 + 9999    │ HTTP
 ┌─────────────┴──────────────┐                  ┌───────────┴───────┐
 │ onstep_server              │                  │ dashboard         │  protocol
 │ parse, reply formats       │                  │ JSON API          │  front-ends
 └─────────────┬──────────────┘                  └───────────────────┘
 ┌─────────────┴──────────────┐
 │ mount: target, flip branch,│  one flip flag for the whole mount
 │ DEC reversal, guide, sync  │
 └──────┬──────────────┬──────┘
        │              │
 ┌──────┴──────┐ ┌─────┴──────┐        ┌───────────────────────────────┐
 │ ra_axis     │ │ dec_axis   │        │ lib/core/astro (pure C++)     │  pure logic,
 │ task + queue│ │ FastAccel- │ ◄────► │ LST, HA/RA, meridian flip,    │  host-tested
 │ tracking,   │ │ Stepper    │        │ DEC steps, OnStep parse/format│
 │ goto, guide │ └─────┬──────┘        └───────────────────────────────┘
 └──────┬──────┘       │
 ┌──────┴──────┐       │           services: clock (NTP / LX200 / browser → later GPS),
 │ skywatcher  │       │                     settings (NVS), netlog, OTA
 │ protocol    │       │
 ├─────────────┤       │
 │ mount_usb + │   EasyDriver
 │ pl2303      │   (DEC motor)
 └──────┬──────┘
    USB OTG → Star Adventurer (RA)
```

Design rules:

- **One owner per motor.** The `ra_axis` task is the only code that sends RA motion commands.
  Everything else posts requests to its queue, so commands can't interleave (that was
  `lx200.py`'s `ra_locker` and its "junk data" workarounds). `mount_usb` serializes the
  request/response pairs underneath.
- **Protocols are thin front-ends.** `onstep_server` only parses and formats. The mount-level
  state (target, meridian-flip branch, guide flags) lives in `mount`, shared by every
  front-end, so there is one flip flag whatever the client.
- **Math lives in `lib/core`** with no Arduino dependencies. It is unit-tested on the PC against
  golden values produced by `lx200.py`'s own functions (`test/gen_golden.py`).
- **Guiding is never disturbed.** Pulses change only the RA step period while the motor keeps
  running. DEC pulses are exact step moves. Recovery from firmware auto-stops uses a bare
  `J`, never a stop/restart.
- **Cores:** Wi-Fi, lwIP and USB host on core 0; RA/DEC/OnStep server on core 1. DEC step pulses come from hardware.

Planned additions: GPS (time, site, magnetic declination), a 9-axis IMU in the base box
(polar axis altitude and heading) and a 6-axis IMU on the camera hot shoe (mechanical pose,
push-to, pier side, safety limits), plus fast polar alignment in tiers from sky-less
(~0.5–1°) to two plate solves (~1′). The design is in
[docs/sensors-and-alignment.md](docs/sensors-and-alignment.md).

## Star Adventurer firmware quirks (measured)

| Quirk | Handling |
|---|---|
| Motion mode `G` is two digits: `1x`=tracking slow, `3x`=tracking fast; `x0`=CW, `x1`=CCW (the fix in the modified pysynscan) | `sw::setMode` |
| `G` is refused (`!2`) while the motor runs | stop, poll `:f` until stopped, then G → I → J |
| Max RA speed ≈ 0.28°/s (~68× sidereal); T1 below ~6 or fast mode is not faster | slews use slow mode, T1 = 5 |
| At slew speed, CW auto-stops after ~0.8 s; after any direction reversal both directions stop after ~0.5 s | `J` re-sent every 400 ms during slews; in tracking, `:f` polled every 250 ms and a bare `J` sent if stopped |
| Register below 0 → tracking stalls every second; above ±241.7° the 24-bit register overflows | GoTo/sync targets limited to HA 2°–183° (`offset_star_adventurer` = 2°), overflowing register values refused |
| At power-on the mount starts tracking by itself, so the register isn't exactly 0 | a register within 1° of 0 = fresh mount → set to home (HA 2°) |

## Differences from lx200.py

- **LST is correct.** `lx200.py` gives ephem the longitude as a float (read as radians) and local
  time as UTC, so its sidereal time is off by a constant (~93° plus the timezone). A sync hid this
  for RA, but HA-based decisions moved: **the meridian flip now happens at the true meridian.**
- `:SG/:SL/:SC` set the clock (the Pi had its own). `:Sg/:St` parse signs correctly. `:GG` uses the
  Meade sign convention.
- `:MS`/`:CM` without a prior valid `:Sr` don't move or sync RA.
- **The protocol is OnStepX's** (since 2026-09-26), not `lx200.py`'s dialect. Set commands answer
  `1`/`0`, `:CM` answers `N/A#`, stops and pulses answer nothing, and `:GVP#` is `On-Step`.
  Standard LX200 clients don't notice. Tracking can be turned off (`:Td`/`:Te`).
- DEC guide pulses are exact step counts, not timed stops (those overshot by the queued steps).
- RA guide/slow-move directions follow ASCOM/EQMOD: west = 1.5×, east = 0.5× sidereal (`lx200.py`
  had them swapped). Redo PHD2 calibration made with the old firmware.
- DEC guide rate is 0.5× sidereal (1.7 steps/s). `lx200.py`'s "sidereal" 6.796 steps/s was really 2×.
- DEC backlash is compensated (below), and DEC GoTos always end moving north (+steps).
- **RA window and flip.** The register window starts at a configurable east limit (dashboard, NVS
  `ra_east`, default 0 = `lx200.py`'s flip at the meridian; −30 = flip 2 h before it). register =
  HA − east limit + 2°. GoTos use the window [2°, 182°] on either branch (`lx200.py` flipped at 180,
  leaving a 2° dead zone). Tracking goes on past the window and **stops at register 235°**, before
  the 24-bit overflow. Changing the east limit shifts the register so the position keeps its meaning.
- **Branch choice happens at use, not at `:Sr`.** `:MS` takes the window's branch, and `:CM` keeps
  the branch nearest the current register (a sync doesn't move the mount). Before, a sync past the
  window flipped the maths without moving: the old beyond-the-pole bug. DEC steps are computed then
  too, so the `:Sr`/`:Sd` order doesn't matter. The flip flag survives OTA and crash resets.
- The RA register survives ESP restarts (read back from the mount). The DEC position survives soft
  resets and OTA (kept in RAM). After a power-on DEC starts at 0° (146400 steps), as `lx200.py` did.

## Measured on the sky (2026-09-25, plate solves through a window)

M50 + 70-200 @ 200 mm (3.9″/px), astrometry.net 4100 indexes, scripts in `tools/sky/` (prototypes: ESP IP and `wlo1` are hard-coded in `pa.py`; run them from `$HOME`, gphoto2 is a snap)
(`pa.py`: polar alignment from two solves around an RA move; `mount_tests.py`: DEC, sync,
GoTo, guide, drift; `pe_record.py`: RA drift from live-view frames, no shutter).

| What | Result |
|---|---|
| DEC backlash | ~250 steps (0.30–0.34°) on every reversal, reproducible |
| DEC scale | 805 steps/° on one clean move (nominal 813.3) |
| RA/DEC orthogonality | 89.8° |
| RA GoTo after sync | 0.5–3′ (limited by the RA drift below) |
| DEC GoTo after sync | before: ~1′ if ending north, 17–18′ if ending south; with compensation: 0.4–1.4′ both ways |
| DEC guide pulses | before: the first ~4 pulses after a reversal did nothing; now the first one overshoots ~2× once, then settles |
| RA guide pulses | 0.45–0.6× sidereal (nominal 0.5×) |
| RA tracking (unguided, 10 min, 1 frame/min) | register exactly sidereal, but the sky wanders ±60″ — mechanical |
| RA periodic error (live view, 22 min, ~7 fps) | **±30″ sine at the worm period (~10 min)**, cycles correlate 0.89, 7″ rms non-periodic left. Max slope 0.3″/s: PHD2 + Predictive PEC; no firmware PEC (no worm index, phase lost at power-on) |

**DEC backlash model.** `dec_axis` keeps the motor position and the gear output apart: the gear
sits within `[motor − backlash, motor]` and only moves when pushed. Everything reported and
targeted is the gear position. On a reversal the play is run out at slew speed (guide pulses
included, so PHD2 sees no dead zone). `backlash` is a setting (dashboard, NVS key `dec_bl`,
default 250 steps); lowering it softens the one-pulse overshoot after a reversal.

## PEC, limits and GPS (2026-09-26)

- **PEC**, OnStep-style (`:$QZ+ - / Z ! ?`, `:VR`/`:WR`, `:GXE6/7/8`; the INDI OnStep PEC tab):
  - The table has one correction per worm segment (~1 sidereal s; 599 segments of 86,751 counts
    per 144-tooth worm turn), in register counts.
  - **Record** takes RA guide pulses for one worm turn (turn PHD2's Predictive PEC off). It is
    smoothed, its mean removed, and averaged with the previous table.
  - **Play** changes the RA step period each segment and carries the rounding into the next
    segment, so a whole turn is exact despite the coarse T1.
  - **Worm phase** follows every register rewrite (sync, home, east limit) and is saved to NVS
    every 10 s. After a power-on it is restored, on the assumption that the worm didn't turn
    while unpowered.
  - **Playback only starts after 60 s without guide pulses**, so PHD2 never sees PEC switch on mid-guiding.
- **Limits**:
  - GoTo is refused below the horizon limit (`:Sh`, code `1`) and above the overhead limit (`:So`, `2`).
  - Tracking stops `raWestMinutes` past the meridian on the flipped branch (`:SXEA`, default
    120, capped by the overflow). `:SXE9` sets the east limit.
- **GPS** (boilerplate until the module is wired: UART1 RX 17 / TX 18, 9600, any NMEA RMC+GGA):
  - Time is trusted below NTP and above clients.
  - A fix with ≥4 satellites and HDOP < 5 sets the site.
  - `:GU#` reports `S` while the GPS keeps the clock.
  - The sentences are relayed on TCP 10110.
  - Tested by injection.
- **Resources** (measured, guiding plus a client polling 10× faster than INDI):
  - core 0 6% busy, core 1 18% busy
  - 157 KB internal heap free, PSRAM unused

## Rates, refraction, limits (2026-09-26)

- **Tracking rates:**
  - sidereal, lunar, solar, King and custom (`:TQ :TL :TS :TK :ST`); not saved, sidereal at boot
  - The base is now true sidereal (`lx200.py`'s 0.004176 was a fixed "refracted" slow-down).
  - **Calibrated:** the SA's register runs at exactly timerHz / T1, within ±0.04% from T1 280 to
    439 (`test/hil/rate_calibration.py`, data in `rate_calibration.csv`). The earlier "lunar
    0.36% slow" and PEC's 1.04 gain were laptop-side HTTP timing. Rate tests now use the ESP's
    own register-read timestamp (`counts_ms` in `/api/status`).
- **Refraction** (`:Tr`/`:Tn`, default on, RA only; `:T2` dual axis is not supported):
  - Every 10 s the rate is scaled by d(apparent HA)/d(true HA) at the current pointing
    (Saemundsson's formula).
  - Examples: 0.99975 on the meridian at Dec +20, about 0.9985 at 20° altitude in the west.
- **Guide rate** is one setting for both axes (`:SX90,n#`, dashboard, 0.1–0.9×, default 0.5).
  - DEC backlash can be set from the client (`:$BD[arcsec]#`). RA backlash only accepts 0.
- **Axis limits** (`:GXEe/w` RA window in axis hour angle, `:GXEC/D` DEC mechanical, set with `:SXEx,n#`):
  - A GoTo past a DEC limit is refused with code `6`.
  - Manual moves stop at the limit, and guide pulses are clamped.
  - This is the counts-based layer of the collision limits.
- **PEC start rule** (default *strict*): playback starts only before guiding begins for the
  target (no pulse since boot or the last GoTo); otherwise it waits for the next GoTo.
  - Option: after 60 s without pulses.
  - A table recorded with a different segment count (it changed from 599 to 598 with true
    sidereal) is not loaded: record again.

## Tests

- `pio test -e native`: astro math, DEC steps, parsing. Regenerate golden values with
  `python3 test/gen_golden.py` (needs `ephem`).
- `test/hil/` (hardware in the loop, against a running ESP and mount):
  - `guide_soak.py [N]`: random N/S/E/W pulse guiding; checks both axes moved exactly as
    commanded and that the RA motor was never restarted
  - `goto_test.py`: sync, then two combined RA+DEC GoTos and back. **Moves the mount.**
  - `pec_test.py`: records a synthetic periodic error from simulated guide pulses (one worm
    turn), checks the table, plays it back and checks that the register follows it (~17 min, RA tracking only).
    2026-09-26: table correlation 0.978, playback correlation 0.999, gain 1.04, 5.4 counts rms residual.
  - `features_test.py`: tracking rates, refraction, guide rate, backlash, DEC axis limits and
    the strict PEC rule (~3 min, small moves)
  - `rate_calibration.py`: RA register rate vs raw T1, timed on the ESP (~2.5 min per point)

  ```sh
  cd test/hil && STARMOUNT_HOST=starmount.local STARMOUNT_IFACE=wlo1 python3 guide_soak.py
  ```
