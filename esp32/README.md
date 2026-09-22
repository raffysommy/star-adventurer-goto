# StarMount: ESP32-S3 mount controller

Firmware that turns an ESP32-S3 into the complete controller for the Star Adventurer
GoTo mod. It replaces the Raspberry Pi running `lx200.py` and the separate DEC
controller:

- **RA**: the Star Adventurer's own motor, over USB (the ESP is the USB host)
- **DEC**: the NEMA17 stepper on an EasyDriver, driven directly
- **Clients**: Meade LX200 over Wi-Fi (INDI "LX200 GPS", Stellarium, ...), plus a web dashboard

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
| TCP 5001 | LX200 (same replies as `lx200.py`, so existing client setups keep working) |
| `http://starmount.local/` | Dashboard: RA/DEC, state, LST, clock, settings, DEC test buttons |
| `/api/status` | JSON state (used by the dashboard and the tests) |
| `/api/settings`, `/api/time`, `/api/home`, `/api/stop` | Settings, set clock from the browser, set RA home, stop |
| `/api/lx200?c=:GR` | Run one LX200 command over HTTP |
| `/api/register`, `/api/goto_ha`, `/api/dec` | Debug: redefine the RA register, slew to a register angle, DEC test moves |
| `/log`, telnet 23 | Log (kept across resets: after a crash `/log` still shows the previous boot) |
| `/sys`, `/cmd?c=:e1`, `/update`, `/wifi` | System status, raw mount command, firmware upload, Wi-Fi setup |
| UDP 11880 | SynScan Wi-Fi-dongle compatible bridge to the mount (debugging only: it bypasses the RA controller) |

## Architecture

```
 clients:  INDI / Stellarium / PHD2        browser            (later: OAT tools)
               │ LX200 TCP 5001               │ HTTP                 │
 ┌─────────────┴──────────────┐   ┌───────────┴───────┐   ┌──────────┴─────────┐
 │ lx200_server               │   │ dashboard         │   │ (OAT protocol)     │  protocol
 │ commands → targets, flip,  │   │ JSON API          │   │ front-end          │  front-ends
 │ DEC reversal, pulse guide  │   │                   │   │                    │
 └──────┬──────────────┬──────┘   └───────────────────┘   └────────────────────┘
        │              │
 ┌──────┴──────┐ ┌─────┴──────┐        ┌───────────────────────────────┐
 │ ra_axis     │ │ dec_axis   │        │ lib/core/astro (pure C++)     │  pure logic,
 │ task + queue│ │ FastAccel- │ ◄────► │ LST, HA/RA, meridian flip,    │  host-tested
 │ tracking,   │ │ Stepper    │        │ DEC steps, LX200 parse/format │
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
- **Protocols are thin front-ends.** `lx200_server` turns commands into axis calls and keeps
  the protocol-level state (target, meridian-flip flag). An OpenAstroMount/OAT-compatible
  front-end would sit next to it on the same axis API.
- **Math lives in `lib/core`** with no Arduino dependencies. It is unit-tested on the PC against
  golden values produced by `lx200.py`'s own functions (`test/gen_golden.py`).
- **Guiding is never disturbed.** Pulses change only the RA step period while the motor keeps
  running. DEC pulses are exact step moves. Recovery from firmware auto-stops uses a bare
  `J`, never a stop/restart.
- **Cores:** Wi-Fi, lwIP and USB host on core 0; RA/DEC/LX200 on core 1. DEC step pulses come from hardware.

Planned additions: GPS (UART; time and site), a 9-axis IMU on the battery pack (polar axis
altitude/heading), a 6-axis IMU on the camera (absolute pointing sanity checks, rough
initial sync), all feeding `clock`/`settings` and a pointing-check module; optionally the
OAT protocol front-end.

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
- DEC guide pulses are exact step counts, not timed stops (those overshot by the queued steps).
- The RA register survives ESP restarts (read back from the mount). The DEC position survives soft
  resets and OTA (kept in RAM). After a power-on DEC starts at 0° (146400 steps), as `lx200.py` did.

## Tests

- `pio test -e native`: astro math, DEC steps, parsing. Regenerate golden values with
  `python3 test/gen_golden.py` (needs `ephem`).
- `test/hil/` (hardware in the loop, against a running ESP and mount):
  - `guide_soak.py [N]`: random N/S/E/W pulse guiding; checks both axes moved exactly as
    commanded and that the RA motor was never restarted
  - `goto_test.py`: sync, then two combined RA+DEC GoTos and back. **Moves the mount.**

  ```sh
  cd test/hil && STARMOUNT_HOST=starmount.local STARMOUNT_IFACE=wlo1 python3 guide_soak.py
  ```
