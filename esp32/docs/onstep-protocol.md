# OnStep front-end: the command subset

**Status (2026-09-26):** implemented in `src/onstep_server.cpp`, replacing the LX200 front-end.
Tested with the real INDI OnStep driver against the ESP: connect, sync, GoTo there and back,
guide pulses, moves, tracking off/on, time, location. Pier side reached INDI. The HIL guide soak
passes on port 5001. Not yet tested: Ekos + PHD2 across a real flip, NINA (ASCOM).

Goal: an OnStepX-compatible protocol front-end on the ESP, next to the LX200 one, so that
INDI (Ekos, ASIAIR) and ASCOM (NINA) use their **OnStep drivers**. Those drivers understand
pier side, meridian limits, backlash, PEC and park, which the generic LX200 protocol can't
express. The biggest gain is pier side: PHD2 then flips its DEC calibration on its own after
a meridian flip, so no recalibration is needed.

## How this list was made (2026-09-26)

- The **real INDI OnStep driver** (`indi_lx200_OnStep`, INDI 2.2.0) was pointed at a logging
  stand-in (`tools/onstep/stub.py`).
- It was driven through connect, GoTo, sync, guide pulses, manual moves, abort, tracking on/off
  and track rate, time, location, park and unpark (`tools/onstep/actions.py`).
- The capture is `tools/onstep/capture-indi-2.2.0.log`.
- Reply formats come from the **OnStepX firmware source**, taken as the reference
  implementation (github.com/hjd1964/OnStepX, 2026-09-08).
- Unknown commands get `0`: OnStepX's own "error / not supported" reply, with no `#`.

**The key finding: "not supported" isn't always safe.** Where the driver expects a
`#`-terminated string (`:Gc#`, `:GM#`, `:GtH#`, `:%BD#`…), a bare `0` makes it wait 2–5 s
per command and reconnect. Those commands need their real reply, even if it's a dummy value.

## Connection and identification

| Command | Reply | Notes |
|---|---|---|
| ACK (0x06) | `G` | German mount |
| `:GVP#` | `On-Step#` | The product name. The driver sends it twice at connect |
| `:GVN#` | `10.26a#` | The version decides the driver's behaviour: a `10.` prefix = OnStepX, with high precision on |
| `:GVD#` / `:GVT#` | `Sep 08 2026#` / `12:00:00#` | Firmware date and time |
| `:Gc#` | `24#` | Clock format. **A string reply is required** |
| `:GM#` `:GN#` `:GO#` `:GP#` | `Site 1#` … | Site names. **A string reply is required** |
| `:GT#` | `60.16427#` | Tracking rate in Hz (`0` when not tracking) |
| `:Gt#` `:GtH#` | `+40*52:22#` / `+40*52:22.000#` | Latitude, from the GPS once it exists |
| `:Gg#` `:GgH#` | `-014*26:16#` | Longitude, **west positive** (Meade) |
| `:GG#` `:GL#` `:GC#` | `-02:00#`, `HH:MM:SS#`, `MM/DD/YY#` | UTC offset, local time, date |

Probed once and answered "none", which is fine:
- `:FA#`, `:F1A#`…`:F9A#` (focusers) and `:fA#` (rotator): `0`
- `:GX98#`: `N#` (no rotator)
- `:GXY0#` (auxiliary features): `0`

## Status poll (every poll period, ~3 s by default)

| Command | Reply (ours) | Meaning / maps to |
|---|---|---|
| `:GU#` | e.g. `NpEW220#` | Status letters, see below |
| `:GR#` `:GD#` | `HH:MM:SS.SSSS#`, `sDD*MM:SS.SSS#` | High precision, since OnStepX reports it |
| `:Gm#` | `E#` `W#` `N#` | **Pier side.** INDI → `TELESCOPE_PIER_SIDE` → PHD2 |
| `:%BD#` `:%BR#` | `n#` arcsec | DEC/RA backlash. `dec_bl` 250 steps ≈ 1100″ |
| `:GX90#` | `0.50#` | Pulse-guide rate (× sidereal) |
| `:GX95#` | `0#` / `1#` | Automatic meridian flip |
| `:GX96#` | `E#` `W#` `B#` | Preferred pier side (B = best) |
| `:GXE9#` `:GXEA#` | minutes, `n#` | **Minutes past the meridian allowed, east / west.** This is our register window (east limit, tracking stop) |
| `:Gh#` `:Go#` | `-10*#`, `85*#` | Horizon and overhead limits: where the collision limits will live |
| `:GX9A#` | `0` | Temperature. `0` = no weather sensor, so the driver skips `:GX9B#`…`:GX9F#` |

**`:GU#` letters** (OnStepX `Status.command.cpp`), in this order:
- `n` = not tracking; `N` = no GoTo in progress. Tracking = `N` without `n`; slewing = no `N`.
- Park state: `p` not parked, `I` parking, `P` parked, `F` park failed.
- `H` at home, `S` PPS/GPS synced (later, from the GPS), `G` pulse guide active, `g` guide active.
- `a` auto meridian flip.
- `R` PEC recorded; PEC state `/` ignore, `,` ready to play, `~` playing, `;` ready to record, `^` recording.
- Mount type `E` = GEM. Pier side `o` none, `T` east, `W` west.
- Then three digits: pulse-guide rate index, guide rate index, error code.
- Quirk: the INDI driver also reads the pier-side `W` as an old PEC flag and shows "PEC: Autorecord". Real OnStepX has the same issue; it's cosmetic.

`:Gu#` (bit-packed status) exists in newer OnStepX. The driver only uses it if supported, and a
`0` keeps it on `:GU#`.

## Motion and settings (captured)

| Action | Command(s) sent | OnStepX reply | Maps to |
|---|---|---|---|
| GoTo | `:Sr17:30:00.00#` `:Sd+10*00:00.0#` `:MS#` | `1`, `1`, then `0` = accepted (`1`… = error codes) | Same as LX200 `:MS` |
| Sync | `:Sr…#` `:Sd…#` `:CM#` | `N/A#` on success, `En#` on failure | Same as LX200 `:CM` (keeps the branch) |
| Pulse guide | `:Mgn0500#`, `:Mgw0700#` | nothing | `dec::guidePulse` / `ra::guide` |
| Slew rate | `:R4#` | nothing | Manual-move speed |
| Manual move | `:Mn#` … `:Qn#`, `:Me#` … `:Qe#` | nothing | `:Mn/:Me` / `:Q` per axis |
| Abort | `:Q#` | nothing | Stop |
| Tracking off / on | `:Td#` / `:Te#` | `1` = ok, `0` = failed | RA stop / start tracking |
| Track rate | `:TL#` (lunar), then `:GT#` | `1` | Only sidereal is supported: answer `0` for the others |
| Time | `:SG-02:00#` `:SL03:00:00#` `:SC09/26/26#` | `1` each | Clock (GPS later) |
| Location | `:Sg345:33:44.28#` `:St+40:52:22.08#` | `1` each | Site (GPS later) |
| Park / unpark | `:hP#` / `:hR#` | `1` / `0` | No park position yet: answer `0` (fails cleanly) |

**Parser requirements, more than the LX200 front-end handles today:**
- Fractional seconds in `:Sr`/`:Sd`/`:St`.
- **`:Sr17:23:60.00#`:** the driver rounds up to 60 seconds, so a seconds value of 60 must carry over.
- Longitude as `DDD:MM:SS.ss`, 0–360, west positive.

## Implementation notes

- **It replaces the LX200 front-end** instead of sitting next to it: OnStepX is a superset of the
  LX200 set. It is served on 5001 (the old port) and 9999.
- **Also answered** for generic LX200 clients:
  - `:U#` precision toggle: low `HH:MM.T` / `sDD*MM`, default `HH:MM:SS`, highest via `:GRH#`/`:GDH#`
  - `:Gr#`/`:Gd#` target, `:GA#`/`:GZ#` alt/az, `:GS#` sidereal time
  - `:CS#` sync without a reply, `:TQ#` sidereal
- **Meridian limits:**
  - `:GXE9#` = −east limit × 4 (minutes before the meridian allowed on the normal branch)
  - `:GXEA#` = (TRACK_MAX − offset − 180) × 4 (minutes after the meridian on the flipped branch)
  - With −30°: 120 and 92.
- `:MS#` answers `9` (unspecified) without a target, and `:CM#` answers `E6#` if the sync is refused.
- `:GU#` error digit: `7` (hardware fault) while the RA mount isn't connected.

### Added 2026-09-26

| Commands | Reply | Notes |
|---|---|---|
| `:$QZ+#` `:$QZ-#` `:$QZ/#` `:$QZZ#` `:$QZ!#` | nothing | PEC play / stop / record / clear / save (INDI's PEC tab) |
| `:$QZ?#` | `I` `p` `P` `r` `R`, plus `.` when the worm phase is known | Status: ignore, ready to play, playing, ready to record, recording |
| `:VR[n]#` / `:VR#` / `:WR[n,sn]#` | `sn#` / `sn,n#` / nothing | Table entry (register counts), current segment, write an entry |
| `:GXE6#` `:GXE7#` `:GXE8#` `:VW#` `:VS#` | numbers | Counts per sidereal second, per worm turn, segments |
| `:Sh[sDD]#` `:So[DD]#` | `1`/`0` | Horizon (−30..30) and overhead (60..90) limits, enforced on GoTo (`:MS#` → `1`/`2`) |
| `:SXE9,[n]#` `:SXEA,[n]#` | `1`/`0` | Minutes past the meridian: east limit (shifts the register window) and west tracking stop |
| `:GU#` | adds `S`, `R` and the PEC state character | GPS time sync, PEC recorded, `/,~;^` |

### Added 2026-09-26 (2)

| Commands | Reply | Notes |
|---|---|---|
| `:TQ#` `:TL#` `:TS#` `:TK#` `:ST[H.H]#` | `1` | Sidereal / lunar / solar / King / custom rate (Hz). `:GT#` reports it |
| `:Tr#` `:Tn#` `:T1#` | `1` | Refraction on / off, single axis. `:T2#` (dual axis) and `:To#` answer `0` |
| `:SX90,[n.n]#` | `1`/`0` | Guide rate for both axes, 0.1–0.9× (non-standard set; `:GX90#` reports it) |
| `:$BD[n]#` `:$BR[n]#` | `1`/`0` | DEC backlash in arcsec. RA accepts only 0 |
| `:GXEe#` `:GXEw#` `:GXEB#` `:GXEC#` `:GXED#` | `n#` | RA axis limits (axis hour angle, deg; `B` in hours), DEC mechanical limits |
| `:SXEe,n#` `:SXEw,n#` `:SXEC,n#` `:SXED,n#` | `1`/`0` | Set them (non-standard in OnStepX, where they are compile-time) |
| `:MS#` | adds `6` | Target outside the DEC axis limits |
| `:GU#` | `rs` with refraction on, else `(` `O` `k` for lunar / solar / King | |

## Next

1. Ekos + PHD2 across a real flip, to check that PHD2 flips its calibration from the pier side.
2. NINA through the ASCOM OnStep driver.
3. When the GPS is wired: check INDI's GPS NMEA driver on `starmount.local:10110`, and GPS time in the field (no NTP).
