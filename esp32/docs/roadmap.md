# Roadmap (as of 2026-09-26)

What's done, what to test on the sky, what to build next. The details are in
[sensors-and-alignment.md](sensors-and-alignment.md) (MEMS, alignment, collisions) and
[onstep-protocol.md](onstep-protocol.md) (the protocol).

## Done

- **Protocol:** OnStepX-compatible front-end (LX200 superset) on TCP 5001 and 9999, tested with
  the real INDI OnStep driver. It covers:
  - pier side
  - meridian and axis limits
  - backlash
  - PEC
  - tracking rates and refraction
  - GPS time/site (boilerplate)
- **RA:** a configurable register window (east limit −30° set; flip from 2 h before to ~1.5 h
  after the meridian), a tracking stop, and a sync that keeps the branch.
  - The register rate was calibrated: exactly timerHz/T1.
- **DEC:** backlash compensation (250 steps), GoTos finishing northward, 0.5× guide rate, axis limits.
- **PEC:** record from guide pulses, playback, the phase kept in NVS, and the strict start rule
  (never switches on under an active guider).
- **Sky tools** (`tools/sky`): polar alignment from two solves (`pa.py`), mount tests, a live-view
  periodic-error recorder. Measured: ±30″ worm PE (598 s), polar alignment down to ~11′.

## Next clear night (no new hardware)

1. **Re-sync first.** The mount was powered off since, and the DEC NOINIT position is lost
   whenever a firmware build changes the memory layout.
2. **Clearance check before the first GoTo east of the meridian** (east limit −30° is already set):
   - Camera body ~22° below horizontal on the east side and ~17° on the west side.
   - Swing the lens through its whole DEC range.
   - If anything touches: a smaller east limit, plus DEC axis limits on the dashboard.
3. **Ekos + PHD2 with the "LX200 OnStep" driver** (TCP `starmount.local:9999`):
   - Calibrate, then check PHD2's Guiding Assistant backlash against our 250 steps.
   - Go through a real meridian flip, and check that PHD2 flips its calibration from the reported pier side.
4. **Record a real PEC table:**
   - PHD2's Predictive PEC off, guide ≥1 worm turn (10 min), then Record in the INDI PEC tab or on the dashboard.
   - Next night: enable Play before starting guiding, and compare the RA RMS with and without it.
5. **NINA through the ASCOM OnStep driver** (Windows): connect, GoTo, sync, flip.
6. More DEC scale samples: one clean move gave 805 steps/° vs 813.3 nominal.

## Small firmware items (no hardware needed)

- ~~Reset-reason log~~: done 2026-09-27 (`/api/boots`, `/sys`). The "OTA power dips" of 2026-09-26
  were the user powering off. The 2026-09-23 offline event is still unexplained, and the log
  will tell next time.
- ~~Manual-move speeds~~: done 2026-09-27 (`:R0`–`:R9`, `test/hil/move_test.py`).
- PEC phase verification: after a power-on the restored phase is trusted as it is. It should be
  fitted against the first minutes of guiding without PEC (the first target of the night; PEC
  then plays from the next GoTo), and PEC refused if it doesn't match.
- Optional, only if the clearance check allows tracking more than ~1.5 h past the meridian on the
  flipped branch: no register limit at all (a software offset; shift the register after each GoTo).

## When the hardware arrives

Order from the design doc:
1. **GPS (GY-GPS6MV2)** on UART1: RX 17 ← GPS TX, TX 18 → GPS RX, 9600 baud. The firmware is ready:
   - check `/api/status` → `gps`
   - install `indi-gpsnmea` and point it at `starmount.local:10110`
   - test time in the field without NTP (`:GU#` shows `S`)
2. **Pi alignment tool:** turn `tools/sky/pa.py` into the T3/T2 tool (INDI + ASTAP + the ESP), with a live mode for turning the knobs.
   - Add a branch check at every sync from the solve's field rotation (180° apart on the two branches).
3. **Camera IMU (MPU6500, hot shoe)** on its own I2C bus: SDA 1, SCL 2, pull-ups at the ESP end.
   - mechanical pose at power-on and branch detection without sky
   - hand-slew detection and push-to
   - collision layers: accelerometer tilt rules, gyro stall detection during slews, taught limits
   - possibly exposed as OnStep "encoders"
   - **automatic DEC backlash and scale at startup** from the gyro (see the calibration table in the design doc)
4. **Base box IMU + magnetometer (LSM6DSV + QMC6309)**, I2C0 SDA 8 / SCL 9:
   - T0 polar alignment
   - knob feedback
   - tripod-bump detection
   - leveling (the tripod isn't level: turning the base changes the axis altitude)
5. Later / optional:
   - park and home from the IMU pose
   - pointing model from several syncs
   - BME280 for refraction and dew warnings
   - auto meridian flip once the collision limits exist
