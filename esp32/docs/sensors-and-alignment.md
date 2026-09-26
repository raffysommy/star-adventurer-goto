# Sensors, pose awareness and polar alignment

Design notes: what GPS and two IMUs can add to the ESP32 mount controller, what they
can't, and how they fit with the motor counts and plate solving. Nothing here is
implemented yet. See [README](../README.md) for what exists today.

## Goals

- **Pointing awareness:** a mount that knows where it is pointing, after assembly, after hand moves, on either side of the pier.
- **Fast manual pointing:** move it by hand (the RA motor tops out at ~0.28°/s, so 90° takes over 5 min) and let the ESP work out where it is.
- **Quick polar alignment good enough for guiding,** without the slow Ekos procedure. Precision on demand.
- **Never disturb tracking or guiding.**
- **Don't modify the Star Adventurer:** everything attaches outside (its USB port, the DEC knob belt, the battery bay).

## Constraints

- **The rig is disassembled for every trip.**
  - Nothing mechanical survives transport, so there's no fixed home and no park position to remember.
  - Only calibrations of *rigid* parts carry over.
- **Camera:** Canon M50 on a 70–200 at 200 mm. APS-C gives ~6.4° × 4.3° at ~3.8″/px.
  - The lens is held by its **tripod collar**, whose foot is screwed tightly to the DEC bracket's moving plate (minimal yaw).
  - The camera and lens can rotate *inside* the collar ring.
- **Clutches:** both can be released (RA on the Star Adventurer, DEC on the bracket), so both axes can be moved by hand.
- **Imaging host:** a Pi running INDI/Ekos, with ASTAP available for plate solving.
- **Polar alignment is manual:** the altitude and azimuth knobs have no motors, so "self alignment" means *guided* adjustment. Only motorised adjusters would make it automatic.
- **Power:** a USB-C power bank with a PD trigger set to 9 V.
  - 9 V feeds the EasyDriver directly.
  - A step-down converter feeds 5 V to the ESP, the Star Adventurer (through the OTG port's VBUS), the GPS and the IMUs.

## Hardware and placement

| Part | Where | Role |
|---|---|---|
| ESP32-S3, EasyDriver, GY-GPS6MV2 (u-blox NEO-6M), **LSM6DSV + QMC6309** magnetometer (one breakout) | **Base box**, ideally a keyed 3D-printed insert in the (empty) battery bay. That's the fixed body of the Star Adventurer, which tilts with the polar axis | Controller, time and site, polar-axis altitude and heading |
| **MPU6500** (6-axis) | **Camera hot shoe** | Mechanical pose of both axes, plate-solve-calibrated |

### IMU choices

- **Base: LSM6DSV + QMC6309.**
  - ~5× lower accelerometer noise and much lower temperature drift than the MPU6500, which keeps the magnetometer's tilt compensation accurate. Needed for sky-less polar alignment (T0 below).
  - A current part from real suppliers.
- **Camera: MPU6500 is enough.**
  - Its temperature drift (~0.5–1 mg/°C, up to ~0.5° over a night) is compensated with **its own on-chip temperature sensor**.
  - The model is learned from plate solves, and every solve re-anchors it.
  - The LSM6DSV's temperature wouldn't help: it measures the base box, not the lens.
- **Avoid the MPU9250.** It's discontinued, most boards sold as MPU9250 are clones or relabelled MPU6500/9255 dies without a working magnetometer, and even genuine ones have MPU6500-class accelerometers.

### Camera IMU placement: hot shoe vs collar ring

| | Hot shoe | Collar ring / foot (keyed clip) |
|---|---|---|
| IMU ↔ camera | Fixed. **Every plate solve calibrates the IMU directly** (offsets, temperature model) | Changes when the camera is rotated in the collar |
| IMU ↔ DEC axis | Changes with the collar rotation. **Re-measured per session by a short DEC move** (~10° in 10 s, gyro or two solves) | Fixed, calibrated once |
| Single-picture PA (T1) and IMU learning | Direct | Needs the camera ↔ collar roll per session |

**Decision: hot shoe**, with an automatic short DEC move at startup.
- If the camera is rotated in the collar during the night, the gyro sees a rotation about the optical axis while the motors are idle, and the ESP re-measures.
- The cable crosses both axes either way: leave a loose loop with slack for DEC passing the pole.

### Other hardware notes

- **EasyDriver stays.** It's disabled almost all night (a few slews, short guide pulses), so heat and power are not a concern, and the camera has never slipped with DEC unpowered.
  - A TMC2209 would be optional: silent, cool, holding current, StallGuard for belt-slip or collision detection.
  - Keep the IMU away from the driver chip anyway.
- **Magnetometer environment:** the EasyDriver's coil currents, the Star Adventurer's RA motor, the ESP, and the DEC motor (**permanent magnet**) and counterweight bar, which rotate with RA.
  - Sample only with the DEC driver disabled, at a fixed reference RA angle.
- **GPS:** the patch antenna faces the sky, away from the ESP's antenna and the driver.
- **Travel connectors:** latching and impossible to plug in wrong.
  - RJ12 (6-wire, like the ST4 cable) for the camera IMU.
  - JST or small aviation connectors for the DEC motor and power.
- **Box extras:** an RGB status LED (tracking, slewing, guiding, error, GPS fix) and a physical **STOP** button that stops both axes without needing Wi-Fi.
- **Not planned:**
  - hall sensors / home switches (the IMUs replace "home", and they'd need magnets on the mount)
  - opening the Star Adventurer
  - motorised alt/az adjusters: only for truly automatic alignment, as a separate unit under the mount

### Electrical plan (proposed pins)

| Function | Bus | Pins |
|---|---|---|
| GPS, NMEA 9600 | UART1 | ESP RX 17 ← GPS TX, ESP TX 18 → GPS RX (TX optional, for configuring it) |
| LSM6DSV + QMC6309 (base, short wires) | I2C0, 400 kHz | SDA 8, SCL 9 |
| MPU6500 (camera, long cable) | **I2C1 (its own bus)**, 100 kHz | SDA 1, SCL 2; 2.2–4.7 kΩ pull-ups at the ESP end; twist SDA with GND if possible |
| Already used | | USB 19/20, DEC 5/6/38 |
| Avoid | | 0/45/46 (strapping), 35–37 (octal PSRAM), 43/44 (UART0) |

- **Why the camera IMU gets its own bus:** a long cable crossing both axes is the most likely thing to glitch or hang an I2C bus. On its own bus, a flaky or unplugged camera IMU can't take the base sensors down.
- **Firmware must:** recover the bus and handle hot-unplug.

## Power: the power-bank risk

PD power banks can:
1. **Switch off at low load**, typically below ~50–150 mA. At night the EasyDriver is off and the ESP plus a tracking Star Adventurer draw little at 9 V, close to that threshold.
2. **Briefly reset all ports** when another device is plugged or unplugged, or when the charge negotiation changes.

Either one is a **power loss**, which is worse than a reboot:
- **The Star Adventurer's RA register restarts at 0**, and today's "fresh mount" logic would assume home (HA 2°), **silently wrong**.
- **The DEC position is lost.** It only survives soft resets, because it's kept in RAM.

**Mitigations:**
- Give the mount its own bank (or at least its own port), and don't charge a phone from it.
- Use the bank's low-current / always-on mode if it has one; otherwise add a small constant load (e.g. the status LED) to stay above the threshold.
- **Firmware:** detect a power-on reset mid-session and **mark the position untrusted** ("power lost: re-sync") instead of assuming home. With the camera IMU, the pose is recovered automatically.

**5 V rail:** the Star Adventurer's motor current flows through the step-down and the dev board's OTG pad trace.
- The buck should deliver at least 2–3 A.
- Measure the 5 V at the board during a RA slew once. It should stay above ~4.8 V.

**Suspected cause of the unexplained offline event on 2026-09-23:** the ESP stayed unreachable for minutes. A firmware hang would have tripped the task watchdog and rebooted within ~15 s, so a power cut is more likely. `/log` shows the reset reason on the next boot (1 = power-on).

## What each sensor physically gives

| Sensor | Gives | Realistic accuracy | Limits |
|---|---|---|---|
| GPS (NEO-6M) | UTC time, latitude and longitude, offline; **magnetic declination** via the World Magnetic Model | Time ±0.1 s from NMEA (= 1.5″ of RA); declination ±0.3–0.5° | Needs sky view; first fix takes a while. PPS is on the board's LED if ever needed |
| Accelerometer | Direction of gravity = **2 of 3 orientation angles**, absolute, no drift | LSM6DSV ~0.05–0.1°; MPU6500 ~0.1–0.2° with temperature compensation | Can't see rotation about the vertical (azimuth) |
| Gyro | Rotation while moving (hand slews, the direction of a rotation axis) | Good over seconds | Drifts, so short-term only |
| Magnetometer | Heading | ~0.5–1° absolute with care (error budget below); ~0.1–0.2° *relative* over minutes | Dominated by the rig's iron and currents, not by the chip |

### Magnetometer heading: error budget

| Source | Size | Mitigation |
|---|---|---|
| Magnetic vs true north | ~4° here | GPS position + World Magnetic Model: ±0.3–0.5° |
| Tilt compensation | Accelerometer error × ~1.5 (the field dips ~56° at this latitude) | LSM6DSV: ~0.1–0.15° (the MPU6500 would give ~0.5°) |
| Fixed rig iron (Star Adventurer body and RA motor, EasyDriver, ESP) | Several degrees | Hard/soft-iron calibration of the **fully assembled rig**, turned through a full circle on the ground once. Leaves ~0.3–0.5° |
| Parts rotating with RA (counterweight bar, lens, NEMA17 magnet) | Varies with the RA angle | Always measure at the same RA angle, with the EasyDriver off |
| Tripod/wedge steel moving when the azimuth knob turns | Small | Mostly negligible |
| Box heading axis vs polar axis | Fixed offset | Calibrated once against a plate-solve alignment (T3). Stable thanks to the keyed battery-bay insert |

**Total: ~0.5–1° absolute**; better relative while turning a knob.

## Polar alignment accuracy actually needed (200 mm, guided)

| Polar alignment error | Max DEC drift | Field rotation at the frame corner (~3.9° off-centre) in a 2-min sub | Verdict |
|---|---|---|---|
| 1′ | ~0.3″/min | negligible | ideal |
| 0.5° | ~8″/min | ~1″ (< 1 px at 3.8″/px) | **workable**: PHD2 handles the drift |
| 1° | ~16″/min | ~2″ | borderline, OK for 1–2 min subs |

Stacking handles rotation *between* subs. Longer focal lengths or longer subs need the precise tiers below.

## Polar alignment tiers

| Tier | Needs | Estimated accuracy | Time |
|---|---|---|---|
| **T0: no sky** | Base IMU (altitude) + magnetometer + GPS declination (azimuth) | 0.5–1° | ~1 min, live feedback on **both** knobs |
| **T1: single picture** | 1 solve + both IMUs + mechanical model (DEC axis in camera frame from the startup DEC move) | ~0.3–0.6° | Free at the first solve; re-checked at every later solve |
| **T2: two pictures, DEC move** | 2 solves ~10° apart **in DEC (the fast axis)** + base IMU altitude + one-time bracket calibration | ~0.1–0.2° | ~20 s |
| **T3: two pictures, RA move** | 2 solves + the exactly known RA rotation | ~1′ | ~1 min |

T1 and T2 are estimates on paper and need measuring. The magnetometer's remaining offset is **learned from every T3 run** and stored, so T0 improves from session to session.

### Why a single picture alone isn't enough (T1)

- **What a solve gives** (with GPS time and site): the camera's full orientation, including which way it faces.
- **What we need:** which way the *base* (the polar axis) faces.
- **What gravity gives:** the relation between camera and base in tilt, but **not in rotation about the vertical**. That's exactly the azimuth we're after.
- **How it's closed:** the mechanical chain (RA rotation → DEC rotation → camera on the DEC axis). With the camera's mounting on the DEC axis known (measured by the startup DEC move), the camera IMU gives the RA and DEC angles. That fixes camera ↔ base completely, and one solve gives the polar axis in the sky.
- **Accuracy:** limited by the IMUs and the mechanical model (RA/DEC non-orthogonality, cone error). Hence ~0.3–0.6°.

### T2: second picture via DEC

- **Measurement:** two solves ~10° apart in DEC give the DEC axis on the sky precisely.
- **Constraints on the RA axis:** it is perpendicular to the DEC axis (the bracket's small non-orthogonality is calibrated once against T3), and its altitude comes from the base IMU.
- **Result:** at most two solutions, and the IMUs pick the right one.
- **Why it's fast:** DEC moves at ~1°/s, so no slow RA rotation is needed.

### T3: second picture via RA (precise)

- **Why Ekos is slow:** its polar alignment treats the mount as a black box and solves 3 frames about 30° apart, roughly 60° of RA travel. That's ~3.5 min of motion at ~0.28°/s, before the adjustment loop.
- **What we use instead:** the ESP knows the RA rotation between two frames to ~0.1″ (12.5 M counts per revolution).
- **The method** (from any starting pose):
  1. Take frame 1 and plate-solve it.
  2. The ESP rotates RA by an exactly known angle Δ (5–10°).
  3. Take frame 2 and plate-solve it (with the first solve as a hint).
  4. The image centre moved along a small circle around the RA axis. Two points plus the known Δ give the circle's centre, which **is the RA axis on the sky**.
  5. The error is the vector from that axis to the celestial pole, with refraction applied.
- **Precision:** solves at 3.8″/px are good to ~1–2″. With Δ = 5° the axis comes out at **~1′**; periodic error adds up to ~1′.
- **Time:** 18–36 s of rotation, plus two 2–4 s exposures and two ASTAP solves. **~1 min.**
- **Special case:** if the camera points near the pole, matching stars between the frames gives the rotation's fixed point even more precisely.

### Adjustment feedback while turning the knobs

- **Altitude knob:** the base IMU, instant and live.
- **Azimuth knob:**
  - the magnetometer, instant (relative ~0.1–0.2°), or
  - images: 1–2 s exposures, re-solved with a hint or tracked by star shift, every few seconds (Ekos "refresh" without the slow part).
- **End:** one confirmation solve.

### Where it runs

- **Pi:** a companion Python tool. It captures frames through INDI (to confirm: the M50 via the GPhoto driver) and solves them with ASTAP. It drives the ESP through a new `/api/pa` endpoint: rotate RA or DEC by an exact amount and report the counts.
- **ESP dashboard:** results and live knob feedback.
- The ESP never handles images.

### Practical notes

- Tape the 70–200's zoom ring: zoom creep at steep angles changes the image scale between frames.
- At 70 mm the field is ~18°×12°: solving is easier, but T3 comes out ~3× coarser (~3′).
- **Quick check before building anything:** lower the rotation in Ekos's polar alignment from 30° to 10°.

## Pose awareness: "where the mount is"

### 1. Mechanical pose: always, from power-on

- **What's measured:** camera IMU + base IMU give the RA angle (counterweight), the DEC angle, pier side and over-the-pole, at ~0.5°. No sky, no alignment.
- **This replaces "home":** no more assuming HA 2° / DEC 0° at power-on.
- **Two candidate poses:** for some orientations gravity allows two mechanical solutions.
  - They are always far apart.
  - The gyro's motion history picks the right one, or failing that "counterweight roughly down" at power-on.
  - The ESP flags the rare cases it can't resolve.

### 2. Sky pointing: depends on how well the polar axis is known

| Stage | Polar axis known from | Sky pointing accuracy |
|---|---|---|
| Just assembled | T0 (base IMU + magnetometer + GPS) | ~1° |
| After polar alignment | T1–T3 | ~0.5–1°, live, everywhere |
| After the first plate solve of the night | Camera IMU re-anchored | ~0.3–0.5°, re-anchored at every solve |

### Calibrations

| Calibration | How | How often |
|---|---|---|
| Base IMU → polar axis | Rotate RA 60–90° and fit the cone traced by the camera IMU's gravity vector (its axis is the polar axis). Compare with the base IMU at the same moment | Once, while the box sits in its keyed insert |
| Camera IMU → camera (offsets, temperature model) | Every plate solve: predicted vs measured gravity in the IMU frame | Continuous, learned over nights |
| DEC axis in the camera frame | Short DEC move at startup (gyro, or two solves) | Each session, and when the camera is rotated in the collar |
| **DEC backlash and scale** (auto) | The same startup move, ±1–2°. After a reversal, the motor steps counted until the gyro sees the camera turn = backlash (at 100–200 steps/s, well above gyro noise; repeat and average). Rotation vs steps = steps/°. Steps with no rotation beyond the backlash = slip | At startup for a first value, then **at every GoTo**. The DEC approach always ends with a small reversal (overshoot ~100 steps), done after RA has arrived and at ~200 steps/s: zero extra moves, measured in each target's pose and load, refreshed after flips. Use only the gyro component about the DEC axis. Keep a running average with outlier rejection; accept only 100–500 steps. Never from guide pulses (below gyro noise) or while guiding |
| RA/DEC non-orthogonality (bracket) | Against a T3 run | Once |
| Magnetometer (hard/soft iron) | Fully assembled rig turned through a full circle once, at a reference RA angle | When the box layout or site changes |
| Magnetometer heading offset | Learned from each T3 run | Continuous |

### Which side of the pole: one solve is enough

The 360° DEC axis reaches every patch of sky on two branches: normal, or with DEC swung past the pole (after a meridian flip).
- The two branches see the same sky **rotated by 180°**, so a single plate solve's field rotation tells them apart.
  - On 2026-09-25 image +x pointed at position angle ≈ 1.3° on the normal branch. The other branch would read ≈ 181°.
  - Remounting the camera a few degrees off each trip doesn't matter against 180°.
- **Calibration:** once, the camera's rotation on the bracket, from a solve on a known branch.
- **Check at every sync:** if the solve's rotation disagrees with the branch the ESP assumes, refuse the sync and say so. That's exactly the old beyond-the-pole failure, caught before the first wrong GoTo.
- **Without sky:** the camera accelerometer does the same. On the two branches, for the same pointing, the camera is rolled 180°, so gravity sits on opposite sides of the camera frame.
- The DEC-move test (the sign of ΔDec for +steps) also works, but costs a move and two solves.

## RA register window (implemented 2026-09-26, east limit still 0 until the clearance check)

**Today:** register = HA + 2° (`OFFSET`, from `offset_star_adventurer`), and GoTo/sync are limited to register 2°–183°.
- This keeps clear of both register failures: below 0 tracking stalls, and above 241.7° the 24-bit value overflows.
- The consequence is that only the western sky is reached directly. Everything east of the meridian is a meridian flip (RA +180°, DEC 180 − Dec).
- The narrow window was chosen when a Raspberry Pi drove the mount over USB, for fear of overflow if the Pi disconnected while the mount kept tracking.

**Why it can move now:**
- The ESP is on the mount itself, sees the register continuously, and enforces limits at every poll.
- If the ESP dies, the mount keeps tracking on its own at 15°/h. With the west limit kept ~59° below overflow, that still leaves the same ~4 h of margin as today.

**Proposal (chosen: east limit −30°):**

| | Now | Proposed |
|---|---|---|
| Register | HA + 2° | HA + 32° (east limit as a setting; register = HA − east limit + 2°) |
| Normal branch (no flip) | target HA +0° … +181° | target HA **−30° … +150°** (register 2°–182°) |
| Flipped branch | target HA −180° … 0° | target HA −210° … −30° |
| Where the flip happens | at the meridian | 2 h **before** the meridian |
| Margin to overflow at the west limit | 59° (~4 h) | 59° (~4 h) |

**The window must stay 180° wide.**
- The normal branch covers the axis window, and the flipped branch covers the same window shifted by 180°.
- Narrower than 180°, some targets are reachable on neither branch. So the west limit moves with the east limit.

**Behaviour:**
- Every target crossing the sky still needs **one** flip somewhere, since the window is 180°. The offset only chooses **where**, and the flip point is the east limit.
- With −30°, anything you start within 2 h before the meridian, or any time after it, never flips. That covers transit, the best part of the night.
- A target started earlier (more than 2 h east) starts on the flipped branch and flips once, at HA −30° instead of at the meridian.
- The flip itself is unchanged: a real GoTo to the other branch, triggered by the client re-slewing (NINA/Ekos "meridian flip" set to 2 h before the meridian, i.e. −30°).
- **Tracking past the window** (implemented): tracking continues to register 235° and stops there. A flipped-branch target can track from 2 h before the meridian to ~1.5 h after it, and you flip (re-slew) whenever it suits you.
- **Overflow margin:** the mount is powered from the ESP's OTG port, so it can't keep tracking with the ESP dead. The 4 h margin is no longer needed.
- **The register limit could go entirely:** a software offset, with a ~1 s pause between subs to shift the register back near the end of the span. The only limits would then be collisions and cables.
- **Sync keeps the branch** (implemented): `:CM` picks the candidate nearest the current register, so solve → sync → GoTo ("center") never flips the maths without a move.

**Before doing it:**
- **East of the meridian on the normal branch, the camera sits below the RA head** (on a classic mount, the counterweight-up position). This is where the lens can hit the head or a tripod leg.
  - So the east limit (−60°) must be a **setting**, set from a real clearance check with the rig.
  - The collision limits below should exist first.
- **Home at power-on** (register near 0 → "HA 2°") changes meaning. It must be restated for the new offset, or replaced by the IMU pose.
- Update the golden tests (`test/gen_golden.py` uses lx200.py's offset) and the flip logic (`selectTarget`), and document it as a deliberate difference from lx200.py.

## Collision limits

Collisions are geometric: the lens or body against the SA head, the tripod legs, or the base box. They depend on the full mechanical pose (RA angle, DEC angle), not on which branch we're on.

| Layer | How | Needs |
|---|---|---|
| **Configured limits** | Forbidden zones in mechanical coordinates (RA register, DEC motor angle). A GoTo whose path enters one is refused or rerouted | A one-time **teach**: move slowly toward each obstacle, press "mark limit" on the dashboard, and the ESP stores the counts, plus the IMU pose when there is one |
| **Accelerometer** (camera) | Gives the camera's tilt directly: "lens pointing below −10°" or "camera upside down under the head" are simple, absolute rules. It works even when the counts are wrong (after a hand slew, a lost sync, a wrong branch) | Nothing beyond the planned MPU6500. It can't see rotation about the vertical, but the counts or the base IMU supply that |
| **Gyro** (camera): stall or collision detection | During a slew the ESP knows the commanded rate (up to 0.28°/s RA, 870 steps/s DEC). If the gyro reads ~0 while the motor should be turning, something is blocking or slipping: stop within ~0.5 s | The same MPU6500. Only active during slews (guiding rates are far below the gyro noise) |
| **Driver** (optional) | A TMC2209 with StallGuard on DEC | A driver swap |

- **The IMU layers are a safety net for when the model is wrong.** A wrong sync, a wrong branch or a hand slew are exactly the cases where count-based limits fail.
- The stop rules follow "When the IMU may act": stop at once during slews; during tracking, warn first and only stop at a hard limit.

## Hand-slew and push-to workflow

1. Pick a target (e.g. Andromeda) on the phone dashboard.
2. Release the clutches and swing the lens. **Push-to:** the dashboard shows a live arrow and the remaining distance ("12° up, 30° left… 0.5°, lock").
3. Lock the clutches. When the camera IMU is still, the ESP sets its registers to the new pose.
4. Plate solve and sync (the Pi tool); the IMU supplies the pier side.
5. A short motor GoTo centres the target.

## Sensor fusion, authority and interference

### Who's in charge (from most to least trusted)

1. **Plate solve:** precise, absolute. Always wins and re-anchors everything.
2. **Motor counts** (RA register, DEC steps): precise, relative. They run tracking and guiding.
3. **IMUs:** coarse, absolute. Advisory by default.

Guide corrections are arcseconds; the IMUs resolve ~0.2–0.5°. **The IMUs cannot see guiding.**

### When the IMU may act

| Situation | Signature | Action |
|---|---|---|
| Hand slew (clutch open) | Camera IMU rotates fast (≫ 1°/s), base still, motor counts unchanged | After ~2 s of stillness: update the registers to the new pose |
| Camera rotated in the collar | Rotation about the optical axis, motors idle, base still | Re-measure the DEC axis in the camera frame (short DEC move, when not guiding) |
| Slow drift or disagreement | IMU and counts drift apart by a fraction of a degree over minutes | **Nothing to the mount.** Dashboard estimate only; the next solve re-anchors |
| Tripod bumped | **Both** IMUs jump together | Alert: "mount bumped, alignment/sync may be off". No register change |
| Tripod settling in azimuth | T1 estimates drift between solves | Alert only |
| Suspected DEC belt slip | DEC steps vs camera IMU disagree by more than ~1° | Alert only. Never stops guiding by itself |
| Power loss mid-session | Power-on reset while a session was active | Position marked untrusted; recovered from the IMUs, confirmed by the next solve |
| Collision limit | Pose past a limit (lens toward the tripod, counterweight far up) | During **slews**: stop. During **tracking**: warn first, stop only at a hard limit, and only if it holds for over 1 s and agrees with the pose predicted from the counts |

Built-in protections:
- **An RA register rewrite needs the motor stopped.** The Star Adventurer only accepts `:E` when stopped, so RA can't be rewritten mid-tracking. It is only done after a detected hand slew, which interrupted tracking anyway.
- **While guiding is active** (pulses within the last ~minute), every automatic IMU action is disabled except the hard collision stop. Alerts only.
- **An "IMU authority" setting:** *off* / *advisory* (default: estimates, push-to, alerts) / *auto* (adds hand-slew register updates and collision stops).

### Other kinds of interference

- **Electrical** (long cable, EasyDriver noise, I2C glitches):
  - Readings are sanity-checked: gravity ≈ 1 g, plausible gyro rates, the bus responds.
  - A bad sensor is marked **invalid** and the ESP carries on without it.
  - The camera IMU has its own bus.
  - The magnetometer is sampled only while the DEC driver is off and has no authority.
- **Timing:**
  - Sensor reading runs in a low-priority task.
  - The RA control task, the guide-pulse timers and the DEC step pulses (generated in hardware) have priority and are independent.
- **Mechanical** (wind, touching the lens): readings are filtered, and a pose counts only when the mount is still.

**Worst case:** the IMU is wrong. In advisory mode the only symptom is a wrong push-to arrow or a false alert. Guiding is never touched.

### Tests to add

- The existing `test/hil/guide_soak.py` must still give **0 RA motor restarts** with the IMUs running.
- A new HIL test that simulates a tripod bump, a hand slew and a camera rotation in the collar, and checks the reactions against the table above.

## Session flow (target)

1. **Assemble:** counterweight roughly down, roughly north; the box goes into the battery bay.
2. **Power on:**
   - GPS gives time and site.
   - The IMUs give the mechanical pose.
   - A short automatic DEC move measures the camera mounting.
3. **T0 polar alignment:** turn both knobs with live feedback to ~0.5–1°. Often enough at 200 mm.
4. **First solve:** T1 check and camera IMU calibration. Refine with T2/T3 if needed.
5. **Point:** by hand with push-to, or by motor GoTo; plate-solve sync; centre.
6. **Track and guide:** the IMUs are advisory only.
7. **Disassemble:** nothing to save except the rigid calibrations.

## Priorities

Before any sensor work:

1. **Read `/log` about the offline event;** check the power bank's behaviour and the 5 V rail.
2. **Field networking:** decide whether the ESP joins the Pi's hotspot or the Pi joins the ESP's access point. The clock comes from the LX200 client or the browser until the GPS exists.
3. **DEC soft limits** (configurable) and **power-loss detection** (untrusted position after a mid-session power-on).
   - Collision limits by teach (counts first, the IMU later).
   - A branch check from the solve's field rotation at every sync.
   - **RA register window** (HA −30°…+150°) and the stop at the tracking limit: done 2026-09-26. Set the east limit to −30 on the dashboard after a clearance check on **both** sides: camera under the head east of the meridian, and on the flipped branch past it.
   - Log the reset reason to NVS: on 2026-09-26 an OTA attempt apparently power-cycled the mount (it came back fresh after only a software reset), and a power-on wipes the RAM log.
4. **Night test** with INDI "LX200 GPS" + PHD2, watching:
   - the meridian flip (now at the true meridian)
   - the `"0"` reply to `:Mg`
   - `:GG`
   - hours of tracking and guiding
5. **Mount simulator:** a simulated Star Adventurer with its quirks (CW auto-stop, stop after reversal, stall below zero, 24-bit overflow), so the RA state machine and the IMU authority logic can be tested on the PC.

Then the sensor roadmap:

1. **GPS:** offline time, site and declination. It also solves the field clock.
2. **Pi alignment tool** (T3, then T2): INDI + ASTAP + ESP `/api/pa`, including "solve and sync".
3. **Camera IMU (MPU6500, hot shoe):** mechanical pose at power-on, pier side, hand-slew detection and push-to, collision limits, belt-slip alerts, T1.
4. **Base IMU + magnetometer (LSM6DSV + QMC6309):** T0, knob feedback, tripod-bump detection.
5. **Later / optional:**
   - an OpenAstroMount/OAT-compatible LX200 front-end
   - motorised alt/az adjusters
   - a pointing model (cone error from multiple syncs)

## Open questions

- Does the M50 capture through INDI's GPhoto driver in the current Ekos setup?
- How the collar clip or hot-shoe adapter and the camera IMU cable are routed across both axes.
- Battery-bay insert design: does the base box fit in the bay, keyed and repeatable?
- Power bank model: low-current mode, and behaviour when other ports are used?
