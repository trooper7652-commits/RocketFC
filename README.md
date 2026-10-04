# RocketFC — TVC Self-Landing Model Rocket Flight Computer

A complete flight computer for a thrust-vector-controlled (TVC), propulsively
landed model rocket, built for the **Teensy 4.1** with a **BMI088**
accelerometer/gyro and a **GY-63 (MS5611)** barometer.

- 2-axis servo-gimbal TVC with per-axis PID and correct roll decoupling
- Quaternion attitude estimation (gyro-integrated, accel level-init on the pad)
- 3-state Kalman filter (altitude, vertical velocity, accel bias) fusing
  barometer + accelerometer
- Full flight state machine with debounced transitions, backup timers, and
  layered aborts
- Landing-burn ignition timed by a precomputed `h_ignite(velocity)` table
  generated from the Estes F15 thrust curve
- Spring-ejected parachute on a servo latch, with accelerometer
  confirmation and automatic re-cycling of a stuck latch
- Two pyro channels with continuity sensing and multiple safety
  interlocks: landing-motor igniter, and a nichrome landing-leg release that
  burns from 1 s into a confirmed landing burn until touchdown
- 100 Hz CSV logging to the built-in microSD, USB-serial CLI, buzzer/LED
  status, EEPROM-persisted settings, hardware watchdog
- A PC replay harness that compiles the *exact* flight code and tests it
  against synthetic flights — no hardware needed

```
RocketFC.ino          hardware glue: polling scheduler (500/100/50/10 Hz)
src/config.h          every pin, gain, mass, and threshold — start here
src/core/             flight LOGIC, pure C++, shared with the PC test harness
  quat.h              vector/quaternion math
  ahrs.h              attitude estimation
  altitude_kf.h       altitude/velocity Kalman filter
  control.h           TVC PID
  flight_state.h      state machine + aborts
  chute_deploy.h      parachute release: canopy confirm + latch re-cycle
  burn_table.h        GENERATED — landing-burn ignition table
  flight_core.h       wires the above together (single entry point)
src/hw/               Teensy-only drivers: sensors, actuators, logger, CLI...
tools/
  burn_table.py       F15 thrust curve -> burn_table.h  (rerun after measuring!)
  synth_flight.py     generates synthetic test flights
  replay/             PC test harness (g++/MSYS2): make test
  plot_flight.py      post-flight log plots (matplotlib)
```

---

## 1. Hardware & wiring

| Signal | Teensy 4.1 pin | Notes |
|---|---|---|
| Gimbal servo A (body X torque) | 2 | PWM, 50 Hz |
| Gimbal servo B (body Y torque) | 3 | PWM, 50 Hz |
| Parachute latch servo | 23 | PWM, 50 Hz; holds the spring-ejection latch |
| Pyro fire — leg-release nichrome | 6 | MOSFET gate, low-side driver |
| Pyro fire — landing motor | 7 | MOSFET gate, low-side driver |
| Continuity sense — legs | 14 (A0) | voltage divider across the nichrome |
| Continuity sense — landing | 15 (A1) | voltage divider across e-match |
| Battery voltage | 16 (A2) | divider, ratio in `VBAT_DIVIDER` |
| Buzzer | 8 | active buzzer or transistor-driven |
| Arm switch (optional) | 9 | to GND, `INPUT_PULLUP`; enable via `REQUIRE_ARM_SWITCH` |
| BMI088 + MS5611 | 18 (SDA), 19 (SCL) | I2C, 400 kHz, 3.3 V |
| LED | 13 | built-in |

Hardware rules that save rockets:

- **Teensy 4.1 is 3.3 V and NOT 5 V tolerant.** Never feed 5 V servo signals
  or sensor boards with 5 V pull-ups into its pins.
- **Servos get their own BEC/regulator**, never the Teensy's 3.3 V rail. A
  servo stall browning out the flight computer mid-burn = lost rocket.
  Common ground between BEC, battery, and Teensy.
- Parachute latch: design it so the spring **cannot back-drive the servo**
  (e.g. a pin loaded across its axis, not along it). Before the firmware
  starts, the servo gets no signal; a latch that slips when the servo goes
  limp ejects the chute on the pad. The chute servo shares the gimbal
  servos' BEC — check the rail with all three moving at once.
- Pyro channels: logic-level MOSFETs, low-side, **gate pulldown resistors**
  (so a floating pin during boot can't fire), flyback-safe wiring, and a
  **physical arm switch in series with pyro battery power** — software
  interlocks are the second layer, not the only layer.
- Leg-release nichrome: it is on for **~3–4 s** (fire + 1 s → touchdown
  detected), drawing amps the whole time, overlapping the e-match gate by
  0.2 s and the TVC servos' hardest work. Power it from the **pyro battery**,
  never the flight computer's, through a MOSFET rated for that current.
  Touchdown is only declared after 0.8 s of stillness, so the wire keeps
  glowing ~1 s on the ground — keep it clear of anything flammable.
- Mount the IMU rigidly, close to the CG, axes square to the airframe. Set
  `IMU_R_SB` in `config.h` to match the mounting orientation.
- Barometer: open-cell foam over the MS5611 port, in a vented bay, shielded
  from sunlight (it is light-sensitive) and from ram/exhaust airflow.
- Use fast metal-gear micro servos (speed matters more than torque for TVC),
  and landing legs with some compliance — expect 1–3 m/s residual touchdown
  velocity even with perfect timing.

## 2. Toolchain setup

**Arduino IDE 2.x** (for uploading):
1. Install Arduino IDE 2.x.
2. File → Preferences → *Additional boards manager URLs*, add:
   `https://www.pjrc.com/teensy/package_teensy_index.json`
3. Boards Manager → install **Teensy**.
4. Library Manager → install **Bolder Flight Systems BMI088**.
5. Open `RocketFC.ino`, select **Teensy 4.1**, Verify/Upload.

Servo, SD/SdFat, EEPROM, and Wire ship with the Teensy core. The MS5611
driver is written in-project (`src/hw/sensors.h`) — no library needed.

**Command line** (what was used to develop this):

```
arduino-cli compile --fqbn teensy:avr:teensy41 RocketFC
```

**PC test harness** (needs g++ and Python 3):

```
cd tools/replay
make test        # regenerates synthetic flights and runs all assertions
```

## 3. Before you fly anything: measure and replace placeholders

Everything marked `[MEASURE]` in `src/config.h` is a placeholder. The
important ones:

| Parameter | How to measure |
|---|---|
| `MASS_PAD_KG`, `MASS_DESCENT_KG` | kitchen scale, fully loaded / after ascent burnout |
| `ASCENT_BURN_MS` | your ascent motor's datasheet or static test |
| `IGNITION_DELAY_MS` | **static-test the F15 igniter chain**: time from fire command to first thrust. This number directly moves the ignition altitude. |
| `CDA_M2` | estimate from a drop test or flight log velocity decay |
| Servo trims, `US_PER_GIMBAL_DEG`, signs | bench: CLI `trim`, `servotest`, direction test below |
| `CHUTE_LOCK_US`, `CHUTE_RELEASE_US` | bench: CLI `chute us <n>` until the latch is fully closed / fully open |
| `CHUTE_RECYCLE_LOCK_MS` | bench: time the servo takes to swing release → lock (`chute cycle`) |
| Leg release timing | bench: `pyrotest legs`, time until the band parts; put it in the sim's "Legs: band cut" field and check the margin (see §4) |
| `VBAT_DIVIDER`, `CONT_DIVIDER_RATIO` | multimeter vs. `status` readout |

Then regenerate the burn table with your measured values:

```
python tools/burn_table.py     # rewrites src/core/burn_table.h
cd tools/replay && make test   # confirm the logic still passes
```

**Servo direction test (do this or the rocket flies into the ground):**
power up on the bench, run `cal`, then tilt the nose toward body +X.
The gimbal must deflect so the *thrust vector pushes the tail back under
the nose* (i.e. it fights the tilt, not amplifies it). If a channel moves
the wrong way, flip `SERVO_A_SIGN`/`SERVO_B_SIGN`. Repeat for +Y.

## 4. Flight states

```
IDLE -> ARMED -> BOOST -> COAST -> APOGEE -+-> DESCENT -> LANDING_BURN -> TOUCHDOWN
                                           |   (FULL_LANDING mode)
                                           +-> DESCENT_CHUTE -> TOUCHDOWN
                                               (CHUTE_TEST mode, or any abort)
```

- **ARMED**: gyro/level/baro calibrated, log open, launch detection live
  (>1.5 g longitudinal for 80 ms).
- **BOOST**: TVC active with boost gains. Burnout on <0.35 g for 150 ms,
  backed up by a max-burn timer.
- **COAST**: servos centered. Apogee on KF velocity ≤ 0 (200 ms), backed up
  by altitude-drop and absolute timers.
- **DESCENT**: fires the landing motor when KF altitude falls to
  `h_ignite(descent rate)` from the burn table — but only if tilt < 15°,
  the KF is healthy, and igniter continuity is present.
- **LANDING_BURN**: TVC active with landing gains. Confirms ignition by the
  accel jump; a dud igniter aborts to chute. No aborts once burning — TVC
  rides it out. `LEGS_DELAY_MS` (1 s) after the fire command — and only into
  a confirmed burn — the leg-release nichrome comes on and stays on until
  touchdown is detected (`LEGS_BURN_MAX_MS` is an independent cutoff). The
  legs never deploy in CHUTE_TEST or after an abort. **Timing is tight:** in
  the closed-loop sim the vehicle reaches the ground ~2.9 s after the fire
  command, leaving ~1.9 s for the band to part and the legs to swing down.
- **ABORT** (tilt > 30°, IMU failure, KF unhealthy, missed window, dud):
  permanently inhibits the landing motor and releases the chute (waiting for
  motor burnout if aborting during BOOST).

**Parachute release** (`src/core/chute_deploy.h`): the chute is pushed out by
a spring; the servo on pin 23 holds the latch. On release, the flight
computer watches the accelerometer for the canopy (≈0 g in freefall, ≈1 g
plus an opening spike under a canopy). If nothing shows within
`CHUTE_CONFIRM_MS` (1.5 s), it swings the latch back to LOCK for
`CHUTE_RECYCLE_LOCK_MS` and releases again — up to `CHUTE_MAX_RELEASES` in
total — then holds it open for good. Re-cycling a latch whose chute is
already out is harmless, so the detector errs on the strict side; a canopy
that opens near apogee (no airspeed yet) can show up late and cost one
needless re-cycle. Jolts below `TOUCHDOWN_ALT_M` never count, so a crash is
never logged as a good chute. Events in the log: `CHUTE_RELEASE` (every
release), `CHUTE_DETECTED`, `CHUTE_UNCONFIRMED`; the `pyro` column's bit 0 is
the latch position and the `cont` column's bit 0 is "canopy detected".
Each re-cycle costs ~2 s of fall, so on a low flight there is only time
for one or two.

**Modes** (`mode chute` / `mode land`, persisted in EEPROM):
`CHUTE_TEST` flies the full TVC ascent and pops the chute at apogee —
this is how you validate everything before risking a landing attempt.
`FULL_LANDING` attempts the propulsive landing; the chute is released only
as the abort recovery.

## 5. CLI (USB serial, 115200)

| Command | Action |
|---|---|
| `help` / `status` | command list / one-shot health & sensor readout |
| `stream` | toggle 10 Hz live telemetry |
| `cal` | gyro bias + level calibration (keep still ~4 s) |
| `zero` | baro ground reference + noise measurement |
| `arm` / `disarm` | pre-arm checks, then ARMED (launch detect live!) |
| `mode chute` / `mode land` | select flight mode (persisted) |
| `trim a +20` / `center` | servo trim (persisted) / center servos |
| `servotest` | slow gimbal sweep for the direction test |
| `chute` | show the parachute latch position |
| `chute open` / `chute lock` | open the latch to load the spring / lock it (arming is refused until locked) |
| `chute us <n>` | jog the latch servo to find `CHUTE_LOCK_US` / `CHUTE_RELEASE_US` |
| `chute cycle` | one in-flight re-cycle: LOCK dwell, then RELEASE — tests freeing a sticky latch |
| `pyrotest land` | fire the landing e-match channel on the bench — two-step typed confirmation, disarmed only, **no motor/match connected** |
| `pyrotest legs` | burn the leg-release nichrome for 4 s (its longest in-flight burn) — two-step confirmation, disarmed only |
| `stop` | every pyro output off immediately |

## 6. Beeps & LED

| Pattern | Meaning |
|---|---|
| Boot chirp | power-on |
| Slow single blink/beep | IDLE, healthy |
| Repeating N beeps in IDLE | fault: 1=IMU 2=baro 3=SD 4=battery |
| 2 beeps repeating | ARMED, CHUTE_TEST mode |
| 3 beeps repeating | ARMED, FULL_LANDING mode |
| Silence | in flight (nothing to say, everything logged) |
| Loud repeating chirp | TOUCHDOWN — locate beacon |

## 7. Launch-day checklist

1. Fresh battery; `status` shows vbat > `VBAT_MIN`.
2. SD card inserted; `status` shows SD OK.
3. `mode` set correctly (listen for the 2-vs-3-beep pattern at arm).
4. Load the chute: `chute open`, spring + chute in, `chute lock`. Strap the
   legs with a fresh band over the nichrome. Then rig motors + e-match
   **last**, on the pad, pyro arm switch OFF.
5. Vehicle vertical on the rail, still. `arm` from a laptop, or arm switch.
   Pre-arm checks verify sensors, SD, battery, chute latch locked, landing
   e-match and legs nichrome continuity (FULL_LANDING), and tilt < 5°.
6. Clear the area. Launch when ready — everything from here is autonomous.
7. After recovery: `flight_NNN.csv` from the SD card →
   `python tools/plot_flight.py flight_NNN.csv` and review before any tuning.

## 8. Tuning guide

- **PID (`GAINS_BOOST`, `GAINS_LANDING`)**: start on a static test stand or
  a tethered/gimbaled mount. Raise `kp` until the response is crisp but not
  oscillating, add `kd` to damp, keep `ki` small (it exists to trim steady
  thrust misalignment). The landing gains act on a lighter vehicle —
  expect them ~30% hotter. Replay real flight logs through `tools/replay`
  to sanity-check changes before flying them.
- **Kalman filter**: `R` is measured from pad noise automatically at arm.
  `KF_SIGMA_ACCEL` is the knob: bigger = trust the baro more (noisier but
  no drift), smaller = trust the accel more (smoother but drifts).
  The logged `innov` column should look like zero-mean noise; sustained
  offsets mean the filter is mistuned.
- **Burn table margin**: after a landing attempt, compare the logged
  ignition altitude/velocity against the table's prediction and adjust
  `BURN_TABLE_MARGIN_M` (positive = ignite higher/earlier).

## 9. Reality check: what dominates landing success

The F15 is a fixed-impulse solid motor — it cannot throttle. The physics
(from `tools/burn_table.py` with a 0.95 kg descent mass):

- Only apogees roughly **15–80 m** arrive at the ignition window with an
  energy the F15 can cancel. Size the ascent motor accordingly.
- **±10% motor-to-motor thrust variation → ~9–15 m/s touchdown error** if
  uncompensated. This — not the software — is the hard part. Mitigate with:
  legs that absorb residual velocity, fresh motors from the same pack,
  static-tested ignition delay, and iterating on logged flights.
- ±150 ms of unmeasured ignition delay costs similar error. Measure it;
  don't guess it.

Fly the incremental program: **bench tests → static TVC stand → CHUTE_TEST
flights (validate AHRS/KF/TVC/logging) → landing attempts**, reviewing the
log after every single flight.

## 10. Safety & legal

E-matches and rocket motors are energetic devices. Never work on the pad
with the pyro battery connected; keep the physical arm switch off until the
pad is clear. Propulsive-landing TVC flights generally fall **outside**
NAR/TRA club safety codes — fly under FAA Part 101 hobby rules on private
land with generous clear distances, or coordinate explicitly with your club.
You are responsible for complying with your local regulations.

---

*Built with Claude Code. The core flight logic in `src/core/` is verified by
`tools/replay` (unit checks + 6 synthetic flight scenarios) on every change:
`cd tools/replay && make test`.*
