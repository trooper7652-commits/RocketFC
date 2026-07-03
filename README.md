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
- Two pyro channels: backup parachute + landing-motor igniter, with
  continuity sensing and multiple safety interlocks
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
| Pyro fire — chute | 6 | MOSFET gate, low-side driver |
| Pyro fire — landing motor | 7 | MOSFET gate, low-side driver |
| Continuity sense — chute | 14 (A0) | voltage divider across e-match |
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
- Pyro channels: logic-level MOSFETs, low-side, **gate pulldown resistors**
  (so a floating pin during boot can't fire), flyback-safe wiring, and a
  **physical arm switch in series with pyro battery power** — software
  interlocks are the second layer, not the only layer.
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
  rides it out.
- **ABORT** (tilt > 30°, IMU failure, KF unhealthy, missed window, dud):
  permanently inhibits the landing motor and fires the chute (waiting for
  motor burnout if aborting during BOOST).

**Modes** (`mode chute` / `mode land`, persisted in EEPROM):
`CHUTE_TEST` flies the full TVC ascent and pops the chute at apogee —
this is how you validate everything before risking a landing attempt.
`FULL_LANDING` attempts the propulsive landing; the chute remains armed as
the abort recovery.

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
| `pyrotest 1|2` | fire a pyro channel on the bench — two-step typed confirmation, disarmed only, **no motors/matches connected** |

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
4. Rig motors + e-matches **last**, on the pad, pyro arm switch OFF.
5. Vehicle vertical on the rail, still. `arm` from a laptop, or arm switch.
   Pre-arm checks verify sensors, SD, battery, continuity, and tilt < 5°.
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
`tools/replay` (unit checks + 5 synthetic flight scenarios) on every change:
`cd tools/replay && make test`.*
