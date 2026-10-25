# tools/hil — static-fire ground test with simulated sensors

A rocket clamped to a stand never "flies". Its IMU reads 1 g and its baro reads 0 m, so the real firmware would never detect launch and never fire the F15. This folder runs the **real RocketFC firmware on the real rocket**, with the sensor data replaced by a recorded simulator flight. This is hardware-in-the-loop (HIL) testing: real computer, wiring, servos, pyros and motors, with fake sensors.

What a run proves:
- the landing e-match fires from the pyro LiPo at the right moment;
- the legs nichrome follows 1 s later;
- the gimbal servos move under real thrust and vibration;
- SD logging and the 500 Hz loop survive a real burn.

It also **measures the real ignition delay** (`IGNITION_DELAY_MS`, still a 500 ms guess). The real IMU keeps recording during the burn, so the vibration onset can be timed against the fire command on the same clock.

What it can't prove: the control loop reacting to real motion. The attitude data is scripted. A dud F15 also isn't caught, because the "ignition confirmed" signal comes from the trace.

> ⚠️ **The HIL build is not flight firmware.** It prints `HIL BUILD - NOT FOR FLIGHT` at boot. Reflash RocketFC before any flight (step 7).

## Pieces

| File | What it does |
|---|---|
| `make_trace.py` | Runs one sim flight and writes `traces/<name>.bin/.json` (the trace), `_sim.csv` (the sim's log) and `_ref.csv` (the expected Teensy log). It also checks that the trace replays to the same events. Presets: `nominal` (powered landing) and `abort` (tilt abort; the F15 must not fire). |
| `make_hil.py` | Builds the HIL firmware from a **fresh copy** of `RocketFC.ino` + `src/` in `build/RocketFC_HIL/`, swapping only `sensors.h` for `firmware/sensors_hil.h`. Your `src/` is never edited, and the copy can't drift. USB type is Dual Serial: port 1 is the normal CLI, port 2 is HIL control. |
| `hil_ctl.py` | Laptop side: `load` a trace, run CLI commands (`cli arm`), `go` with a countdown and cues, `abort`. |
| `compare.py` | Checks the Teensy's SD files against `_ref.csv`: event times, gimbal, loop timing and pyro gates. It also estimates the ignition delay (use `--plot` and confirm it by eye). |
| `test_hil.py` | **Run this first.** Compiles the HIL firmware for the PC with stand-in Teensy libraries (`test/mock`), runs the nominal, abort and operator-abort cases, and checks them with `compare.py`. |

## How the HIL firmware behaves

- **No trace loaded:** identical to the flight build.
- **Trace loaded:**
  - the flight computer sees a still pad;
  - `arm` uses the trace's pad level and baro noise, but still calibrates the real gyro and baro (keep the rocket still);
  - `go` (only while ARMED) plays the trace in real time, paced by the clock rather than by loop ticks.
- **After the trace ends:** a resting rocket.
- **`abort` (or Ctrl-C in `hil_ctl.py go`):** the IMU reports unhealthy. In flight, the core aborts after 200 ms with IMU_FAIL: chute released, F15 inhibited. The flight log then stays open, because the core waits under the chute. Wait 1 s and power off; the log is flushed every 0.5 s.
- **Real-IMU log:** from `go` until 10 s after the trace ends, the real IMU goes to `HIL/IMU_NNN.BIN` on the SD card.

## Procedure

Each stage must pass before the next.

**0. On the PC (no hardware)**
```
python tools/hil/make_trace.py
python tools/hil/make_trace.py --preset abort
python tools/hil/test_hil.py          # all three cases must PASS
python tools/hil/make_hil.py --upload COM7
python tools/hil/hil_ctl.py ports     # shows which COM port is which
```

**1. Bench, no motors, LED loads.** Put an LED + 1 kΩ in place of the e-match and the nichrome. That passes the continuity check and lights when the channel fires.
```
python tools/hil/hil_ctl.py load tools/hil/traces/nominal
python tools/hil/hil_ctl.py cli "mode land"
python tools/hil/hil_ctl.py cli "chute lock"
python tools/hil/hil_ctl.py cli arm
python tools/hil/hil_ctl.py go tools/hil/traces/nominal
```
Then copy `FLIGHTS/flight_NNN.csv` and `HIL/IMU_NNN.BIN` off the card and run:
```
python tools/hil/compare.py --trace tools/hil/traces/nominal --flight <csv> --imu <bin>
```
- **Pass:** `RESULT: PASS`; the landing LED lit at FIRE_LANDING for 1.2 s; the legs LED lit about 1 s later and went off at touchdown.
- Repeat 3 times. Power-cycle between runs, with the chute spring unloaded (the latch boots OPEN).
- Then run the `abort` trace once. The landing LED must never light, and the chute latch must release.

**2. Real e-match and band, no motors.**
- A bare e-match on the landing channel, outdoors: proves the MOSFET and pyro LiPo really light it.
- The real nichrome and rubber band on the legs channel: time how long the band takes to burn (video).

**3. Dress rehearsal on the stand.** Install the motors with no igniters, and keep the e-match outside the motor. Run the whole countdown from the safety distance, using the laptop on an active USB extension cable.

**4. Live: F15 only.** Same as stage 3, but with the e-match in the F15. Run `compare.py --plot` afterwards for the measured ignition delay.

**5. Live: both motors.** `hil_ctl.py go` beeps and prints `>>> T-0 LIGHT THE ASCENT MOTOR NOW <<<`. Light it with the normal launch controller. If you miss the cue by more than ~2 s, don't light it: the F15 fires about 7 s after T-0 regardless, and the stand shouldn't take both burns at once.

**6. Afterwards.**
- Feed the measured delay into `IGNITION_DELAY_MS` and `tools/burn_table.py` only when you decide to. That is a flight-code change.
- A real ignition delay changes the burn table, so rerun the sim studies afterwards.

**7. Back to flight firmware**
```
arduino-cli compile --fqbn teensy:avr:teensy41 RocketFC
arduino-cli upload  --fqbn teensy:avr:teensy41 -p COM7 RocketFC
```
Check that the boot banner has **no** `HIL BUILD` line and that the Teensy shows up as one serial port again.

## Stand and safety checklist

- [ ] Static firing is allowed where you are, and the site is clear. Stay at least the NAR/Tripoli distance for the total installed impulse (30 ft / 9 m minimum for F/G class). Keep an extinguisher and water on hand.
- [ ] The hold-down is rated well above the larger motor's peak thrust, plus the side load at full gimbal deflection. Nose up, with a blast deflector under the tail.
- [ ] Neither motor has an ejection charge (F15-**0**; the ascent motor plugged or without a charge).
- [ ] The chute spring is unloaded, or the nose cone is tethered. FULL_LANDING releases the latch on any abort, and the latch also boots OPEN on every power-up.
- [ ] The legs are free to swing, or you expect the band to burn and the legs to drop.
- [ ] At the rocket: `load` the trace, `cli "mode land"`, `cli "chute lock"`. The pyro LiPo goes in **last**; in this design, plugging it in is the arm.
- [ ] Then everyone walks back to the safety distance. Only then `cli arm` (keep the rocket still for ~4 s) and `go`, from the laptop. Ctrl-C after `go` sends `abort`.
- [ ] After the test, wait for the motors to cool before approaching. Unplug the pyro LiPo first.
