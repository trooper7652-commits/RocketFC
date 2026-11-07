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
- [ ] Neither motor has an ejection charge (landing F15-**0**; the AeroTech F10 ascent motor plugged, no charge).
- [ ] The chute spring is unloaded, or the nose cone is tethered. FULL_LANDING releases the latch on any abort, and the latch also boots OPEN on every power-up.
- [ ] The legs are free to swing, or you expect the band to burn and the legs to drop.
- [ ] At the rocket: `load` the trace, `cli "mode land"`, `cli "chute lock"`. The pyro LiPo goes in **last**; in this design, plugging it in is the arm.
- [ ] Then everyone walks back to the safety distance. Only then `cli arm` (keep the rocket still for ~4 s) and `go`, from the laptop. Ctrl-C after `go` sends `abort`.
- [ ] After the test, wait for the motors to cool before approaching. Unplug the pyro LiPo first.

## Live closed-loop bench sim

Everything above plays a **recorded** flight into the Teensy. LIVE mode closes the loop instead. The real Teensy flies a **simulated** rocket in real time over USB: the PC runs the physics and sends fake IMU and baro data, and the Teensy's own gimbal, landing-fire, legs and chute commands steer that simulated rocket. You watch it live in the sim dashboard. Afterwards a pure-PC **twin** flies the same flight with the same firmware built for the PC, and the two are compared.

What it proves that the trace can't: the real flight code, on the real board at its real 500 Hz, **reacting** to the rocket's motion. That covers the control loop, the state machine, the estimator, faults and aborts, with no motors and no rocket.

### What is real and what is simulated

| Real | Simulated (on the PC) |
|---|---|
| Teensy 4.1 running the flight code (HIL build) | the rocket's motion: thrust, drag, wind, chute, legs, ground |
| both gimbal servos and the chute servo, really moving | the IMU and baro the flight code reads (the real ones stay plugged in, unused) |
| SD logging, the 500 Hz loop, USB timing | both motors, and the landing e-match: **its gate pin is never driven** |
| the leg nichrome, **only** if you opt in with "live legs" | the leg nichrome otherwise (timed, but the pin stays off) |

Two words used below:
- The **KF** (Kalman filter) is the Teensy's altitude/speed **estimator**. It blends the noisy baro with the accelerometer to guess height and vertical speed. The plots show its guess against the simulated truth.
- **Stale ticks**: how many 2 ms loop ticks the Teensy flew on an old sensor frame because the PC's reply was late. A few is normal. Hundreds means the link stalled, and after 100 ms the flight code treats the IMU as failed.

### Safety
- In LIVE mode the **landing e-match gate pin is never driven**. FIRE_LANDING is timed and logged, but the pin stays LOW. The PC test checks this on every case.
- The **leg nichrome** is driven only with **live legs**, which you switch on by typing `LEGS` (`live_run.py --live-legs`, or the "Live legs" box in the dashboard). Without it, both pyro channels are simulated. Continuity and pyro volts read 7.8 V while simulated, so `arm` works with nothing connected.
- Nothing on the landing channel during bench runs.
- `reboot` (HIL channel) restarts the Teensy for the next run. It is refused while armed or in flight. After it the **chute latch boots OPEN** (it isn't a power-on reset), so keep the chute spring unloaded.
- The HIL build is not flight firmware (step 7 above).

### Setup (once per bench session)
```
python tools/hil/make_hil.py --profile e18w_1400g --upload COMx
```
- **USB Type: Dual Serial** (make_hil sets it). Port 1 is the normal CLI; port 2 is the HIL channel.
- The **flight battery** must be in: it powers the servos and gives vbat.
- The **SD card** must be in: the flight log is written as usual.
- The profile `e18w_1400g` is AeroTech E18W ascent + E18W landing at 1.4 kg. It changes only the build copy's config.h and burn table; your `src/` is untouched. `tools/hil/profiles/e18w_1400g.json` also carries the PC plant's settings (casing eject 46.3 g, 0.75 s band cut, chute CdA 3.387 m²).

### A bench run in the dashboard
`python tools/sim/dashboard.py` → **HIL bench...** opens a separate window.
1. **Ports:** Auto finds the Teensy. Otherwise type the COM ports, or use "PC build (socket)" for the test host below.
2. **Scenario:** mode, seed, wind and gust, motor faults (dud igniter, landing thrust %, ignition delay), stuck chute releases, sensor faults. Fault times are seconds after Launch; a duration of 0 is off.
3. **Connect.** The Teensy enters LIVE and sees a still pad.
4. **Prepare.** This does the pad calibration, sends `mode` and `chute lock`.
5. **Arm.** The Teensy calibrates for ~4 s; keep the bench still.
6. **Launch.** The simulated ascent motor lights.
- **Abort** fails the IMU on purpose, so a flying core aborts to the chute. **Stop** ends the session.
- The plots update live: altitude (truth vs KF), vertical speed, tilt, gimbal command and a state strip. Open **3D view** to watch the rocket fly.
- At touchdown the flight loads into the main dashboard like a normal sim run, the twin flies, and the comparison table fills in.

### The same from the command line
```
python tools/hil/live_run.py --mode chute
python tools/hil/live_run.py --mode land --set dud_landing_igniter=true
python tools/hil/live_run.py --mode land --set imu_frozen_at_s=1.5 imu_frozen_s=2
python tools/hil/live_run.py --mode land --abort-at 1.5     # operator abort
python tools/hil/live_run.py --mode land --live-legs        # asks you to type LEGS
```
`live_run.py` runs pad → Prepare → arm → countdown → Launch → fly → twin compare. Exit code 0 means the twin agrees, 1 means it differs, 2 means a link or sequence error. Each run is saved in `tools/hil/sessions/live_<time>/`: `session.json`, `result.json`, `twin.json`, `compare.txt`.

### Reading the twin table
The twin replays the session's own tick timeline (each tick's time, loop period, and which sensor frame the Teensy used). It runs it through the same firmware compiled for the PC, with the same physics and the same random sensor noise. So a healthy bench run matches **exactly**: every row reads `0.000` and `ok`.
- **Events** (LAUNCH, APOGEE_DET, FIRE_LANDING ...) must match within ±50 ms.
- **Heights** within ±0.5 m; **speeds** within ±0.3 m/s.
- Abort reason, final state, chute releases and the landing verdict must be identical.

A **DIFFERS** row means the Teensy did something the PC build of the same code didn't, given identical inputs. Likely causes:
- a firmware difference between the board and the PC build (an old upload? re-run make_hil);
- floating-point or timing behaviour on the board;
- a missed or garbled packet (check the CRC count and max stale).

The link stats are printed with every run. A healthy link has a max gap of about 4–15 ms, max stale under ~10 ticks, and 0 CRC errors.

### PC test (no Teensy)
```
python tools/hil/test_live.py
```
This builds the e18w profile's HIL sketch for the PC (`test/host_live.cpp`: the real firmware, real time, its two USB ports on TCP sockets). It flies 8 cases, each against a fresh host: chute, land, dud igniter, 2 stuck chute releases, IMU dropout, frozen IMU, operator abort, and live legs. Every case must have the twin agree and never drive the landing gate. Live legs must drive the legs gate; plain land must not. At the end it regenerates the plain HIL build. It takes about 3.5 minutes. To drive the host by hand:
```
python tools/hil/make_hil.py --profile e18w_1400g --no-compile
g++ -std=gnu++17 -O2 -Wall -Wno-unused-function -I tools/hil/test/mock -I tools/hil/build/RocketFC_HIL tools/hil/test/host_live.cpp tools/hil/test/net.cpp -o tools/hil/build/host/host_live -lws2_32
tools/hil/build/host/host_live tools/hil/build/host/sd_live 7700 120
python tools/hil/live_run.py --port socket://127.0.0.1:7701 --console socket://127.0.0.1:7700 --mode chute
```
(`-lws2_32` is Windows only.) `host_live` waits up to 20 s for both clients, and prints `landing gate ON/OFF` and `legs gate ON/OFF` whenever those pins change.

### How it works
- **The Teensy is the clock.** Each 500 Hz tick, right after the flight core and actuators update, the HIL build sends a telemetry packet. The PC steps its physics to that packet's time with the commands it already holds, takes the new commands, and replies with the sensor frame for that moment. The Teensy uses that frame on its next tick. That is one tick (2 ms) more sensor-to-actuator lag than `tools/sim/run.py`; the twin has the same lag.
- **Batching.** If several packets arrive at once, each still steps the physics, but only the newest frame is sent. The telemetry says which frame the Teensy used, so the twin can replay it exactly.
- **IMU fail** (the flight code's `imuOk()` false) on any of: no frame for 100 ms (link loss), 250 identical ticks (frozen data), the frame's fail flag, or the operator abort.
- **Framing:** `A5 5A <type> <len> <payload> <crc8>`. CRC-8 is poly 0x07, init 0, over type+len+payload; little-endian throughout.

| type | direction | what |
|---|---|---|
| `F` | PC → Teensy | sensor frame, 32 B: `uint16 seq; uint8 flags` (bit0 new baro, bit1 IMU fail)`; uint8; float ax ay az gx gy gz baroAlt` (body frame, bias-corrected gyro) |
| `T` | Teensy → PC | telemetry, 80 B: ms, frame seq used, state, abort reason, gimbal, servo µs, chute position, bits, KF alt/vel, tilt, quaternion, PID terms, stale ticks, loop period |
| `C` | PC → Teensy | text: `live <pad ax ay az> <baro var> <legs 0/1>`, `live off`, `status`, `abort`, `reboot` |
| `M` | Teensy → PC | text reply |

  Telemetry bits: 1 landing fire (timed only), 2 legs commanded, 4 legs pin really driven, 8 chute release, 16 chute detected, 32 in flight, 64 TVC active, 128 IMU bad.

| File | What |
|---|---|
| `firmware/sensors_hil.h` | the shim: LIVE mode (frames in, telemetry out, commands) |
| `firmware/hil_hooks.h` | pyro gates and simulated volts (patched into the build copy's `actuators.h`) |
| `profiles/*.json` | motor/mass profiles (`make_hil.py --profile`) |
| `live/link.py` | ports, packing, decoder, console EVENT parsing, `find_ports()` |
| `live/scenario.py` | the `Scenario` (faults, wind, mode, seed) |
| `live/flight.py` | `LiveFlight`: physics, sensors, servos and faults on the Teensy's clock |
| `live/runner.py` | the 500 Hz loop, session save/load, `run_twin()`, the dashboard process |
| `live/twin.py` | twin flight and the comparison |
| `live_run.py` / `test_live.py` | command line / PC test |
| `test/host_live.cpp` | the HIL sketch on the PC in real time, over TCP |
| `../sim/hil_panel.py` | the dashboard's HIL bench window |
