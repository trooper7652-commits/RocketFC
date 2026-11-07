# Hand-off: finish the closed-loop HIL bench sim

This was written for the cloud agent that finishes this work. Delete this file in your last commit, and move anything worth keeping into `tools/hil/README.md`.

## Goal
Bench-test the real firmware on the real Teensy 4.1 with no rocket and no motors.

- The PC simulates the rocket's physics in real time and feeds the Teensy fake IMU and barometer data over USB.
- The Teensy's own commands go back and steer the simulated rocket (closed loop). Those are the gimbal, landing-motor fire, legs and chute commands.
- The flight shows live in the existing sim dashboard (`tools/sim/dashboard.py`): 3D view plus plots.
- After each run, a pure-PC twin flies the same scenario, and the two are compared.

**The owner's choices:**
- On the bench: gimbal servos, chute servo, and the real IMU + baro plugged in.
- Nothing on the landing e-match. The leg nichrome IS wired, and must stay off unless they opt in for a run.
- Scenarios: wind and gusts, motor faults, chute faults, sensor faults.
- Vehicle: AeroTech E18W/E18W at 1.4 kg (profile `e18w_1400g`).
- Controls live in the dashboard.

## Hard rules
- **Don't change `src/` or flight behaviour.** All HIL changes live in `tools/hil` (the generated build copy is patched, never `src/`).
- **LIVE mode never drives the landing e-match gate pin.** The leg nichrome gate is driven only with live legs, which the user opts into by typing `LEGS`.
- **Don't propose:**
  - an arm switch;
  - thrust-modulation hardware;
  - alternative landing concepts (chute + retro burst, crushable legs).
- **Apogee limit:** 100 ft (30.48 m).
- **Keep the owner's Windows toolchain working.** That is MSYS2 ucrt64 g++ at `C:\msys64\ucrt64\bin` and `mingw32-make`. Linux support is a fallback only.
- **Report style:** the owner is a hobbyist learning control theory. Keep reports short, and explain estimation and control terms plainly.

## Design: how it works (all built)
- **The Teensy is the clock.**
  - Each 500 Hz fast tick, the HIL build sends a telemetry packet right after the core step and actuator update (`baroTick` in the same loop pass).
  - The PC steps its plant to that packet's `ms` with the commands it already holds, takes the new commands, and replies with the sensor frame for that moment.
  - The next tick uses that frame. That is one tick (2 ms) more sensor-to-actuator lag than `tools/sim/run.py`.
- **Framing:** `A5 5A <type> <len> <payload> <crc8>`. CRC-8 is poly 0x07, init 0, computed over type, len and payload. Everything is little-endian.
  - **'F'** (PC → Teensy), `HilLiveFrame`, 32 B: `uint16 seq; uint8 flags` (bit0 new baro, bit1 IMU fail)`; uint8 _; float ax ay az gx gy gz baroAlt`. Body frame, bias-corrected gyro.
  - **'T'** (Teensy → PC), `HilTelemetry`, 80 B:
    - `ms`, the `seq` of the frame the core used, `state`, `abortReason`;
    - `gimbalX/Y`, `usA/B`, `usChute`, `chutePos`, `bits`;
    - `kfAlt`, `kfVel`, `tilt`, `quat w x y z`, PID `pX iX dX pY iY dY`;
    - `staleTicks`, `periodUs` (the core's dt).
    - `bits`: 1 landing fire (timed only, pin never driven), 2 legs commanded, 4 legs pin really driven, 8 chute release, 16 chute detected, 32 in flight, 64 TVC active, 128 IMU bad.
  - **'C'** (PC → Teensy) text commands:
    - `live <pad ax ay az> <baro var> <legs 0|1>`. Entering fresh resets seq and holds a still pad. Re-sending it while LIVE (done at Prepare) only updates the pad level, baro variance and legs.
    - `live off`, `status`, `abort` (the IMU "fails", so a core in flight aborts to the chute), `reboot` (software reset; not while armed or in flight; the chute boots OPEN because it isn't a power-on reset).
  - **'M'** (Teensy → PC): text reply.
  - **Entering LIVE from text mode:** send the text line `live ...\n`; the reply is the text line `OK live legs=sim|LIVE`.
- **The IMU counts as failed** (`imuOk()` false) on any of: no frame for 100 ms (link loss), 250 identical ticks (frozen-data check), frame flag bit1, or operator abort.
- **Pyro hooks** (`firmware/hil_hooks.h`, patched into the build copy of `actuators.h` by `make_hil.py`):
  - In LIVE the landing fire is timed but its pin stays LOW.
  - The legs pin is driven only with live legs.
  - Continuity and pyro volts read 7.8 V while pyros are simulated, so `arm` works with nothing connected.
- **Event lines:** flight events print on the console as `[ms] EVENT NAME value`. Firmware names map to `core.FlightEvent` through `flight.event_code()` (for example `APOGEE` → `APOGEE_DET`).
- **Batching:**
  - If several T packets arrive at once, each still steps the plant and draws its noise; only the newest frame is sent, carrying the baro flag of any it replaced.
  - The T `seq` says which frame the Teensy used, so the twin can replay it exactly.
- **Twin** (`live/twin.py`):
  - replays the session's own tick timeline (`ms`, `periodUs`, seq used) through the PC-built flight core (`core.build_from_src(<build>/src)`, same config.h and burn table);
  - uses the same flight model and the same seeded sensor streams, and emulates the shim's held-frame, link-loss and frozen logic;
  - so a healthy run matches EXACTLY. Tolerances: events ±50 ms, heights ±0.5 m, speeds ±0.3 m/s.

## Files
| file | what |
|---|---|
| `tools/hil/firmware/sensors_hil.h` | the HIL shim; LIVE mode added (frames in, telemetry out, commands) |
| `tools/hil/firmware/hil_hooks.h` | the pyro gates and simulated volts; `g_hilPeriodUs`; `hilReboot()` (the PC host defines `hostReboot()`) |
| `tools/hil/make_hil.py` | generates `tools/hil/build/RocketFC_HIL`. Patches actuators and adds the `.ino` period patch. `--profile NAME` patches the build's config.h, regenerates the burn table, and writes `hil_profile.json`. `--no-compile` skips the Teensy compile (needs arduino-cli). |
| `tools/hil/profiles/e18w_1400g.json` | the E18W/E18W 1.4 kg profile: config overrides, burn table, plant settings (casing eject 46.3 g, legs cut 0.75 s, chute CdA 3.387 m²) |
| `tools/hil/test/host_live.cpp` + `net.cpp/.h` | the generated sketch on the PC in real time. Serial → TCP `127.0.0.1:<port>`, SerialUSB1 → `<port+1>`. Pin changes print `[t host] landing gate ON/OFF` and `legs gate ON/OFF`; the console is copied to `<sd dir>/host_console.txt`. |
| `tools/hil/test/mock/Arduino.h` | `HostSerial.rawSink`, `availableForWrite`, `flush` |
| `tools/hil/live/link.py` | ports (COM or `socket://`), packing, the in-order `Decoder`, `Console` (EVENT parsing), `find_ports()` |
| `tools/hil/live/scenario.py` | `Scenario` dataclass. Times are seconds after Launch; a fault with duration 0 is off. |
| `tools/hil/live/flight.py` | `LiveFlight`: plant, sensors, servos and faults on the flight computer's clock (`advance` → `command` → `frame` → `row`). Also `summarize()`, `rebase()`, `vehicle_for()`, `flight_dll()`. |
| `tools/hil/live/runner.py` | `Runner` (`start`/`poll`/`prepare`/`arm`/`launch`/`abort`/`reboot`/`finish`), session save/load, `run_twin()`, and `runner_process()` for the dashboard |
| `tools/hil/live/twin.py` | `fly()`, `compare()`, `format_table()` |
| `tools/hil/live_run.py` | headless CLI. Auto sequence: pad → Prepare → arm → countdown → launch → twin compare. `--set field=value` for scenario fields. Exit code 0 = twin agrees. |
| `tools/sim/core.py` | new `build_from_src()` |
| `tools/sim/mc_batch.py` | new `--dll` |
| `tools/sim/plant.py`, `vehicle.py`, `tools/burn_table.py` | casing-eject model ported (`ascent_casing_eject_kg`, default 0 = unchanged) |

## How to run it (Linux)
```
python tools/hil/make_hil.py --profile e18w_1400g --no-compile
g++ -std=gnu++17 -O2 -Wall -Wno-unused-function -I tools/hil/test/mock \
    -I tools/hil/build/RocketFC_HIL tools/hil/test/host_live.cpp tools/hil/test/net.cpp \
    -o tools/hil/build/host/host_live            # add -lws2_32 on Windows only
mkdir -p tools/hil/build/host/sd_live
tools/hil/build/host/host_live tools/hil/build/host/sd_live 7700 120 > host_out.txt &
python tools/hil/live_run.py --port socket://127.0.0.1:7701 \
    --console socket://127.0.0.1:7700 --mode chute
```
`host_live` waits up to 20 s for both clients before it boots, and exits when the HIL client disconnects.

## Status
- **Verified on the owner's PC (2026-11-07):** a full chute-mode flight against `host_live`.
  - Link: 8852 packets, max gap 3.8 ms, max stale 2 ticks, 0 CRC errors.
  - Twin: identical to the bench run (every event to 0.000 s; apogee 18.203 m on both).
- **Changed after that run, not yet re-run:**
  - the event-name mapping above (in that run `APOGEE`/`TOUCHDOWN` were flagged only because of names);
  - the profile's chute CdA 3.387 m² (that run used the 0.45 m² default and touched down at 6.97 m/s; expect about 2.5 m/s now).
- **Also verified earlier:**
  - Teensy compile of the plain HIL build and the profile build (before the latest firmware edits: telemetry after the step, 80 B packet, `live` re-entry, `reboot`, period patch);
  - `test_hil.py` 3/3 and the replay tests;
  - casing-eject port with the knob at 0: 50/50 identical;
  - `mc_batch --dll` with the profile DLL reproduced the study's 487/600, with 600/600 identical outcomes.

## Remaining tasks (in order)
1. **Re-run chute and land against `host_live`.** Confirm:
   - the event names match;
   - chute touchdown is about 2.5 m/s;
   - land mode works and the twin agrees;
   - the host output never shows `landing gate ON`.

   Fix whatever turns up.
2. **`tools/hil/test_live.py`**, in the style of `test_hil.py`.
   - In `test_hil.py`, GXX is hard-coded to `C:\msys64\ucrt64\bin\g++.exe`: use it when present, else `shutil.which("g++")`. Do the same in the new test.
   - Build `host_live`. Regenerate the build with `--profile e18w_1400g --no-compile`.
   - Start a fresh `host_live` per case, on its own port.
   - Cases:
     - nominal chute;
     - nominal land;
     - dud igniter: a DUD_IGNITER abort, then the chute;
     - `chute_stuck_releases=2`: 3 releases, then the canopy;
     - IMU dropout in flight: an IMU-fail abort to the chute;
     - frozen IMU;
     - operator abort (`--abort-at`);
     - live legs off: `legs gate ON` never printed;
     - live legs on, in land mode: `legs gate ON` printed.
   - Every case: no `landing gate ON`, and the twin compare passes.
   - Print the link statistics.
   - Restore the plain build at the end, the way `test_hil.py` regenerates it.
3. **`flight3d.FlightView.set_track(track, follow_end=True)`.** It refreshes `_wide_center`, `_dist_wide` and the camera `max_dist` from `track.bounds()`. The dashboard already swaps `view.track` directly.
4. **`tools/sim/hil_panel.py` + a "HIL bench..." button in `dashboard.py`'s top bar** that opens a Toplevel window. Use a window, not a tab: the main figure's full re-plot takes ~140 ms.
   - **Controls:**
     - ports: auto (`link.find_ports`), COM, or socket;
     - the `Scenario` fields (mode, seed, wind/gust, dud, landing thrust %, ignition delay, stuck releases, each sensor fault with at/duration);
     - a "Live legs" checkbox that pops a dialog: type `LEGS`;
     - buttons Connect / Prepare / Arm / Launch / Abort / Reboot / Stop.
   - **Status line:** link Hz, phase, Teensy state, max gap and max stale. Add a log box.
   - **Process:** run `runner.runner_process(cmd_q, out_q, hil, console, scen_dict)` in a `multiprocessing.Process`, which must work with Windows spawn. Poll `out_q` with `after()`.
   - **Live plots:** altitude (truth vs KF), vertical speed, tilt, gimbal X/Y, and a state strip. Update with `line.set_data` at ~10 Hz.
   - **Live 3D (if the dashboard's 3D window is open):** about 4 times a second, rebuild `flight3d.Track` from the streamed rows and put the playhead at the newest sample.
   - **On `("finished", session_dir)`:**
     - load `runner.read_result(session_dir/result.json)` into the dashboard as a normal sim result (`dashboard.result/rows/events`, `is_log=False`, `_after_new_data()`);
     - run `runner.run_twin(session_dir)` off the Tk thread;
     - show the comparison table, ok in green and differs in red.
5. **Add a "Live closed-loop bench sim" section to `tools/hil/README.md`.**
   - Setup:
     - `python tools/hil/make_hil.py --profile e18w_1400g --upload COMx`;
     - USB Type Dual Serial;
     - the flight battery for the servos and vbat;
     - the SD card in.
   - What's simulated and what's real.
   - Safety: the landing gate is never driven; legs are opt-in; `reboot` boots the chute OPEN.
   - The bench sequence, in the dashboard and with `live_run.py`.
   - How to read the twin table.
   - The PC-build test.
6. **Regression checks:**
   - `python tools/hil/test_hil.py`: 3/3;
   - `tools/replay` `make test`;
   - `python tools/sim/test_bridge.py`;
   - 50-flight seed-7 `mc_batch` identical to a baseline from a `git worktree` of `origin/hil-and-landing-fixes`, comparing summaries.

   On Linux, `core.py`'s shared-library compile may need `-fPIC`. Add it for non-Windows only.

Teensy compile: only if `arduino-cli` and the Teensy core can be installed. Otherwise list it for the owner: `python tools/hil/make_hil.py` and `python tools/hil/make_hil.py --profile e18w_1400g`.
