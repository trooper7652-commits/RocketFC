# Task: research, build, and prove a revised in-flight landing-burn solver for RocketFC

You are running at maximum effort. Use it. Be exhaustive in research, rigorous in testing, skeptical of your own results, and don't stop at the first plausible answer. This is a multi-hour task. Work through every phase below in order. Write intermediate results to files so nothing lives only in your context.

## 1. Background you need first

**Project:** `C:\Users\troop\OneDrive\Documents\Inventor\SpaceX Rocket\RocketFC` (the path has a space in "SpaceX Rocket", so quote it). It's a Git repo on branch `main`. It is the flight computer for a hobby TVC (thrust vector control, meaning a gimballed motor) self-landing model rocket. Hardware: Teensy 4.1 (600 MHz Cortex-M7 with a single-precision FPU), BMI088 IMU, and MS5611 barometer. Read `README.md` and `src/config.h` first.

**The working tree has uncommitted changes the user cares about** (README, RocketFC.ino, config.h, actuators.h, and several tools/sim files). Never stash, reset, checkout, or otherwise discard them. Never commit unless the user asks.

**What "burn solver" means here:** the landing motor is an Estes F15, a solid motor. It can't throttle, shut down, or restart. The only thing the flight computer controls is **when it sends the fire command**. After that, the e-match ignition delay (nominally `IGNITION_DELAY_MS` = 500 ms, a `[MEASURE]` value with about ±15% uncertainty) passes before thrust appears. The goal is to time that command so the rocket reaches ~0 m/s right at the ground, without stopping in mid-air.

**The current solver:**
- `tools/burn_table.py` runs offline. It integrates the F15 thrust curve (`tools/motors/`) with drag (CdA) and mass to produce `src/core/burn_table.h`, which is GENERATED: a 1-D table `H_IGNITE[vDown]` giving the altitude at which to fire for a given descent speed.
- In `src/core/flight_state.h`, `updateDescent()` fires when `kfAlt <= hIgnite(-kfVel) + BURN_TABLE_MARGIN_M`, subject to interlocks: tilt < `TILT_IGNITE_MAX_DEG` (15°), Kalman filter healthy, and landing-pyro continuity. Below `MISSED_WINDOW_ALT_M` it aborts to the parachute.
- On hardware, the only inputs available to the solver are the `FsmInput` fields: `kfAlt`, `kfVel`, `kfHealthy`, `tiltRad`, `accelLongG`, `accelNormG`, `imuHealthy`, `contLanding`, `ms`, and `dtMs`. You may add inputs that the firmware genuinely computes (for example the full attitude quaternion from `src/core/ahrs.h` or KF covariance from `src/core/altitude_kf.h`). You may **never** use simulator plant truth.

**What has already been tried. Read this carefully so you don't repeat it.** On 2026-10-12 an "exact" burn-table solver was built: one RK4 integration per table row, which is more accurate than the current table. It made **no measurable closed-loop difference** and was not adopted. The closed-loop sim showed that the real limits on touchdown speed are:
1. **Estimator lag.** The altitude Kalman filter runs about 0.9 m behind truth during fast descent, so the motor fires late. That alone sets a ~3.5 m/s touchdown floor even with zero dispersions.
2. **Tilt interlock.** Above ~58 m apogee, the 15° tilt interlock delays firing, giving 7.6–9 m/s touchdowns.
3. **Ignition-delay dispersion** of ±15%.
4. `BURN_TABLE_MARGIN_M = 0.5` improved clean flights (3.5 → 2.5 m/s) but gave mixed Monte Carlo results: soft landings went from 2 to 7 of 70, but the median got worse (6.63 → 6.90).
5. **The hop cliff.** Aiming exactly at 0 m/s at the ground puts the solver on a cliff edge: a little early means the rocket stops in the air and falls back at ~20 m/s. The cost function is badly asymmetric.

So **table accuracy is not the bottleneck.** A revised solver has to address lag, tilt, delay uncertainty, and the asymmetric hop risk. Any candidate that doesn't explain which of these it attacks should be dropped.

**Tools available:**
- Closed-loop sim in `tools/sim/`. The real C++ core is compiled to `rocketfc_core.dll` (see `tools/sim/Makefile` and `bridge.cpp`) and driven over ctypes. `run.py` runs one flight. `sweep.py grid --param ... --range LO HI --n N` and `sweep.py monte-carlo --n N --seed S --out file.json` cover sweeps and Monte Carlo. Read `DISPERSION_FIELDS` in `sweep.py` and the summary fields (`touchdown_speed_mps`, etc.) before designing experiments. The sim patches a throwaway copy of `src/` under `tools/sim/build/` for constant sweeps.
- PC unit tests: `tools/replay`, run with `make test` using `C:\msys64\ucrt64\bin\g++.exe`.
- Firmware build: `$env:LOCALAPPDATA\Programs\arduino-cli\arduino-cli.exe compile --fqbn teensy:avr:teensy41`.
- Python 3.12 with numpy, scipy, and matplotlib. There is no PyQt.

## 2. Hard rules

- **Prove before changing.** Build and test everything in an isolated copy of the working tree, including its uncommitted changes, placed in your scratchpad directory. Compare against the unchanged current version in the closed-loop sim with fixed seeds. Port to the real project **only if the new solver is measurably better** on the metrics defined in Phase 3. If it isn't, leave the project untouched and report the numbers. That outcome is acceptable and expected to be possible.
- Before porting, record a hash of every file you will touch when you make the copy, and verify the hashes still match before writing. If anything changed, stop and tell the user.
- Don't change hardware decisions. There is no arm switch, and `REQUIRE_ARM_SWITCH` stays false by the user's choice. The chute is a servo latch on pin 23. Pyro pins and the legs-nichrome timing stay as they are.
- Don't change PID gains, the AHRS, or the Kalman filter itself in this task. If research shows the estimator must change, write that up as a separate recommendation and don't implement it. **Exception:** a lag-compensating predictor that lives inside the solver and reads the KF's outputs is in scope.
- Keep `[MEASURE]` placeholders as placeholders. Every new tunable constant goes in `src/config.h` with a comment, and if it must be measured it gets a `[MEASURE]` tag.
- The solver must be deterministic and allocation-free, and it must be safe for worst-case flight-loop timing on the Teensy (see Phase 4).
- The user is a hobbyist learning control theory. In your reports, define terms (for example predictor-corrector, ZEM/ZEV, convex guidance, Monte Carlo, or p90) in plain language the first time you use them, and favor physical intuition over notation.
- If you need a fact only the user has, ask, and include the exact command or steps for getting it in the question. Offer an "I'll handle both outcomes" option where you can.

## 3. Phases

### Phase 0: Understand the current system (no edits)
Read `src/core/flight_state.h`, `burn_table.h`, `altitude_kf.h`, `ahrs.h`, `flight_core.h`, `src/config.h`, `tools/burn_table.py`, and all of `tools/sim/` (especially `plant.py`, `sensors.py`, `vehicle.py`, `run.py`, `sweep.py`, and `bridge.cpp`). Write `notes/00_current_system.md` in your scratchpad covering:
- the exact firing logic
- loop rate
- the sources of estimator lag, quantified from a sim run as KF altitude/velocity minus truth versus time during DESCENT
- how ignition delay and its dispersion are modelled
- the thrust-curve source
- what the sim does and doesn't model (thrust-curve variation, mass error, wind, baro noise, tilt during the burn)

Then **reproduce the baseline numbers above** (3.5 m/s clean floor, the 58 m tilt effect, and Monte Carlo medians). If you can't reproduce them, find out why before going further.

### Phase 1: Research successful solvers (web)
Search widely and in parallel, and fetch primary sources: papers, source code, flight videos with technical write-ups, and forum threads from the builders themselves. Leads to start from (verify each one; don't trust this list):
- **Hobby propulsive landing with solid motors.** Look at BPS.space (Joe Barnard, Scout / Echo / Lander vehicles) and how his landing-burn ignition timing evolved across attempts, including any successful landings and what changed to make them succeed. Look for other hobbyists or university teams who attempted solid-motor propulsive landings, and for any use of TVC to dump excess impulse (deliberately gimballing to waste thrust) as a way of "throttling" a solid motor.
- **Suicide-burn / hoverslam timing.** Look at open-source Kerbal Space Program kOS and MechJeb landing scripts (real, tested code with readable logic), and at descriptions of predictor-corrector landing guidance.
- **Professional powered-descent guidance.** Apollo lunar module descent guidance (polynomial / E-guidance), Mars powered descent, Falcon 9 landing, convex-optimization guidance (G-FOLD, Açıkmeşe/Blackmore), and ZEM/ZEV guidance. Mostly mine these for ideas that transfer to a no-throttle motor, such as reasoning about uncertainty and margins and predicting through delays.
- **Delay and lag compensation.** Smith predictors, forward-propagating state through known latency, and accelerometer-aided altitude prediction.
- **Decision-making under uncertainty with asymmetric cost.** Choosing an ignition time that minimizes *expected* or *worst-case* touchdown speed over the ignition-delay and thrust-scale distribution, instead of hitting the nominal exactly. This addresses the hop cliff.
- **Solid-motor variability data.** Published F15 or Estes thrust-curve spread (ThrustCurve.org, NAR/TRA certification data) and e-match ignition-delay spread, so the dispersions in the sim are realistic.

For each source, record: URL; whether it was **actually flown successfully, only simulated, or only proposed**; the vehicle and motor type (throttleable or not); the algorithm in a few sentences; what it needs as inputs; and how it maps, or doesn't map, to this rocket's constraints (solid motor, ~0.5 s uncertain ignition delay, laggy baro/IMU Kalman filter, single-precision Teensy, and ±15° tilt). Put all of this in `notes/01_research.md` with a summary table. Never fabricate a source or a claimed success. If you can't verify something, say so.

### Phase 2: Design candidates
From the research, design **2–4 candidate solvers**. For each one, state which bottleneck(s) from section 1 it attacks and why it should help. Examples of the kind of candidate expected (you're not limited to these):
- **(A) Onboard predictor.** Each cycle, propagate the current state forward through the expected ignition delay and the F15 thrust curve (fixed-step RK4, float, a precomputed thrust table) and fire when the predicted touchdown speed crosses a target.
- **(B) Lag compensation.** Advance the KF state by its estimated lag using the accelerometer, which is very good during freefall, before any solver decision.
- **(C) Risk-aware aim point.** Choose the fire time that minimizes expected touchdown speed over the delay and thrust distribution, explicitly penalizing the hop.
- **(D) Tilt-aware timing.** Include cos(tilt) thrust loss and predict when tilt will clear the interlock, instead of simply waiting, possibly as an earlier fire with known vertical-thrust loss.

Write the math, the inputs, the per-cycle cost estimate, and the failure modes for each in `notes/02_candidates.md`. Flag clearly any candidate that would need changes outside the solver (for example, impulse dumping through the TVC). Those need the user's approval before implementation, so ask once you have evidence for them.

### Phase 3: Define "better" before testing
Write the metrics and pass criteria in `notes/03_metrics.md` **before** you run any comparison. At minimum:
- touchdown speed median, mean, and p90 (the speed that 90% of flights land at or below)
- soft-landing count, using the threshold the sim/sweep already uses (or justify one)
- hop count, meaning flights that reach zero velocity above ground and fall back
- abort count by reason (MISSED_WINDOW, DUD, etc.)
- clean-flight touchdown speed across an apogee sweep that includes the >58 m regime

"Better" means a clear improvement in median and p90 touchdown speed with no increase in hops or aborts. Decide on paired comparisons with the same seeds and dispersion draws for both solvers, n ≥ 200 per seed, at least 3 seeds, and report bootstrap confidence intervals. Don't move these goalposts after seeing results.

### Phase 4: Implement in the isolated copy
- Put the solver in a header-only C++ file in `src/core/` (for example `landing_solver.h`), callable from `updateDescent()`, with the current table kept available behind a config switch so A/B comparison is trivial.
- Add PC unit tests in `tools/replay`: analytic freefall cases, monotonicity, edge cases (very slow or very fast descent, NaN/unhealthy KF, tilt at the limit), and agreement with `burn_table.py` in the zero-lag, zero-dispersion limit.
- Rebuild the sim DLL from the copy and confirm that the copy's sim uses the new code.
- Confirm a clean `arduino-cli` build for `teensy:avr:teensy41`. Estimate or measure the worst-case cycle cost (count operations, or time it on the PC and scale conservatively) and show it fits the flight-loop budget with large margin. If it doesn't fit, reduce the work: run fewer cycles, use a coarser step, or switch to a lookup.

### Phase 5: Evaluate honestly
- Run every candidate **separately**, plus useful combinations, against the unchanged baseline under the Phase 3 protocol. Keep only changes that individually help (ablation).
- **Robustness to model mismatch.** The solver's internal model will be wrong on the real rocket. Run the solver with deliberately wrong assumptions (mass ±10%, CdA ±30%, thrust scale ±10%, ignition delay biased ±15%) while the sim plant uses the truth, and make sure it degrades gracefully and is never worse than the table. A solver that only wins when its model matches the sim exactly is overfitted and must be rejected.
- Check for the sim and the solver sharing the same model. If they use the identical thrust curve and delay model, say so and test with a perturbed plant curve.
- Generate plots (touchdown-speed CDFs, baseline vs candidate scatter, apogee sweep) into `notes/plots/`.

### Phase 6: Decide and report
- **If a candidate passes:** verify the file hashes are unchanged, then port **only** the winning solver, its config constants, the tests, and any `burn_table.py` or README updates into the real project. Rerun `make test`, the arduino-cli build, and one closed-loop Monte Carlo in the real tree to confirm the numbers reproduce. Don't commit; show the diff and let the user decide.
- **If nothing passes:** change nothing in the project.
- In both cases, write `notes/REPORT.md` and summarize it to the user. Cover:
  - what the research found, with links and the flown/simulated/proposed status of each source
  - what was tried
  - the results table with confidence intervals
  - what won or why nothing did
  - remaining risks
  - what the user must measure on real hardware (for example ignition-delay spread from the thrust stand in `ThrustStand/`) before trusting it

Write the report in plain language for a hobbyist.
