# TVC Rocket PID Auto-Tuner

A single-file Python simulation that models the rotational dynamics of a thrust-vectoring
(gimbaled-motor) model rocket and **automatically finds attitude PID gains (Kp, Ki, Kd)**
for the pitch/yaw control loop. It accounts for the things that actually destabilize a TVC
rocket — finite servo speed/lag, sensor noise, a thrust curve whose gain changes during the
burn, and a disturbance torque — and picks gains that stay stable across the **whole burn**,
not just at one thrust level.

It runs headless and saves PNG plots. Everything is in `tvc_pid_tuner.py`.

## Install

```bash
pip install -r requirements.txt   # numpy, scipy, matplotlib
```

## Run

```bash
python tvc_pid_tuner.py                 # default: differential-evolution tuner + built-in F15
python tvc_pid_tuner.py --method grid   # coarse grid search instead
python tvc_pid_tuner.py --eng motor.eng # use a real RASP .eng thrust file
python tvc_pid_tuner.py --out results   # write PNGs into ./results
python tvc_pid_tuner.py --seed 7        # change the optimizer seed
```

It prints the recommended gains and metrics, then writes `tilt.png`, `gimbal.png`,
`rate.png`, and `robustness.png`. The last printed line is ready to paste into Arduino:

```
// PID gains from tvc_pid_tuner.py  ->  float Kp = ..., Ki = ..., Kd = ...;
```

## Plugging in YOUR rocket's numbers

Open `tvc_pid_tuner.py` and edit the `Config` dataclass near the top (section 1). Every
physical value is a **placeholder** tagged `MEASURE/REPLACE`. The ones that matter most:

| Field | Meaning | How to get it |
|---|---|---|
| `I_kgm2` | Transverse moment of inertia about the CG (kg·m²). **Most important** — the whole response scales with it. | Estimate as a uniform rod `I ≈ (1/12)·m·L_body²`, then add the motor + nosecone as point masses `+ m·d²` at their distance `d` from the CG. Best: measure with a **bifilar (two-string) pendulum**: `I = (m·g·d²·T²)/(16·π²·Lₛ)` where `d` = half the string spacing, `Lₛ` = string length, `T` = period of small twisting oscillations. |
| `L_arm_m` | Distance from the CG to the gimbal pivot (m). | Balance the rocket to find the CG; measure to the pivot; subtract. |
| `mass_kg` | Liftoff mass with motor (kg). | Scale. |
| `delta_max_deg` | Mechanical gimbal limit (± deg). | Measure the linkage's max travel. |
| `servo_slew_dps` | Max servo slew rate (deg/s). | A "0.10 s / 60°" servo ≈ 600 °/s; derate for the gimbal linkage. Slow servos cause oscillation. |
| `servo_lag_tau_s` | First-order servo lag (s). | Bench-measure the step response; realistic digital servos are ~5–15 ms (default 8 ms). **This plant is sensitive to it** — much above ~15 ms the high loop gain becomes very hard to stabilize and the tuner returns sluggish gains. |
| `loop_rate_hz` | Controller update rate. | Match your firmware. |
| `gyro_noise_dps` | Gyro rate-noise std-dev (deg/s). | From your IMU's datasheet / a still-bench capture. |
| `disturbance_torque_Nm` | Constant bias torque (thrust misalignment / CG offset). | Set a realistic worst case; it's what proves `Ki` removes steady-state error. |
| `initial_tilt_deg` | Starting tilt the controller must recover from. | A stress test (10–15°). |

To use your motor's real thrust curve, download its `.eng` file (e.g. from thrustcurve.org)
and pass `--eng path/to/motor.eng`. Otherwise the built-in Estes F15 curve is used (a
**placeholder** — replace it).

After editing, just rerun `python tvc_pid_tuner.py`.

## How the tuning works

- **Plant** (section 3): `τ = F(t)·L·sin(δ) + disturbance`, `α = τ/I`, integrated with a
  small fixed timestep. `F(t)` changes over the burn, so the loop gain changes too.
- **Servo** (section 4): commanded gimbal → rate limit → first-order lag → mechanical clamp.
- **PID** (section 5): `error = setpoint − angle`; integral with an anti-windup clamp;
  derivative from the **measured rate** (`−Kd·rate`, not d(error)/dt); output clamped to
  ±δ_max. This matches typical flight firmware.
- **Cost** (section 7): one run is scored by ITAE + overshoot + steady-state error +
  residual-oscillation + gimbal-thrashing + control-effort/saturation penalties, plus a big
  penalty for divergence. The reported cost is the **worst case across peak / mid / tail
  thrust**, so the gains hold up as the motor burns down. Tuning runs on the deterministic
  (noise-free) plant for a clean cost landscape (`tune_with_noise = False`); gyro noise adds
  an irreducible floor that hides the optimum and barely shifts it. The **final reported run
  and all plots still include the configured noise**, so the metrics you see are realistic.
- **Optimizer** (section 8): `scipy.optimize.differential_evolution` by default (robust for
  this non-smooth, multi-modal problem), or a coarse grid search via `--method grid`.

Tune the search ranges and cost weights in the `Config` block if you want a more aggressive
or more conservative response.

## Caveat

This models **rotational (attitude) dynamics only** — it does not simulate translation,
altitude, aerodynamics, or the descent/landing trajectory. It is for finding attitude PID
gains. Always validate gains on a restrained test stand before flight.
