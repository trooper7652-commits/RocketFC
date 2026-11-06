# 02 — Candidate solvers

Notation: h = altitude (m, up), v = vertical velocity (m/s, up; negative when falling),
V = −v = descent speed, θ = tilt from vertical, d = ignition delay (fire command → thrust),
S(V) = "stopping distance": how far the rocket falls from the fire command until it stops,
for the nominal vehicle. The current table is H_IGNITE(V) ≈ S(V).

**Key structural fact.** Altitude never enters the burn physics (constant air density, no ground
effect), so for any *nominal* model the fire decision is "fire when h ≤ S(V)". An onboard
predictor that integrates the same model returns the same S(V) as the table; on-board work can
only add value through inputs a table doesn't see: a lag-corrected state, tilt, a measured drag
coefficient, or uncertainty-aware aiming.

## Error budget (1-D physics, how far the *correct* fire altitude moves; `risk_table.py`)
| Source | at V = 10 m/s | at 15 m/s | at 18 m/s | Observable before ignition? |
|---|---|---|---|---|
| Ignition delay +75 ms (+15 %) | +2.6 m | +3.3 m | (motor can't stop it) | **No** |
| Descent mass +5 % | +2.3 m | (motor can't stop it) | (can't) | **No** (only CdA/m is) |
| Motor thrust +1.7 % / +6.7 % | −0.6 / −2.2 m | −1.1 / −3.9 m | −1.4 / −5.0 m | **No** |
| CdA +20 % | −0.2 m | −0.6 m | −1.0 m | Partly (drag/mass from accel) |
| KF lag (speed reads 0.18 m/s slow) | +0.5 m | +0.6 m | — | **Yes** (correctable) |
| Tilt 15° during the TVC recovery | ≈ +0.3 m (est.) | | | **Yes** (gyro) |

Consequence: the biggest errors are *unobservable before ignition*, so they can only be handled
statistically (aim point), not predicted. Also, 15 m/s at fire (the nominal 48 m apogee) is right
at the F15's energy limit for 0.95 kg: a +5 % heavier rocket can't be stopped from there by any
timing. That is a trajectory/vehicle issue outside the solver.

---
## B — Lag compensation (attacks bottleneck 1: estimator lag)
Physics: the KF tracks a barometer that reports where the rocket was τ seconds ago. Predict
forward by τ (a Smith-predictor idea):

  h_c = kfAlt + kfVel·τ_LAG                       (kfVel < 0 ⇒ lowers the altitude)
  v_c = kfVel + a_ff·τ_LAG,  a_ff = −g + k·v²,  k = ½ρ·CdA / m   (freefall model;
        the error of using the model instead of the accelerometer is ≈ 0.002 m/s)

τ_LAG is a new `[MEASURE]` constant (sim: effective 17–22 ms; hardware: unknown — bay venting,
sensor filtering). Inputs: existing `kfAlt`, `kfVel`. Cost: a few multiplies per tick.
Failure modes: wrong τ (over-correction fires early → hops); a baro bias from airflow is NOT a
lag and is not fixed by this. Must be paired with an explicit aim point (C) because today's lag is
an accidental safety margin.

## C — Risk-aware aim point (attacks bottlenecks 3 and 5: delay/mass/thrust uncertainty and
## the asymmetric hop cliff)
Offline, for each table speed V: draw J vehicles from an assumed uncertainty distribution
(delay, mass, CdA, thrust scale, leftover estimator error), simulate the vertical flight from the
fire command *including* hop → climb → burnout → fall, and pick the fire altitude H*(V) that
minimizes the expected touchdown speed. Because altitude doesn't enter the dynamics, one
integration per draw evaluates every candidate H at once (contact = first time the fall distance
reaches H). Output: a second generated table; zero runtime cost (same interpolation).
This is the "critical fractile" idea: with a lopsided cost you aim at a quantile of the error
distribution, not at its middle. Inputs: none new. Failure modes: the assumed distribution is
wrong (tested in the robustness matrix); flat optimum ⇒ small gains.

## D1 — Tilt thrust-loss compensation (attacks the cos(θ) loss after a tilted ignition)
Thrust acts along the body, so the vertical part is T·cosθ until the TVC rights the vehicle.
Model θ(t) = θ_on·e^{−(t−d)/τ_r} after thrust onset (θ_on = predicted tilt at onset); the extra
stopping distance is ≈ c(V)·θ_on² (small-angle loss ∝ θ²/2), c(V) precomputed offline per table
row. Fire when h_c ≤ H*(V) + c(V)·θ_on². τ_r is an `[ESTIMATE]` (TVC recovery time constant).

## D2 — Predicted-tilt interlock (attacks bottleneck 2: tilt-limited late fires) — TRACK 2
Today: never fire while θ ≥ 15°, so if the window opens while tilted, the rocket waits and fires
far too late (median 22.9 m/s for those flights). In descent the tilt is usually *falling* on its
own (weathervaning), and thrust only starts d ≈ 0.5 s after the command. Rule:
  θ_on = θ + θ̇·d ;  allow fire if θ_on < 15° AND θ < θ_HARD (new constant, e.g. 25°).
θ̇ from a low-pass finite difference of `tiltRad`. **Relaxes a safety interlock you set ("never
light the F15 beyond this tilt") — evaluated in the copy only; porting needs your explicit OK.**

## A — Onboard predictor (fixed-step RK4 over a thrust table)
Equivalent to a table unless it uses inputs a table can't. Its only extra observable here is the
drag-to-mass ratio k = (½ρCdA)/m measured from the accelerometer in freefall (|a_spec| ≈ k·v²);
the error budget says CdA ±20 % moves the fire altitude 0.2–1.0 m, but mass (the larger term)
stays unobservable. Decision: evaluated offline first (does knowing k help in the headroom
data?) before spending C++ effort; it would cost ~1600 thrust-table evaluations per prediction
(~100–200 µs on the Teensy), so it would run decimated (e.g. 50 Hz) if kept.

## E — TVC impulse dumping (outside the solver) — needs your approval before any implementation
The only approach with a flown success on this exact motor (BPS.space) modulates thrust *after*
ignition. With a ±5° gimbal the gimbal angle alone wastes ≤ 0.4 %; a useful dump needs a commanded
vehicle tilt of 15–30° (and a way to cancel the sideways push) — a guidance/control change. Not
implemented; presented as a recommendation with evidence.

## Excluded (by the pre-registered rules)
- "Abort to the chute when a hard landing is predicted" — would improve the powered-landing
  statistics by converting landings into aborts; criterion 4 forbids counting that as a win.

---
## Outcome (filled in after evaluation)
- **B (lag compensation)**: correct physics (fitted effective lag 17 ms; leaves ±0.03 m / ±0.03 m/s)
  but alone it adds hops (58 → 72 on dev), because today's lag is an accidental safety margin.
- **C (risk-aware aim)**: reduces hops (58 → 33–45) but raises the median (+0.1 to +0.4 m/s) and
  makes clean flights land harder (2–3.5 → ~5.5 m/s): the 1-D model aims later than the 6-DOF sim's
  optimum (the sim needs ~0.3 m *earlier* fires than 1-D physics predicts, mostly a wind/tilt
  effect, r = 0.76 with |wind|). Even with a table built for a measured (±1 % mass, ±5 % delay)
  spread it stayed 0.2 m/s worse than the current table.
- **D1 (tilt-loss compensation)**: ≤ 3 cm effect; no measurable change → dropped (ablation rule).
- **D2 (predicted-tilt interlock)**: big tail improvement (dev p90 −8.5 m/s, missed windows 20 → 3;
  clean 60–73 m apogee 8–9 → 3.6–4.1 m/s) but no median gain, more hops when combined with today's
  aim, and the rescued missed-window flights land harder (28 vs 24 m/s). Relaxes a safety rule.
- **A (onboard predictor)**: not built — collapses to the table except for a measured drag
  coefficient, whose effect (CdA ±20 % → 0.2–1.0 m) is small next to the unobservable terms.
- **"Thrust-to-mass from the ascent burn"** (idea found during the work): would remove the sim's
  ±5 % mass uncertainty, but only because the sim's ascent and landing motors are identical; a
  ±10 % ascent-motor thrust error would make it fire metres late — weighing the rocket gets the
  same benefit safely. Not built.
- **E (TVC impulse dumping)**: not built (outside the solver; needs approval). It is the only
  approach with a flown success on this motor.
