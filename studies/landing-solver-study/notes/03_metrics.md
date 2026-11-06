# 03 — Pre-registered metrics and pass criteria

Written 2026-10-12, **before any candidate-vs-baseline comparison was run** (only baseline
characterization, the oracle/headroom scan of the baseline, and 1-D design analysis exist so far).
SHA-256 of this file is recorded in `03_metrics.sha256` and quoted in REPORT.md. These rules are
not changed after results are seen; any later clarification is appended under "Amendments" with a
timestamp and a reason, and never loosens a criterion.

## Definitions (implemented in `scripts/metrics.py`)
- **Flight**: one closed-loop sim run (`tools/sim/run.py`), FULL_LANDING mode.
- **Powered landing**: fire command issued, plant touched down, no abort, no sim error.
- **Touchdown speed**: |vertical velocity| at the first ground contact (`touchdown_speed_mps`).
  After a hop this is the fall-back impact speed.
- **p90**: 90th percentile of touchdown speed, `numpy.percentile(x, 90)` (linear interpolation)
  — "90 % of powered landings are at or below this speed".
- **Soft landing**: < 2.5 m/s (`burn_table.py`'s own "GOOD" threshold); also reported: < 1.0 m/s.
- **Hop**: after landing-thrust onset, true vertical velocity ≥ 0 while true altitude > 0.25 m
  (the rocket stopped in the air). **Severe hop**: hop with touchdown > 5 m/s.
- **Aborts**: counted over all flights, by reason (TILT, MISSED_WINDOW, DUD_IGNITER, KF_UNHEALTHY,
  IMU_FAIL) and by the state they happened in.
- Secondary (reported, not gating): horizontal speed at touchdown, tilt at fire, KF error at fire.

## Protocol
- Monte Carlo = `sweep.py monte-carlo` dispersions exactly (reproduced by `scripts/mc.py`), n = 200
  per seed. Same seed ⇒ identical dispersion draws and sensor-noise seeds for both solvers.
- **Dev seeds 7, 11, 13** (600 flights): all design, tuning, screening and ablation.
- **Holdout seeds 101–110** (2000 flights): run **once** for the Track-1 finalist vs baseline.
- **Holdout seeds 201–210** (2000 flights): run **once** for the Track-2 finalist (if any) vs baseline.
  - Track 1 = best candidate that does **not** relax the 15° tilt interlock.
  - Track 2 = best candidate that **does** (D2). Track 2 can only be ported with the user's explicit OK.
- **Paired statistics**: flights matched by (seed, index); speed statistics over flights that are
  powered landings in both runs; paired bootstrap, 10 000 resamples, `numpy.default_rng(20261012)`,
  95 % percentile intervals for Δ = candidate − baseline of median, p90 and mean.
- **Clean apogee sweep**: no dispersions, ascent thrust scaled 0.80–1.20 in 0.025 steps
  (apogee ≈ 20–82 m, includes the > 58 m tilt regime), noise seeds 42, 43, 44; per point the mean
  touchdown over the seeds that landed under power.

## PASS — a candidate is "measurably better" only if ALL hold
On its holdout set, versus the unchanged baseline:
1. Δmedian ≤ **−0.5 m/s** and the 95 % CI upper bound of Δmedian **< 0**.
2. Δp90 **< 0** and the 95 % CI upper bound of Δp90 **< 0**.
3. Hops (total) **≤** baseline hops.
4. Aborts **≤** baseline for **every** reason (so no outcome is "improved" by turning bad landings
   into chute aborts).
5. Clean apogee sweep: mean over the powered points improved, **no** point worse by > 0.3 m/s,
   no new aborts.
6. **Robustness gate** (below) passed.

## Robustness gate (model mismatch) — run on seeds 101, 102, 103 (600 flights) per condition
For each condition, candidate vs the table **under the same condition** (the table is regenerated
with the same wrong belief where the belief is a `burn_table.py` input):
- Solver believes the wrong value (plant nominal): mass ×0.9, ×1.1; CdA ×0.7, ×1.3;
  ignition delay 425 ms, 575 ms.
- Plant differs from the solver's model: landing thrust ×0.9, ×1.1; ThrustCurve certification
  curves (Coker, Smart) as the true motor; baro lag 0, 10, 50, 75 ms; baro airflow error ±10 % of
  dynamic pressure; accel bias 0.196 m/s² (20 mg) on every axis.
"Never worse than the table" = in every condition: Δmedian ≤ +0.10 m/s, Δp90 ≤ +0.25 m/s (point
estimates), hops ≤ table hops + 2, and aborts per reason ≤ table + 2 (small tolerances for sampling
noise on ~230 powered landings, fixed now).

## Ablation rule
A component stays in the final solver only if it improves the dev-seed median or p90 on its own,
**or** removing it from the final combination makes the dev-seed median or p90 worse
(leave-one-out). Rationale fixed in advance: lag compensation (B) removes a bias that currently
acts as an accidental margin, so it is expected to help only together with an explicit aim point (C).

## Things that would make me reject even a "passing" result
- Any use of plant truth inside a solver (code review of the diff before porting).
- A win that disappears when the plant's thrust curve differs from the solver's (shared-model check).
- A win produced only by converting landings into aborts (criterion 4).
