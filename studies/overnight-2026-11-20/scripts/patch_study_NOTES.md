# patch_study.py notes

`python patch_study.py /tmp/ws/<name>` (after `mkws.sh <name>`) adds study switches, all OFF by default,
as constexprs in the copy's `src/config.h`. The all-off build is flight-for-flight identical to the
unmodified tree (checked on 300 flights, no wind and `--wind 6`, whole summary dicts equal).
Override with `--fw-overrides` literals: ints as `"1"`, floats as `"1.0f"` (core.py pastes the string verbatim).

| Switch | Type | Off |
|---|---|---|
| `STUDY_WP_D_M` | float | 0.0f |
| `STUDY_DUD_GUARD` | int | 0 |
| `STUDY_D2` (+ `_HARD_MAX_DEG` 25, `_RATE_LPF_HZ` 5, `_WARMUP_TICKS` 25) | int | 0 |
| `STUDY_WMA` (+ `_GAIN` 1, `_MAX_TILT_DEG` 5, `_START_S` 0.3, `_END_S` 0.6, `_CNA_AREF_M2` 0.00884, `_SCALE` 0.8, `_MIN_SPEED` 4, `_PRIOR_MPS` 1) | int | 0 |

Plant knob: `VehicleConfig.gimbal_offset_deg` (vehicle.py); run.py adds it (radians) to `act_x` after the servo model.

## T5 window-passed abort (flight_state.h, updateDescent)
Chained after the fire branch: `else if (STUDY_WP_D_M > 0 && !interlocks && kfAlt < hIgn - STUDY_WP_D_M) doAbort(MISSED_WINDOW)`.
`hIgn` includes BURN_TABLE_MARGIN_M. The early return for vDown < BURN_MIN_SPEED_MS stays in front, so the rule
only runs when the table is evaluated. `interlocks` is the D2-modified one when STUDY_D2 is on.

## T6 dud guard (flight_state.h, updateDescentChute)
Only when `abortReason_ == DUD_IGNITER` and state is DESCENT_CHUTE. The ABORT -> DESCENT_CHUTE path and the chute
release are untouched. Thrust seen = `accelNormG > IGNITE_CONFIRM_G && kfAlt > TOUCHDOWN_ALT_M` (latched). While
latched: `out.tvcActive = out.useLandingGains = true` until `accelNormG < BURNOUT_ACCEL_G` for BURNOUT_MS (new
`dbDud_` Debounce; then latched off for good) or touchdown (the existing touchdown check leaves the state).
flight_core.h needed no change: its control path already runs on `fo.tvcActive` / `fo.useLandingGains`
(DRIFT lean included). No events are pushed (kept out of the log on purpose, so sim handlers see nothing new).
Caveats: ChuteDeploy's canopy detector can read the late thrust as a canopy and stop re-cycling the latch
(left as is, chute behaviour must not change); legs nichrome stays off (LANDING_BURN-only).

## T7a D2 (flight_state.h)
Ported from `final_S3_D2.patch` only the D2 rule of `LandingSolver::tableRule` (TABLE mode; RISK mode not ported;
the S3 drift loop is already DRIFT_GAIN). Logic:
`tiltOk = tilt < 15 deg; if (!tiltOk && D2 && rateValid) tiltOk = tilt + rate*IGNITION_DELAY_MS/1000 < 15 deg && tilt < 25 deg;`
`interlocks = tiltOk && kfHealthy && contLanding`.
Tilt rate: first-order LPF (5 Hz) of the finite-difference derivative of `in.tiltRad`, computed INSIDE the FSM
(`d2UpdateRate`, every DESCENT tick, at the top of updateDescent), trusted after 25 ticks; reset on entering DESCENT.
So FsmInput did not need a new field and flight_core.h is untouched for D2. The old patch updated the rate on
every solver call in DESCENT (same ticks). Parameters (hard cap, LPF, warm-up) were the old config values
(TILT_IGNITE_HARD_MAX_DEG, TILT_RATE_LPF_HZ, kRateWarmup).

## T7b wind-matched ascent (flight_core.h)
Ported from `ideas_full.patch` (firmware only): `windEstStep` (verbatim apart from the STUDY_WMA_* names), and the B1
boost lean with WIND_MATCH on and WIND_EST on. Per tick in BOOST while tvcActive: estimate, then
`h = -GAIN*(velH - wind)/|a|`, capped at MAX_TILT_DEG, target `(hx,hy,1)/n` via `tiltErrorBodyTo`, only for
`START_S <= t_boost <= ASCENT_BURN_MS/1000 - END_S` (upright outside it, i.e. also the last 0.6 s).
Estimator: sideways specific force minus gimbal share (`ax + az sin(gimbalY)`, `ay - az sin(gimbalX)`, sign checked
against plant.py thrust components) = normal force/m = -k*v_rel with `k = 0.5*1.225*CNaAref/MASS_DESCENT_KG*speed`;
k^2-weighted running mean, x SCALE (0.8), shrunk toward 0 by measured noise vs PRIOR_MPS (1.0).
Judgement calls: (1) the old patch's `windEstimate()` oracle hook and the GPS / ATH / S6 / COAST grace parts were
NOT ported; (2) `tiltErrorBodyTo` already existed in ahrs.h (S3), nothing added; (3) wind accumulators are cleared
on the pad (the old code never reset them; harmless in the sim where each flight builds a new core);
(4) the burn time is cfg::ASCENT_BURN_MS (2140), the old patch used the same name; (5) the estimator uses the
COMMANDED gimbal angle (ctl_.gimbalX/Y), so gimbal_offset_deg (unknown to the firmware) biases it by design.
(6) The estimator constants (CNa*Aref 0.00884 m^2) are the study's ESTIMATE, not re-fitted to the present plant.
