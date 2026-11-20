# Overnight run 2026-11-20: report

The setup is today's `main` at the measured 1.477 kg (pad) / 1.4563 kg (descent), with E18W/E18W. The sim settings follow PLAN.md:
- measured spreads (mass ±1 %, ignition delay ±5 %);
- band cut 0.75 s;
- wind uniform −4…+4 m/s unless a wind is named.

The CSVs (one row per flight) and `results/summary.md` are in `results/`; the scripts are in `scripts/`. Pairs are matched by (seed, i). p is a two-sided sign test on landings rescued (+) vs lost (−).

**Apogee:** every one of the ~100 setups had a median 15.3–15.5 m, a p99 of 16.0 m and a max of 16.0 m. Nothing came near the 30.48 m (100 ft) limit.

## Recommended next steps
1. **Measure the landing igniter's delay spread.** Fire 10+ e-matches on the static stand and record the time from fire command to thrust. Ignition-delay error is the main reason landings fail (task 6):
   - with every other spread present, the landing works only when the true delay is within about −15…+30 ms of `IGNITION_DELAY_MS` (620 ms);
   - 20 ms early gives 37 % success, 30 ms early gives 8 %, and 620 ms exact gives 99 %.

   No firmware setting fixes a spread wider than that.
2. **Port the "window passed" abort (task 5): `patches/task5_window_passed_abort.patch`, with `WINDOW_PASSED_D_M = 0.5f`.** It follows the owner's chute-ASAP rule:
   - on a missed window, the chute opens at 13.7 m instead of 3.7 m;
   - it lost zero landings on the holdout, on fresh seeds, and at 5, 6 and 7 m/s;
   - the patch file was checked flight for flight against the study switch.
3. **Wind-matched ascent (task 7b):** RESULT_7B
4. **Burn-table margin 0.75 (task 4): owner's choice, not a blanket port.** `patches/task4_burn_table_margin_0.75.patch`.
   - It gains about 8 points in the default −4…+4 m/s wind (holdout and fresh seeds, both significant).
   - It loses 11–17 points in steady 5–7 m/s wind.
   - Port it only if you fly in ≤ 4 m/s.
5. **No change** for the dud window (task 6), the D2 interlock, `DRIFT_GAIN`, `DRIFT_MAX_TILT_DEG` (task 7) and the TVC gains (task 8).

## T0: baseline at 1.477 kg

| Set | Flights | Success | Failures |
|---|---|---|---|
| holdout 701–710 | 2,000 | **78.2 %** | LEGS_BROKEN 409 (20.5 %), LIFTED_OFF 13, LEGS_NOT_DOWN 11, TIPPED_OVER 2 |
| fresh 801–810 | 2,000 | 76.7 % | (pairing base for the fresh confirmations) |
| dev 101–103 | 600 | 78.2 % | LEGS_BROKEN 123, LEGS_NOT_DOWN 5, LIFTED_OFF 3 |

- **Contact speed** (holdout): median 1.97 m/s, p90 3.45 m/s. Every LEGS_BROKEN hit at ≥ 3.0 m/s (median 3.44).
- **No aborts** at the default wind.

**Wind sweep** (300 flights each):

| Wind (m/s) | 0 | 2 | 4 | 5 | 6 | 7 |
|---|---|---|---|---|---|---|
| Success | 74.7 % | 79.7 % | 86.3 % | 86.0 % | 75.3 % | 31.7 % |
| Top failure | LEGS_BROKEN 75 | LEGS_BROKEN 58 | LEGS_BROKEN 30 | LEGS_NOT_DOWN 17 | LEGS_NOT_DOWN 53 | NO_POWERED_LANDING 142 (TILT abort in DESCENT) |

- **Aborts:** at 6 m/s there were 4 TILT and 2 MISSED_WINDOW, all in DESCENT. At 7 m/s there were 142 TILT aborts, all in DESCENT.
- **The wind limit is about 6 m/s, not about 4.** Calm air is *worse* than 4–5 m/s, because it has more hard (≥ 3 m/s) contacts.

## Task 1: `chute wiggle` bench command (shipped)
- **The command:** `chute wiggle` drives a real `ChuteDeploy` at 0 g / 100 m.
  - It maps each target through the new `Actuators::writeChuteCmd`, the one mapping that `applyChute` now uses as well.
  - It stops at OPEN after the last stroke.
  - It is disarmed only and feeds the watchdog.
  - It prints `+0 ms WIG_HIGH 110°`, `+190 WIG_LOW 55°`, `+340 110°`, `+490 55°`, `+640 OPEN 180°`.
  - It saves the last position and prints the `chute set` / load / `chute lock` reminder.
- **Docs:** help text and a README bench row carrying the owner's checks.
- **Test:** the PC test `tools/hil/test_wiggle.py` drives the HIL host with a simulated clock and a logging mock Servo. It asserts the 5 pulses (1722 / 1111 / 1722 / 1111 / 2500 µs at 0 / 190 / 340 / 490 / 640 ms, ±4 ms; it measured 0 ms error) and that an armed `chute wiggle` is refused.
- **Result-neutral:** the sim flies only `src/core`, and `src/hw` is not in the sim build. The task-1 tree's dev 600 is identical to T0 flight for flight (RESULT_T1). `test_hil.py` and `test_live.py` (12 cases) also pass on the refactored `applyChute`.
- **Suites, all green:** replay `make test`, `test_bridge`, `test_landing_outcome`, `test_hil`, `test_live`, `test_wiggle`, `test_fit_cda`.
- **Review:** the head chef said YES. Two nits are fixed: the timeout is now derived from the wiggle dwells, and the bias line is relabelled.

## Task 2: `tools/fit_cda.py` (shipped)
- **Method:**
  - The fit uses the signed body-axial accel (`az`, m/s² in the log; +Z is the nose) and fits `a = c·v² + b` per phase. The intercept absorbs the accel bias, and CdA = 2mc/ρ ± 1σ.
  - The phases are COAST (ascending, after a 0.25 s motor tail-off skip) and DESCENT (falling, up to the first chute or landing-fire bit), using |kf_vel| ≥ 2 m/s.
  - A JOINT fit (coast + descent, one CdA, one bias) separates bias from drag through the opposite drag signs. A POOLED fit (several logs, one bias per log) combines flights.
  - `--mass` defaults to `MASS_DESCENT_KG`; `--rho` defaults to 1.225, or use `--alt-m` / `--temp-c`.
  - The tool warns on n < 50, a narrow speed range, or 1σ > 25 %.
- **plant.py:** the sim applies the same `cda_m2` nose-first and tail-first, on airspeed including wind, along body Z, at a fixed ρ = 1.225.
- **Validation** (`tools/test_fit_cda.py`, plant CdA 0.003 / 0.004 / 0.006):
  - noiseless IMU: every phase within ±5 %;
  - default sensor noise: the JOINT median over 15 seeds is within ±6 %;
  - POOLED over 15 flights: +1.6 % ± 9 %.
- **Caveat: one flight of this rocket gives CdA only to ±25–45 %.** Drag is about 0.05 m/s², against 0.12 m/s² of accel noise, at ≤ 6 m/s. The coast phase alone is unusable (±300 %+), and chute-mode logs have no bare descent. Pool several flights.

## Task 4: burn-table margin (report; conditional patch)
**Dev, 600 flights each** (base 1.0 = 78.2 %):

| Margin (m) | 0 | 0.25 | 0.5 | 0.625 | 0.7 | **0.75** | 0.8 | 0.875 | 1.0 | 1.25 | 1.5 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| Success | 42.5 % | 60.7 % | 74.8 % | 82.3 % | 85.0 % | **86.0 %** | 85.8 % | 82.8 % | 78.2 % | 62.8 % | 44.7 % |

- **The trade:** a lower margin lights later, which means fewer LEGS_BROKEN but more LEGS_NOT_DOWN (legs not swung down at contact).
  - At 1.0: LEGS_BROKEN 123, LEGS_NOT_DOWN 5.
  - At 0.75: LEGS_BROKEN 39, LEGS_NOT_DOWN 36.
- **Holdout (0.75 vs 1.0):** 78.2 → 86.2 %, +288 / −129, p = 5e-15.
- **Fresh seeds:** 76.7 → 85.2 %, +297 / −127, p = 9e-17. Both gates pass.
- **Steady wind (300 each), vs 1.0:**

| Wind | 0 | 2 | 4 | 5 | 6 | 7 |
|---|---|---|---|---|---|---|
| 0.75 | +10.0 (p=7e-5) | +5.3 (p=0.04) | −3.6 (n.s.) | **−12.7** (p=7e-8) | **−16.6** (p=2e-13) | **−10.7** (p=5e-10) |
| 0.875 | +6.3 | — | 0 | −4.7 | −7.0 | −3.4 (all p<0.005 except 4) |

- **Why:** in steady wind the vehicle needs more time for the legs (LEGS_NOT_DOWN grows), so a later light hurts.
- **Recommendation:** not a blanket port.
  - `patches/task4_burn_table_margin_0.75.patch` is there for an owner who flies only in ≤ 4 m/s.
  - Otherwise keep 1.0.
  - The root cause is the ignition-delay sensitivity in task 6.

## Task 5: earlier missed-window abort, chute ASAP (patch recommended)
- **Variants**, in the scratch copy, off by default:
  - (a) `MISSED_WINDOW_ALT_M` 5 / 6 / 8;
  - (b) "window passed": abort when `kfAlt < hIgnite(vDown) + margin − D` while the interlocks fail, D = 0.5 / 1 / 2 m.
- **Missed windows are rare:** 0 at the default wind and at 5 m/s, and 16 / 600 at 6 m/s (seeds 101–103). The other descent aborts there are TILT.

| Variant | Landings: holdout / w5 / w6 (paired) | Chute release height on missed-window flights, w6 (base 3.73 m) |
|---|---|---|
| alt 5 / 6 / 8 m | +0/−0 everywhere | 4.72 / 5.74 / 7.77 m |
| window passed D 0.5 / 1 / 2 m | +0/−0 everywhere | **13.71** / 13.58 / 13.31 m (p10 13.44; 100 % above 8 m) |

- No flight aborted early that would have landed.
- **Fresh seeds, D = 0.5:**
  - default wind: +0 / −0;
  - 6 m/s: 38 missed windows, released at 13.67 m vs 3.72 m, +0 / −0 landings;
  - 7 m/s dev: RESULT_5W7.
- **Port check:** the unapplied patch (`WINDOW_PASSED_D_M = 0.5f`) reproduces the study switch flight for flight (600 / 600 at 6 m/s). With 0 it is identical to today's code.
- **Recommendation: port it at D = 0.5 m.** The release height goes up about 10 m and no landing is lost.

## Task 6: late-ignition risk (report only)
- **The motor-timing window.** True plant delay, no delay spread, everything else measured, 300 each, `DUD_WINDOW_MS` 200:

| True delay (s) | 0.56 | 0.58 | 0.60 | **0.62** | 0.64 | 0.66 | 0.68 | 0.70 | 0.76 | 0.80–1.00 |
|---|---|---|---|---|---|---|---|---|---|---|
| Success | 0 | 0 | 39 % | **99.3 %** | 98.7 % | 32 % | 0 | 0 | 0 | 0 |
| Failure | LEGS_BROKEN | LEGS_BROKEN | LEGS_BROKEN | — | — | LEGS_NOT_DOWN | LEGS_NOT_DOWN | LEGS_NOT_DOWN | LEGS_NOT_DOWN (7 m/s hits) | dud declared |

- **Margin 0.75 moves the window about 10–15 ms toward shorter delays:** 0.58 → 8 %, 0.60 → 89 %, 0.62 → 100 %, 0.64 → 77 %, 0.66 → 3 %.
- **False duds:** with the 200 ms window, every light later than about 0.78 s is declared a dud (100 % at 0.80 s). None are declared at ≤ 0.76 s.
- **What happens after a late light, at window 200:**
  - The chute is released at about 5.4 m and TVC goes off, then the motor lights.
  - Median max height after the light is 5.7 / 5.2 / 4.4 / 3.0 m at 0.80 / 0.84 / 0.90 / 1.00 s, so no climb-away.
  - Median contact is 2.4 / 2.8 / 4.7 / 7.0 m/s, with 3.0 / 2.6 / 2.2 / 1.7 m/s sideways.
  - All end NO_POWERED_LANDING.
  - The sim can't model exhaust damage to the canopy.
- **Trade-off curve:**

| DUD_WINDOW_MS | Lights declared dud | Real dud: chute release height (median) | Real dud: impact |
|---|---|---|---|
| 200 | ≥ 0.80 s | 5.42 m | 11.1 m/s |
| 300 | ≥ 0.90 s | 3.98 m | 13.6 m/s |
| 400 | ≥ 1.00 s | 2.45 m | 15.8 m/s |

- **A longer window buys nothing.** A late light that the window "saves" (e.g. 0.84 s at 300 / 400) still fails: 0 % success, 9.8 m/s contact, and a hop to 11.5 m. Meanwhile real duds hit 2.5–4.7 m/s harder. Keep 200.
- **Guard variant** (after a DUD_IGNITER abort, re-enable TVC with landing gains on a real thrust rise):
  - The trigger needs a ≥ 1 g rise within 30 ms, within 400 ms of the abort. A plain g threshold also fired on the canopy jolt, so it was rejected.
  - Never-lit duds: identical to no guard (300 / 300).
  - Late lights at 0.80 / 0.84 / 0.90 s: sideways contact drops from 3.0 / 2.6 / 2.2 to 0.8 / 1.0 / 1.3 m/s, and the median max tilt after the light drops from 17 / 10 / 9.5° to 7 / 7 / 7.5°.
  - Success stays 0 %.
  - At 1.00 s there is no effect: the light comes too low.
  - **Recommendation: no change.** It's a small damage reduction for a case that can't land anyway. Not worth new flight code now.

## Task 7: shelved fixes on the E18W (report)
Wind sweep, 300 each, paired with T0 at the same wind:

| Variant | 0 | 2 | 4 | 5 | 6 | 7 m/s |
|---|---|---|---|---|---|---|
| T0 | 74.7 | 79.7 | 86.3 | 86.0 | 75.3 | 31.7 % |
| (a) D2 interlock | ±0 | ±0 | ±0 | ±0 | ±0 | ±0 |
| (b) wind-matched ascent | **79.7** (p=0.003) | 81.0 | 80.7 (p=0.003) | 81.3 (n.s.) | **83.3** (p=0.02) | **83.3** (+172/−17, p=2e-33) |
| (c) DRIFT_GAIN 1 / 3 / 4 | ≈0 | ≈0 | ≈0 | ≈0 | ≈0 | ≈0 (all p > 0.2) |
| (d) DRIFT_MAX_TILT 3° | ≈0 | ≈0 | ≈0 | ≈0 | ≈0 | ≈0 |
| (d) DRIFT_MAX_TILT 8° | +0.3 | 0 | +1.7 | −1.0 | **−4.0** (p=0.004) | −2.0 |

- **D2 never changes a flight.** In this plant the tilt keeps growing through the descent (about 14°/s), so the predicted tilt never relaxes the 15° rule. The code path was proven live with a sign-flipped test build.
- **Wind-matched ascent:**
  - It holds about 80–84 % from 0 to 7 m/s, the same at every wind.
  - It loses at 4 m/s.
  - Gimbal-zero error ±0.5° (plant offset): at 0 m/s, 84.7 / 83.0 % vs 79.7 % without the offset, so no loss. At 5 m/s, 84.7 / 84.3 % vs 81.3 %.
  - Holdout / fresh: RESULT_7B_GATE.

## Task 8: TVC gains (report only)
- **Boost gains:** kp, kd ×0.5…2 and ki ×0 / 2 change nothing (≤ 2 flights per 300). The boost is short and the vehicle is stiff.
- **Landing gains, one at a time:** only kd ×0.5 stood out (+16 / −5, p = 0.03). kd ×2 was worse (+1 / −7).
- **3 × 3 grid** (kp ×0.75 / 1 / 1.5 × kd ×0.35 / 0.5 / 0.65): 74.7–83.3 %. Best were kd ×0.5 (F1) and kp ×0.75 + kd ×0.35 (F2).
- **Finalists** (paired):

| | Holdout | Fresh | Wind 5 (dev) | Inertia −30 % | Inertia +30 % | Slow servo (240 °/s, 16 ms lag) |
|---|---|---|---|---|---|---|
| F1 kd 0.065 | 78.2→83.0 (p=9e-15) | 76.7→82.2 (p=6e-20) | 86.2→**79.8** (p=6e-6) | 78.0→80.8 | 78.7→83.7 | 69.2→**47.0** (p=1e-26) |
| F2 kp 0.6, kd 0.0455 | 78.2→83.2 (p=1e-11) | 76.7→82.4 (p=7e-15) | 86.2→**75.7** (p=3e-11) | 78.0→82.3 | 78.7→83.7 | 69.2→**57.5** (p=6e-11) |

- **Max tilt and saturation** (screen): median max tilt 7.0° (base) vs 8.8° (F1). Gimbal-saturation share 2.3 % vs 6.6 %. Sideways contact is unchanged (0.16 vs 0.21 m/s).
- **Recommendation: no change.** Both win in calm and mild wind but fail the robustness gate, badly with a slower servo.
- **Caveat:** inertia and the servo model are estimates, so gains need bench and flight checks anyway. The lower kd mostly shifts the burn timing (fewer LEGS_BROKEN, more LEGS_NOT_DOWN), much like the task 4 margin.
