# Overnight run 2026-11-20: report

The setup is today's `main` at the measured 1.477 kg (pad) / 1.4563 kg (descent), with E18W/E18W. The sim settings follow PLAN.md:
- measured spreads (mass ±1 %, ignition delay ±5 %);
- band cut 0.75 s;
- wind uniform −4…+4 m/s unless a wind is named.

The CSVs (one row per flight) and `results/summary.md` are in `results/`; the scripts are in `scripts/`. Pairs are matched by (seed, i). p is a two-sided sign test on landings rescued (+) vs lost (−).

**Apogee:** in all 234 jobs the median was 15.3–15.5 m and the max 16.0 m (median / p99 / max per setup in `results/summary.md`). Nothing came near the 30.48 m (100 ft) limit.

## Recommended next steps
1. **Measure the landing igniter's delay spread.** Fire 10+ e-matches on the static stand and record the time from fire command to thrust. Ignition-delay error is the main reason landings fail (task 6):
   - with the true delay fixed and every other spread present, success is 99.3 % at 620 ms and 98.7 % at 640 ms;
   - 20 ms early (600 ms) gives 39 %, 40 ms early gives 0 % (all LEGS_BROKEN);
   - 40 ms late (660 ms) gives 32 %, 60 ms late gives 0 % (LEGS_NOT_DOWN);
   - so the usable band is roughly 605–655 ms.

   Today's assumed ±5 % (±31 ms) spread alone explains most of the 22 % failures.

   No firmware setting fixes a spread wider than that.
2. **Port the "window passed" abort (task 5): `patches/task5_window_passed_abort.patch`, with `WINDOW_PASSED_D_M = 0.5f`.** It follows the owner's chute-ASAP rule:
   - on a missed window, the chute opens at 13.7 m instead of 3.7 m;
   - it lost zero landings on the holdout, on fresh seeds, and at 5, 6 and 7 m/s;
   - the patch file was checked flight for flight against the study switch.
3. **Port the wind-matched ascent together with margin 0.75 (tasks 7b + 4), after two bench checks.** Apply `patches/task7_wind_matched_ascent.patch` (set `WIND_MATCH_ASCENT = 1`) and `patches/task4_burn_table_margin_0.75.patch`.
   - Holdout: 78.2 → 84.7 % (p = 8e-10). Fresh seeds: 76.7 → 84.8 % (p = 4e-14).
   - It holds 84–86 % from 0 to 6 m/s, 79 % at 7 m/s (today 32 %) and 65 % at 8 m/s (today 6 %).
   - Never-used seeds 901–910 (run after the design): 77.7 → 82.8 % (+297 / −194, p = 4e-6). The gain is smaller there (+5.1 pts) because this set has more LEGS_NOT_DOWN (177 vs 122 on the holdout).
   - It is not worse at any wind tested. The dips at 4 and 5 m/s (−1.4 / −1.0 pts on 600 flights) are not significant.
   - The two patches applied together reproduce the study copy flight for flight (300 / 300 at 6 m/s).
   - **Port it only together with the task 5 patch.** The ascent change creates a few new missed-window aborts at the default wind where today has none: 3 / 2,000 on the holdout, 8 / 2,000 fresh, 11 / 2,000 on seeds 901–910.
     - Without task 5, their chute opens at about 3.7 m and they hit at 13.8–14.2 m/s.
     - With task 5, it opens at 13.3–13.6 m and they hit at about 7.2 m/s.
   - **Bench checks first:**
     - **Gimbal zero.** Trim it to within ±0.2°. With a 0.5° error at 5 m/s, the change is *worse* than today's code: 85.7 → 77.7 % (p = 1e-4) and 86.7 → 82.0 % (p = 0.01), i.e. −4.7…−8.0 pts.
     - **Servo speed.** The sim assumes 400 °/s and 8 ms lag (`vehicle.py`). At 240 °/s and 16 ms the change is worse than today: 69.2 → 63.8 % (p = 0.04).
     - **Rule:** if the measured servo is slower than about 400 °/s / 8 ms, don't port. Re-run the study with the measured values first.
   - **Why this passes robustness when task 8's gains didn't:** task 8 lost 6–22 pts in wind and with a slow servo, with no upside in wind. Here the loss is 5–8 pts under a trimmable gimbal error (and 5 pts with a slow servo), against a 10–60 pt gain at 6–8 m/s.
   - Neither half alone passes. The ascent alone is not significant on the holdout (p = 0.42) and loses 8 pts at 4 m/s. The margin alone loses 11–17 pts at 5–7 m/s.
4. **Margin 0.75 alone (task 4): only if the ascent change is not ported, and only for flights in ≤ 4 m/s.** `patches/task4_burn_table_margin_0.75.patch`.
   - It gains about 8 points in the default −4…+4 m/s wind (holdout and fresh seeds, both significant).
   - It loses 11–17 points in steady 5–7 m/s wind.
   - Port it only if you fly in ≤ 4 m/s.
5. **All three patches together** (`task4`, `task5`, `task7`) apply cleanly to today's `src/`, in sequence. With `WIND_MATCH_ASCENT = 1` and `WINDOW_PASSED_D_M = 0.5f`:
   - the landing outcomes are identical to the WMA + 0.75 study run on all 2,000 holdout flights (84.7 %);
   - the 3 missed-window flights release at 13.3–13.6 m instead of 3.7 m (task 5), and their contact drops from about 14 to about 7 m/s;
   - 0 / 5 / 7 m/s (300 each) are identical row for row.
6. **No change** for the dud window (task 6), the D2 interlock, `DRIFT_GAIN`, `DRIFT_MAX_TILT_DEG` (task 7) and the TVC gains (task 8).

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
- **Result-neutral:** the sim flies only `src/core`, and `src/hw` is not in the sim build. The task-1 tree's dev 600 is identical to T0 flight for flight (600 / 600 rows identical). `test_hil.py` and `test_live.py` (12 cases) also pass on the refactored `applyChute`.
- **Suites, all green:** replay `make test`, `test_bridge`, `test_landing_outcome`, `test_hil`, `test_live`, `test_wiggle`, `test_fit_cda`.
- **Review:** the head chef said YES. Two nits are fixed: the timeout is now derived from the wiggle dwells, and the bias line is relabelled.

## Task 2: `tools/fit_cda.py` (shipped)
- **Method:**
  - The fit uses the signed body-axial accel (`az`, m/s² in the log; +Z is the nose) and fits `a = c·v² + b` per phase. The intercept absorbs the accel bias, and CdA = 2mc/ρ ± 1σ.
  - The phases are COAST (ascending, after a 0.25 s motor tail-off skip) and DESCENT (falling, up to the first chute or landing-fire bit), using |kf_vel| ≥ 2 m/s (the plan said 4; lowered on purpose, because the v² weighting handles slow samples and this flight rarely exceeds 6 m/s).
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
- **Missed windows are rare:** 0 at the default wind and at 5 m/s, and 9 / 600 at 6 m/s today (seeds 101–103; 16 / 600 with the earlier rules, which turn 7 TILT aborts into missed-window aborts). At 7 m/s the 281 flights are 280 TILT + 1 MISSED_WINDOW today.

| Variant | Landings: holdout / w5 / w6 (paired) | Chute release height on missed-window flights, w6 (base 3.73 m) |
|---|---|---|
| alt 5 / 6 / 8 m | +0/−0 everywhere | 4.72 / 5.74 / 7.77 m |
| window passed D 0.5 / 1 / 2 m | +0/−0 everywhere | **13.71** / 13.58 / 13.31 m (p10 13.44; 100 % above 8 m) |

- No flight aborted early that would have landed.
- **Fresh seeds, D = 0.5:**
  - default wind: +0 / −0;
  - 6 m/s: 38 missed windows, released at 13.67 m vs 3.72 m, +0 / −0 landings;
  - 7 m/s dev: the 281 flights that end in a descent TILT abort today now abort on the window-passed rule first. Their chute opens at 13.55 m instead of 5.74 m (median), and landings are +0 / −0.
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
| (b) wind-matched ascent | **79.7** (p=0.003) | 81.0 | 80.7 (**−5.6**, p=0.003) | 81.3 (n.s.) | **83.3** (p=0.02) | **83.3** (+172/−17, p=2e-33) |
| (c) DRIFT_GAIN 1 / 3 / 4 | ≈0 | ≈0 | ≈0 | ≈0 | ≈0 | ≈0 (all p > 0.2) |
| (d) DRIFT_MAX_TILT 3° | ≈0 | ≈0 | ≈0 | ≈0 | ≈0 | ≈0 |
| (d) DRIFT_MAX_TILT 8° | +0.3 | 0 | +1.7 | −1.0 | **−4.0** (p=0.004) | −2.0 |

- **D2 never changes a flight.** In this plant the tilt keeps growing through the descent (about 14°/s), so the predicted tilt never relaxes the 15° rule. The code path was proven live with a sign-flipped test build.
- **Wind-matched ascent:**
  - It holds about 80–84 % from 0 to 7 m/s, the same at every wind.
  - It loses at 4 m/s.
  - Gimbal-zero error ±0.5° (plant offset): at 0 m/s, 84.7 / 83.0 % vs 79.7 % without the offset, so no loss. At 5 m/s, 84.7 / 84.3 % vs 81.3 %.
  - Alone, it fails the gate. Holdout 78.2 → 78.8 % (p = 0.42), fresh 76.7 → 77.8 % (p = 0.06). At 4 m/s (dev) it goes 86.7 → 78.8 % (p = 1e-8): LEGS_BROKEN doubles, 58 → 116.
  - At high wind it is a big win:
    - 7 m/s dev: 33.3 → 83.2 % (+338 / −39);
    - 8 m/s: 5.7 → 74.3 %.
- **(b) + margin 0.75**, the combination designed to remove the ascent's LEGS_BROKEN excess, **passes the gate**:

| Wind (m/s) | 0 | 2 | 4 | 5 | 6 | 7 | 8 |
|---|---|---|---|---|---|---|---|
| T0 | 74.7 | 79.7 | 86.3 | 86.0 | 75.3 | 31.7 | 5.7 % |
| WMA + 0.75 | **84.3** (p=0.001) | **85.7** (p=0.03) | 84.0 (n.s.) | 84.0 (n.s.) | **84.7** (p=4e-5) | **79.3** (p=2e-39) | **64.7** (p=1e-50) |

  - Dev at 4 / 5 m/s: 86.7 → 85.3 % (p = 0.35) and 86.2 → 85.2 % (p = 0.42).
  - **How it was found:** designed after the single variants failed. The holdout had been seen for each half, and the 0–2 m/s check uses the screen seeds it was designed on. So fresh seeds and never-used seeds 901–910 are the clean confirmations.
  - Holdout: 78.2 → 84.7 % (+281 / −153, p = 8e-10).
  - Fresh: 76.7 → 84.8 % (+315 / −152, p = 4e-14).
  - Seeds 901–910: 77.7 → 82.8 % (+297 / −194, p = 4e-6).
  - **New low aborts:** missed-window aborts at the default wind are 3 / 8 / 11 per 2,000 (holdout / fresh / 901–910), against 0 today. Their chute opens at 3.7 m unless task 5 is ported too.
  - Against margin 0.75 alone at the default wind: holdout 86.2 → 84.7 % (p = 0.008), fresh 85.2 → 84.8 % (p = 0.6). It gives up about 1 pt at the default wind and gains 10–60 pts at 6–8 m/s.
  - **Robustness**, paired against T0 under the same condition:
    - inertia −30 %: 78.0 → 85.3 %; inertia +30 %: 78.7 → 83.2 %;
    - gimbal offset +0.5 / −0.5°: at 0 m/s, 84.7 / 84.7 % (vs 76.3 / 74.7 %); at 5 m/s, 77.7 / 82.0 % (vs 85.7 / 86.7 %, p = 1e-4 / 0.01);
    - slow servo: 69.2 → 63.8 % (p = 0.04).
  - **Caveats:**
    - The wind estimate uses an estimated `CN_alpha·A_ref` (`WMA_CNA_AREF_M2`) and the commanded gimbal angle, so it should be checked against a real flight log.
    - The sim's crossflow model is itself an estimate.
  - **Recommendation:** port the pair, together with task 5, after trimming the gimbal zero to ±0.2° and confirming the servo is at least 400 °/s / 8 ms.

## Task 8: TVC gains (report only)
- **Boost gains:** kp, kd ×0.5…2 and ki ×0 / 2 change nothing (≤ 2 flights per 300). The boost is short and the vehicle is stiff.
- **Landing gains, one at a time:** only kd ×0.5 stood out (+16 / −5, p = 0.03). kd ×2 was worse (+1 / −7).
- **3 × 3 grid** (kp ×0.75 / 1 / 1.5 × kd ×0.35 / 0.5 / 0.65): 74.7–83.3 %. Best were kd ×0.5 (F1) and kp ×0.75 + kd ×0.35 (F2).
- **Finalists** (paired):

| | Holdout | Fresh | Wind 5 (dev) | Inertia −30 % | Inertia +30 % | Slow servo (240 °/s, 16 ms lag) |
|---|---|---|---|---|---|---|
| F1 kd 0.065 | 78.2→83.0 (p=9e-15) | 76.7→82.2 (p=6e-20) | 86.2→**79.8** (p=6e-6) | 78.0→80.8 | 78.7→83.7 | 69.2→**47.0** (p=1e-26) |
| F2 kp 0.6, kd 0.0455 | 78.2→83.2 (p=1e-11) | 76.7→82.4 (p=7e-15) | 86.2→**75.7** (p=3e-11) | 78.0→82.3 | 78.7→83.7 | 69.2→**57.5** (p=6e-11) |

- **Max tilt and saturation** (screen, 300 each):
  - median max tilt: 7.0° (base) / 8.8° (F1) / 10.6° (F2);
  - gimbal-saturation share of powered time, mean: 2.3 % / 6.6 % / 8.4 %;
  - sideways contact, median: 0.16 / 0.21 / 0.30 m/s (slightly higher, still small);
  - tilt aborts: 0 in every task-8 run.
- **Recommendation: no change.** Both win in calm and mild wind but fail the robustness gate, badly with a slower servo.
- **Caveat:** inertia and the servo model are estimates, so gains need bench and flight checks anyway. The lower kd mostly shifts the burn timing (fewer LEGS_BROKEN, more LEGS_NOT_DOWN), much like the task 4 margin.
