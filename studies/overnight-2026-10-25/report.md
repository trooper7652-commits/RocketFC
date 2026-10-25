# Overnight sims 2026-10-25 (report-only)

## Summary (plain language)

- **Landing success is up from 18.15 % to 26.0 %** on the same holdout seeds (2,000 flights) with mass weighed and delay measured. With mass still a ±5 % guess it is only **8.5 %**, so weighing the rocket matters a lot (it was the biggest lever: ~17 points).
- Despite the better numbers, the main failure is still **hitting the ground too hard** (~70 % of flights over the 3 m/s crash limit; median contact speed 3.8 m/s). Tipping over is next (~18 %).
- **Fire margin:** 0.5 m scored best on the 600-flight dev set (23.3 % vs 22.3 % at 1.0), but on fresh seeds 801-810 it did *worse* (23.4 % vs 25.9 %; 265 rescued vs 314 lost, p = 0.046 against it). **Verdict: fails the bar. Keep BURN_TABLE_MARGIN_M = 1.0.** (config.h untouched.)
- **Wind:** success is flat from 0 to 6 m/s (23-25 %). At 8 m/s it collapses to **0 %**. Highest tested wind that's fine: **6 m/s**; the cliff lies somewhere between 6 and 8 m/s (7 not tested).
- **F15 thrust spread is costly:** ±3 % → 16.3 % (-6 pts, p = 0.012), ±5 % → 8.3 %, ±10 % → 4.0 %, versus 22.3 % at nominal. Motor-to-motor variation is a major risk; thrust-stand data per motor would help.
- **Top failure causes (landing_report, after measuring):** hit faster than crash speed 71.7 %, motor lifted it back off the ground 4.2 %, legs late 0.3 %. Among falls-over, thrust is the main push (83 %). Today's-guess case: 84.5 % too-hard hits.

## Run info

- Machine: 4 CPUs (`--workers 4`).
- Elapsed: hold_measured 179 s, hold_massguess 188 s, each margin run 53-57 s, margin confirmation (2 x 2,000 flights) 180 s + 183 s, each wind run 46-55 s, var_3/5/10 56-57 s each, landing_report 724 s. All jobs finished with 0 errors.
- Deviation 1: `test_bridge.py` initially failed because `tools/synth_flight.py` opens `src/core/burn_table.h` with default UTF-8 but the file contains a cp1252 byte (0x97). The repo was NOT changed; I ran everything with a sitecustomize shim outside the repo that defaults `open()` to cp1252. Real fix: `encoding="cp1252"` in synth_flight.py (outside tools/sim, so left alone) or replace the dash in burn_table.h. Both sanity tests then passed.
- Deviation 2: landing_report flew 7,600 flights, not the 8,800 the task cited.
- Observed oddity: in the margin sweep, margin_0.0 and margin_2.0 both show "125 rescued / 134 lost" vs base, i.e. zero overlap in successes, which looks suspicious (possibly a mc_summary pairing quirk); doesn't change the verdict.

## Job 1: Holdout (seeds 701-710; previous version MEAS = 18.15 %)

Previous: 500 ms delay, 0.25 m foot radius, old chute model. Now: 620 ms delay, 0.711 m foot radius, fixed chute model.

| Run | Success [95 % range] | Calm wind | Powered | Aborts | Hit too hard | Lifted off | Hopped | Legs late | Tipped | Contact vz / vh (m/s) | Apogee (m) | Fall at fire (m/s) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| hold_measured | **26.0 %** [24.1-28.0] (520/2000) | 133/505 | 100.0 % | 0.0 % | 69.3 % | 37.7 % | 0.0 % | 0.5 % | 18.2 % | 3.8 / 0.44 | 73.2 | 18.6 |
| hold_massguess | **8.5 %** [7.3-9.8] (169/2000) | 44/505 | 100.0 % | 0.0 % | 89.1 % | 47.3 % | 1.1 % | 0.2 % | 37.2 % | 6.0 / 0.43 | 73.5 | 18.7 |

## Job 2: Fire margin (DEV seeds 101-103, MEAS)

| Run | Success [95 % range] | Calm wind | Powered | Aborts | Hit too hard | Lifted off | Hopped | Legs late | Tipped | Contact vz / vh (m/s) | Apogee (m) | Fall at fire (m/s) | Paired vs base |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| margin_1.0 | **22.3 %** [19.2-25.8] (134/600) | 33/143 | 100.0 % | 0.0 % | 73.8 % | 41.3 % | 0.0 % | 0.0 % | 17.3 % | 3.9 / 0.46 | 73.1 | 18.6 | (base) |
| margin_0.0 | **20.8 %** [17.8-24.3] (125/600) | 35/143 | 100.0 % | 0.0 % | 74.2 % | 67.0 % | 0.0 % | 0.2 % | 38.2 % | 4.2 / 0.45 | 73.1 | 18.8 | 125 rescued / 134 lost, p = 0.619 (600 flights) |
| margin_0.5 | **23.3 %** [20.1-26.9] (140/600) | 35/143 | 100.0 % | 0.0 % | 72.8 % | 53.2 % | 0.0 % | 0.0 % | 27.3 % | 4.0 / 0.46 | 73.1 | 18.7 | 77 rescued / 71 lost, p = 0.681 (600 flights) |
| margin_1.5 | **22.2 %** [19.0-25.7] (133/600) | 28/143 | 100.0 % | 0.0 % | 74.5 % | 27.3 % | 0.0 % | 0.0 % | 10.3 % | 4.2 / 0.46 | 73.1 | 18.5 | 84 rescued / 85 lost, p = 1 (600 flights) |
| margin_2.0 | **20.8 %** [17.8-24.3] (125/600) | 34/143 | 100.0 % | 0.0 % | 77.3 % | 16.7 % | 0.0 % | 0.0 % | 6.7 % | 4.6 / 0.46 | 73.1 | 18.4 | 125 rescued / 134 lost, p = 0.619 (600 flights) |

Confirmation on fresh seeds 801-810 (best alternative 0.5 vs 1.0):

| Run | Success [95 % range] | Calm wind | Powered | Aborts | Hit too hard | Lifted off | Hopped | Legs late | Tipped | Contact vz / vh (m/s) | Apogee (m) | Fall at fire (m/s) | Paired vs base |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| conf_1.0 | **25.9 %** [24.0-27.9] (518/2000) | 139/469 | 100.0 % | 0.0 % | 70.1 % | 38.4 % | 0.0 % | 0.6 % | 17.9 % | 3.8 / 0.44 | 73.2 | 18.6 | (base) |
| conf_0.5 | **23.4 %** [21.6-25.4] (469/2000) | 105/469 | 100.0 % | 0.0 % | 71.5 % | 54.0 % | 0.0 % | 0.6 % | 27.3 % | 3.9 / 0.44 | 73.2 | 18.7 | 265 rescued / 314 lost, p = 0.046 (2000 flights) |

Verdict: 0.5 fails the bar (rescued 265 < lost 314, p = 0.046 against). Keep 1.0.

## Job 3: Launch-wind limit

| Run | Success [95 % range] | Calm wind | Powered | Aborts | Hit too hard | Lifted off | Hopped | Legs late | Tipped | Contact vz / vh (m/s) | Apogee (m) | Fall at fire (m/s) | Paired vs base |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| wind_0 | **23.0 %** [19.8-26.5] (138/600) | 138/600 | 100.0 % | 0.0 % | 73.3 % | 36.7 % | 0.0 % | 0.0 % | 15.3 % | 4.0 / 0.45 | 73.1 | 18.6 | (base) |
| wind_1 | **23.3 %** [20.1-26.9] (140/600) | 0/0 | 100.0 % | 0.0 % | 72.8 % | 37.5 % | 0.0 % | 0.0 % | 15.8 % | 4.0 / 0.48 | 73.1 | 18.6 | 9 rescued / 7 lost, p = 0.804 (600 flights) |
| wind_2 | **24.5 %** [21.2-28.1] (147/600) | 0/0 | 100.0 % | 0.0 % | 72.2 % | 38.7 % | 0.0 % | 0.0 % | 17.0 % | 3.9 / 0.51 | 73.0 | 18.6 | 21 rescued / 12 lost, p = 0.163 (600 flights) |
| wind_3 | **24.5 %** [21.2-28.1] (147/600) | 0/0 | 100.0 % | 0.0 % | 72.5 % | 39.8 % | 0.0 % | 0.0 % | 18.7 % | 4.0 / 0.56 | 73.0 | 18.6 | 28 rescued / 19 lost, p = 0.243 (600 flights) |
| wind_4 | **24.5 %** [21.2-28.1] (147/600) | 0/0 | 100.0 % | 0.0 % | 72.3 % | 41.5 % | 0.0 % | 0.0 % | 21.0 % | 4.0 / 0.63 | 73.0 | 18.6 | 38 rescued / 29 lost, p = 0.328 (600 flights) |
| wind_6 | **24.8 %** [21.5-28.4] (149/600) | 0/0 | 100.0 % | 0.0 % | 72.5 % | 43.8 % | 0.0 % | 0.0 % | 20.3 % | 4.0 / 1.03 | 72.9 | 18.6 | 57 rescued / 46 lost, p = 0.324 (600 flights) |
| wind_8 | **0.0 %** [0.0-0.6] (0/600) | 0/0 | 100.0 % | 0.0 % | 100.0 % | 100.0 % | 0.0 % | 100.0 % | 100.0 % | 18.9 / 4.60 | 72.9 | 23.1 | 0 rescued / 138 lost, p = 5.74e-42 (600 flights) |

Highest wind not clearly below calm: 6 m/s (8 m/s: 0 %).

## Job 4: F15 thrust variation (base = margin_1.0, nominal thrust)

| Run | Success [95 % range] | Calm wind | Powered | Aborts | Hit too hard | Lifted off | Hopped | Legs late | Tipped | Contact vz / vh (m/s) | Apogee (m) | Fall at fire (m/s) | Paired vs base |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| margin_1.0 | **22.3 %** [19.2-25.8] (134/600) | 33/143 | 100.0 % | 0.0 % | 73.8 % | 41.3 % | 0.0 % | 0.0 % | 17.3 % | 3.9 / 0.46 | 73.1 | 18.6 | (base) |
| var_3 | **16.3 %** [13.6-19.5] (98/600) | 30/143 | 100.0 % | 0.0 % | 81.0 % | 42.3 % | 8.3 % | 0.2 % | 23.8 % | 4.9 / 0.46 | 73.1 | 18.6 | 80 rescued / 116 lost, p = 0.0122 (600 flights) |
| var_5 | **8.3 %** [6.4-10.8] (50/600) | 13/143 | 100.0 % | 0.0 % | 91.7 % | 43.7 % | 25.0 % | 0.2 % | 33.8 % | 6.0 / 0.45 | 73.1 | 18.6 | 43 rescued / 127 lost, p = 7.96e-11 (600 flights) |
| var_10 | **4.0 %** [2.7-5.9] (24/600) | 5/143 | 100.0 % | 0.0 % | 94.8 % | 47.2 % | 39.7 % | 0.2 % | 56.7 % | 8.6 / 0.44 | 73.1 | 18.6 | 22 rescued / 132 lost, p = 2.57e-20 (600 flights) |

## Job 5: Failure-cause report

See landing_report/report.md, causes.png, crash_speed_sweep.png in this folder.
