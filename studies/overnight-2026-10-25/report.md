# Overnight sims 2026-10-25 (report-only; in progress)

Machine: 4 CPUs. Raw JSONs kept outside the repo.

Note: test_bridge.py needed a workaround: tools/synth_flight.py reads src/core/burn_table.h with the default (UTF-8) encoding but the file contains a cp1252 byte (0x97). I did NOT change the repo; I ran with a sitecustomize shim (outside the repo) defaulting open() to cp1252. Fix would be encoding='cp1252' (or removing the em dash) in synth_flight.py/burn_table.h. Both test_bridge.py and test_landing_outcome.py pass.

## Job 1: Holdout (seeds 701-710, 2000 flights each; previous version MEAS = 18.15 %)

| Run | Success [95 % range] | Calm wind | Powered | Aborts | Hit too hard | Lifted off | Hopped | Legs late | Tipped | Contact vz / vh (m/s) | Apogee (m) | Fall at fire (m/s) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| hold_measured | **26.0 %** [24.1-28.0] (520/2000) | 133/505 | 100.0 % | 0.0 % | 69.3 % | 37.7 % | 0.0 % | 0.5 % | 18.2 % | 3.8 / 0.44 | 73.2 | 18.6 |
| hold_massguess | **8.5 %** [7.3-9.8] (169/2000) | 44/505 | 100.0 % | 0.0 % | 89.1 % | 47.3 % | 1.1 % | 0.2 % | 37.2 % | 6.0 / 0.43 | 73.5 | 18.7 |

## Job 2: Fire margin (DEV, MEAS)

| Run | Success [95 % range] | Calm wind | Powered | Aborts | Hit too hard | Lifted off | Hopped | Legs late | Tipped | Contact vz / vh (m/s) | Apogee (m) | Fall at fire (m/s) | Paired vs base |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| margin_1.0 | **22.3 %** [19.2-25.8] (134/600) | 33/143 | 100.0 % | 0.0 % | 73.8 % | 41.3 % | 0.0 % | 0.0 % | 17.3 % | 3.9 / 0.46 | 73.1 | 18.6 | (base) |
| margin_0.0 | **20.8 %** [17.8-24.3] (125/600) | 35/143 | 100.0 % | 0.0 % | 74.2 % | 67.0 % | 0.0 % | 0.2 % | 38.2 % | 4.2 / 0.45 | 73.1 | 18.8 | 125 rescued / 134 lost, p = 0.619 (600 flights) |
| margin_0.5 | **23.3 %** [20.1-26.9] (140/600) | 35/143 | 100.0 % | 0.0 % | 72.8 % | 53.2 % | 0.0 % | 0.0 % | 27.3 % | 4.0 / 0.46 | 73.1 | 18.7 | 77 rescued / 71 lost, p = 0.681 (600 flights) |
| margin_1.5 | **22.2 %** [19.0-25.7] (133/600) | 28/143 | 100.0 % | 0.0 % | 74.5 % | 27.3 % | 0.0 % | 0.0 % | 10.3 % | 4.2 / 0.46 | 73.1 | 18.5 | 84 rescued / 85 lost, p = 1 (600 flights) |
| margin_2.0 | **20.8 %** [17.8-24.3] (125/600) | 34/143 | 100.0 % | 0.0 % | 77.3 % | 16.7 % | 0.0 % | 0.0 % | 6.7 % | 4.6 / 0.46 | 73.1 | 18.4 | 125 rescued / 134 lost, p = 0.619 (600 flights) |
