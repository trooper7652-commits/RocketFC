status: RUNNING
heartbeat: 2026-11-20T12:23:53Z
run: 1

# Jobs
- T0 baseline: done — holdout 78.2 % (LEGS_BROKEN 20.5 %), dev 78.2 %, wind 0/2/4/5/6/7: 74.7/79.7/86.3/86.0/75.3/31.7 %; apogee max 16.0 m; no aborts at default wind
- task 1 chute wiggle: done — all suites green, head chef YES; T1_dev identical to T0_dev 600/600
- task 2 fit_cda.py: done — head chef YES; single flight ±25–45 %, pooled 15 flights +1.6 % ±9 %
- task 4 burn-table margin: done — 0.75 wins holdout (78.2→86.2 %, p=5e-15) + fresh (76.7→85.2 %, p=9e-17) but loses in steady 5–7 m/s (−11…−17 pts); 0.8/0.875 same trade; conditional patch
- task 5 missed-window abort: done — window-passed D=0.5: 0 landings lost (holdout, fresh, w5, w6, w6 fresh, w7); release 13.6 m vs 3.7 m (MW) and vs 5.7 m (TILT aborts at w7); patch port-checked 600/600 → recommend
- task 6 late ignition: done — success only for true delay 0.60–0.65 s; ≥0.80 s → dud declared (window 200), longer window → late light with chute held, 8–13 m/s impacts; guard cuts sideways impact 3.0→0.8 m/s, no success change
- task 7 shelved fixes: doing — D2, DRIFT_GAIN, DRIFT_MAX_TILT: no gain; WMA: w7 33→83 %, w8 6→74 %, but holdout p=0.42, fresh p=0.06, w4 −8 pts (LEGS_BROKEN) → gate fails; testing WMA + margin 0.75 combo
- task 8 TVC gains: done — boost gains no effect; landing kd×0.5 (F1) and kp×0.75/kd×0.35 (F2) win holdout+fresh (+5 pts) but lose at wind 5 (−6/−11) and with a slower servo (−22/−12): no change
