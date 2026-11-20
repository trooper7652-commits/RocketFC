status: RUNNING
heartbeat: 2026-11-20T11:59:48Z
run: 1

# Jobs
- T0 baseline: done — holdout 78.2 % (LEGS_BROKEN 20.5 %), dev 78.2 %, wind 0/2/4/5/6/7: 74.7/79.7/86.3/86.0/75.3/31.7 %; apogee max 16.0 m; no aborts at default wind
- task 1 chute wiggle: done — all suites green, head chef YES; T1_dev neutrality: see results
- task 2 fit_cda.py: done — head chef YES; single flight ±25–45 %, pooled 15 flights +1.6 % ±9 %
- task 4 burn-table margin: done — 0.75 wins holdout (78.2→86.2 %, p=5e-15) + fresh (76.7→85.2 %, p=9e-17) but loses in steady 5–7 m/s (−11…−17 pts); 0.8/0.875 same trade; conditional patch
- task 5 missed-window abort: doing — MW alt 5/6/8 and window-passed D 0.5/1/2: zero landing change (holdout, w5, w6); WP D=0.5 releases at 13.7 m vs 3.7 m (w6); fresh + w7 + patch port check running
- task 6 late ignition: done — success only for true delay 0.60–0.65 s; ≥0.80 s → dud declared (window 200), longer window → late light with chute held, 8–13 m/s impacts; guard cuts sideways impact 3.0→0.8 m/s, no success change
- task 7 shelved fixes on E18W: todo
- task 8 TVC gains: done — boost gains no effect; landing kd×0.5 (F1) and kp×0.75/kd×0.35 (F2) win holdout+fresh (+5 pts) but lose at wind 5 (−6/−11) and with a slower servo (−22/−12): no change
