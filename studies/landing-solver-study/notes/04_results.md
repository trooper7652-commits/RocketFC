# 04 — Results (holdout, clean sweep, robustness)

### Track 1: lag fix + risk-aware aim (B+C)

| | powered | median | mean | p90 | soft <2.5 | <1 | hops (severe) | aborts |
|---|---|---|---|---|---|---|---|---|
| baseline | 778/2000 | 6.95 | 9.61 | 28.00 | 31 | 5 | 154 (138) | MISSED_WINDOW 67, TILT 1155 |
| candidate | 775/2000 | 7.32 | 9.90 | 28.04 | 33 | 2 | 115 (109) | MISSED_WINDOW 70, TILT 1155 |

Paired over 775 flights that landed under power in both: Δmedian **+0.37** [95 % CI +0.23, +0.54], Δp90 **+0.00** [+0.00, +0.54], Δmean +0.28 [+0.19, +0.38]; better/worse/same 150/408/217; powered only in baseline 3, only in candidate 0.

| criterion | result |
|---|---|
| 1 median -0.5 & CI<0 | FAIL |
| 2 p90 lower & CI<0 | FAIL |
| 3 hops <= baseline | PASS |
| 4 aborts <= baseline (each reason) | FAIL |
| 5 clean sweep better, no pt >0.3 worse | FAIL |
| 6 robustness gate | FAIL |

Clean apogee sweep: mean touchdown baseline 4.77 vs candidate 6.28 m/s, worst point change +3.64 m/s.

**Verdict: FAIL**

Robustness (seeds 101–103):
```
mass_x0.9        BAD table med 10.43 p90 28.02 hops   0 [MISS23,TILT350] | cand med 11.83 p90 28.02 hops   0 [MISS23,TILT350] | dMed +1.40 [+1.23,+1.52] dP90 +0.00 [+0.00,+0.00]
mass_x1.1        ok  table med 11.87 p90 27.96 hops 166 [MISS18,TILT350] | cand med 11.21 p90 27.96 hops 158 [MISS18,TILT350] | dMed -0.66 [-0.72,-0.41] dP90 +0.00 [+0.00,+0.00]
cda_x0.7         BAD table med  6.89 p90 27.98 hops  66 [MISS20,TILT350] | cand med  7.24 p90 27.98 hops  60 [MISS20,TILT350] | dMed +0.35 [-0.01,+0.54] dP90 +0.00 [+0.00,+0.00]
cda_x1.3         BAD table med  7.17 p90 27.98 hops  28 [MISS20,TILT350] | cand med  7.44 p90 27.98 hops  20 [MISS20,TILT350] | dMed +0.27 [+0.06,+0.53] dP90 +0.00 [+0.00,+0.00]
delay_425        BAD table med  8.23 p90 27.99 hops   3 [MISS21,TILT350] | cand med  8.70 p90 28.00 hops   3 [MISS22,TILT350] | dMed +0.48 [+0.30,+0.64] dP90 +0.00 [+0.00,+0.00]
delay_575        ok  table med  8.24 p90 27.98 hops 115 [MISS20,TILT350] | cand med  7.49 p90 27.98 hops  93 [MISS20,TILT350] | dMed -0.75 [-1.06,-0.22] dP90 +0.00 [+0.00,+0.00]
thrust_x0.9      BAD table med 10.40 p90 28.18 hops   0 [MISS20,TILT350] | cand med 10.74 p90 28.18 hops   0 [MISS20,TILT350] | dMed +0.34 [+0.23,+0.49] dP90 +0.00 [+0.00,+0.00]
thrust_x1.1      ok  table med 14.32 p90 27.94 hops 167 [MISS20,TILT350] | cand med 13.69 p90 27.94 hops 151 [MISS20,TILT350] | dMed -0.63 [-0.81,-0.27] dP90 +0.00 [-0.19,+0.00]
curve_coker      BAD table med  6.66 p90 27.98 hops  48 [MISS20,TILT350] | cand med  6.89 p90 27.98 hops  31 [MISS20,TILT350] | dMed +0.23 [-0.05,+0.55] dP90 +0.00 [+0.00,+0.00]
curve_smart      BAD table med  6.54 p90 27.98 hops  57 [MISS20,TILT350] | cand med  6.66 p90 27.98 hops  43 [MISS20,TILT350] | dMed +0.12 [-0.12,+0.58] dP90 +0.00 [+0.00,+0.00]
baro_lag_0       BAD table med  6.82 p90 27.98 hops  65 [MISS20,TILT350] | cand med  7.19 p90 27.98 hops  48 [MISS20,TILT350] | dMed +0.37 [-0.19,+0.67] dP90 +0.00 [+0.00,+0.00]
baro_lag_10      BAD table med  6.82 p90 27.98 hops  65 [MISS20,TILT350] | cand med  7.19 p90 27.98 hops  48 [MISS20,TILT350] | dMed +0.37 [-0.19,+0.67] dP90 +0.00 [+0.00,+0.00]
baro_lag_50      BAD table med  7.38 p90 27.98 hops  22 [MISS20,TILT350] | cand med  7.80 p90 27.99 hops  14 [MISS21,TILT350] | dMed +0.42 [+0.12,+0.67] dP90 +0.00 [+0.00,+0.00]
baro_lag_75      BAD table med  8.04 p90 28.11 hops   7 [MISS20,TILT350] | cand med  8.64 p90 28.11 hops   3 [MISS20,TILT350] | dMed +0.60 [+0.35,+0.81] dP90 +0.00 [+0.00,+2.36]
baro_q_+0.1      BAD table med  6.05 p90  7.71 hops  18 [KF_U152,MISS12,TILT350] | cand med  6.26 p90  8.63 hops  10 [KF_U154,MISS12,TILT350] | dMed +0.18 [-0.42,+0.88] dP90 +0.91 [-0.12,+1.00]
baro_q_-0.1      BAD table med  7.83 p90 24.77 hops   2 [KF_U67,MISS13,TILT350] | cand med  8.34 p90 24.84 hops   0 [KF_U67,MISS14,TILT350] | dMed +0.51 [+0.38,+1.10] dP90 +0.00 [+0.00,+0.00]
accel_bias_20mg  BAD table med  6.51 p90 27.59 hops  65 [MISS13,TILT347] | cand med  6.75 p90 27.59 hops  47 [MISS13,TILT347] | dMed +0.24 [-0.08,+0.56] dP90 +0.00 [-0.37,+0.00]
GATE: FAIL
```

### Track 2: B+C + predicted-tilt interlock (D2)

| | powered | median | mean | p90 | soft <2.5 | <1 | hops (severe) | aborts |
|---|---|---|---|---|---|---|---|---|
| baseline | 817/2000 | 7.08 | 9.60 | 27.39 | 35 | 6 | 173 (164) | MISSED_WINDOW 77, TILT 1106 |
| candidate | 881/2000 | 7.37 | 10.32 | 27.44 | 34 | 4 | 183 (173) | MISSED_WINDOW 13, TILT 1106 |

Paired over 816 flights that landed under power in both: Δmedian **-0.01** [95 % CI -0.22, +0.25], Δp90 **-8.29** [-9.17, -5.82], Δmean -0.74 [-0.95, -0.54]; better/worse/same 353/457/6; powered only in baseline 1, only in candidate 65.

| criterion | result |
|---|---|
| 1 median -0.5 & CI<0 | FAIL |
| 2 p90 lower & CI<0 | PASS |
| 3 hops <= baseline | FAIL |
| 4 aborts <= baseline (each reason) | PASS |
| 5 clean sweep better, no pt >0.3 worse | FAIL |
| 6 robustness gate | FAIL |

Clean apogee sweep: mean touchdown baseline 4.77 vs candidate 5.04 m/s, worst point change +3.64 m/s.

**Verdict: FAIL**

Robustness (seeds 101–103):
```
mass_x0.9        BAD table med 10.43 p90 28.02 hops   0 [MISS23,TILT350] | cand med 11.92 p90 27.75 hops   0 [MISS3,TILT350] | dMed +1.26 [+1.03,+1.45] dP90 -8.42 [-9.11,-3.36]
mass_x1.1        BAD table med 11.87 p90 27.96 hops 166 [MISS18,TILT350] | cand med 12.13 p90 26.37 hops 179 [MISS2,TILT350] | dMed -0.26 [-0.74,+0.45] dP90 -8.73 [-9.22,-1.51]
cda_x0.7         BAD table med  6.89 p90 27.98 hops  66 [MISS20,TILT350] | cand med  7.41 p90 27.19 hops  76 [MISS2,TILT350] | dMed +0.35 [-0.22,+0.59] dP90 -8.69 [-9.22,-3.54]
cda_x1.3         BAD table med  7.17 p90 27.98 hops  28 [MISS20,TILT350] | cand med  7.49 p90 27.19 hops  31 [MISS2,TILT350] | dMed +0.07 [-0.25,+0.48] dP90 -8.69 [-9.22,-3.62]
delay_425        BAD table med  8.23 p90 27.99 hops   3 [MISS21,TILT350] | cand med  8.58 p90 27.02 hops   7 [MISS5,TILT350] | dMed +0.15 [-0.19,+0.42] dP90 -7.41 [-8.16,-2.14]
delay_575        ok  table med  8.24 p90 27.98 hops 115 [MISS20,TILT350] | cand med  8.31 p90 26.98 hops 117 [MISS2,TILT350] | dMed -0.59 [-1.24,+0.25] dP90 -9.51 [-10.02,-3.80]
thrust_x0.9      ok  table med 10.40 p90 28.18 hops   0 [MISS20,TILT350] | cand med 10.61 p90 27.20 hops   0 [MISS2,TILT350] | dMed -0.01 [-0.30,+0.24] dP90 -7.64 [-7.93,-3.25]
thrust_x1.1      ok  table med 14.32 p90 27.94 hops 167 [MISS20,TILT350] | cand med 14.67 p90 27.15 hops 164 [MISS2,TILT350] | dMed -0.06 [-0.60,+0.49] dP90 -9.08 [-9.62,-0.63]
curve_coker      ok  table med  6.66 p90 27.98 hops  48 [MISS20,TILT350] | cand med  7.06 p90 27.19 hops  46 [MISS2,TILT350] | dMed +0.08 [-0.34,+0.50] dP90 -8.69 [-9.22,-3.62]
curve_smart      ok  table med  6.54 p90 27.98 hops  57 [MISS20,TILT350] | cand med  6.94 p90 27.23 hops  57 [MISS2,TILT350] | dMed -0.04 [-0.43,+0.52] dP90 -8.85 [-9.39,-3.76]
baro_lag_0       BAD table med  6.82 p90 27.98 hops  65 [MISS20,TILT350] | cand med  7.34 p90 27.19 hops  62 [MISS2,TILT350] | dMed +0.14 [-0.40,+0.63] dP90 -8.69 [-9.22,-3.56]
baro_lag_10      BAD table med  6.82 p90 27.98 hops  65 [MISS20,TILT350] | cand med  7.34 p90 27.19 hops  62 [MISS2,TILT350] | dMed +0.14 [-0.40,+0.63] dP90 -8.69 [-9.22,-3.56]
baro_lag_50      ok  table med  7.38 p90 27.98 hops  22 [MISS20,TILT350] | cand med  7.71 p90 27.26 hops  24 [MISS3,TILT350] | dMed +0.10 [-0.23,+0.52] dP90 -8.69 [-9.20,-3.63]
baro_lag_75      BAD table med  8.04 p90 28.11 hops   7 [MISS20,TILT350] | cand med  8.45 p90 27.75 hops   9 [MISS3,TILT350] | dMed +0.27 [-0.15,+0.55] dP90 -8.04 [-9.10,-2.72]
baro_q_+0.1      BAD table med  6.05 p90  7.71 hops  18 [KF_U152,MISS12,TILT350] | cand med  6.61 p90 26.78 hops  10 [KF_U154,MISS4,TILT350] | dMed +0.18 [-0.42,+0.88] dP90 +0.91 [-0.40,+1.00]
baro_q_-0.1      BAD table med  7.83 p90 24.77 hops   2 [KF_U67,MISS13,TILT350] | cand med  8.35 p90 26.21 hops   3 [KF_U67,MISS3,TILT350] | dMed +0.26 [+0.10,+0.95] dP90 -6.24 [-8.43,-0.69]
accel_bias_20mg  BAD table med  6.51 p90 27.59 hops  65 [MISS13,TILT347] | cand med  6.85 p90 26.25 hops  51 [MISS4,TILT347] | dMed +0.17 [-0.18,+0.52] dP90 -7.81 [-8.34,+0.11]
GATE: FAIL
```
