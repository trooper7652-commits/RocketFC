# RocketFC landing-burn solver study — REPORT

*2026-10-12. Everything was built and tested in scratch copies; **your RocketFC folder was not
changed** (verified against the SHA-256 hashes taken at the start).*

## The answer in five lines
1. **No new solver passed the test we fixed in advance**, so nothing was ported. The current table
   stays.
2. **Why:** in the simulator, a typical landing (~6–7 m/s) is limited by things *no timing rule can
   know before ignition* — that flight's real ignition delay, mass and motor thrust. The current
   table, helped by an accidental safety margin from the Kalman filter's lag, already fires within a
   few tens of milliseconds of the best possible moment for those flights.
3. **The one rule change with a large, real effect** is a smarter tilt interlock ("D2": allow the
   fire if the tilt will be under 15° when thrust actually starts). It fixes the high-apogee case
   (clean flights at 60–73 m apogee: 8–9 m/s → 3.6–4 m/s) and the worst Monte-Carlo landings
   (p90 −7 to −8.5 m/s), but it relaxes your safety rule, adds hops, didn't improve the median, and
   flights it "rescues" from a missed window still crash (slightly harder than before).
4. **Measuring beats any solver change:** knowing the ignition delay to ±5 % and the descent mass to
   ±1 % would cut the best achievable typical touchdown from ~6.4 to ~3.7 m/s.
5. **The only flown F15 landing (BPS.space) changed thrust after ignition** (and was still "more slam
   than hover") — that's outside the solver and needs your go-ahead.

## Words used (plain language)
- **Monte Carlo**: fly the simulator hundreds of times, each time with slightly different random
  "unknowns" (mass, wind, ignition delay…), and look at the spread of results.
- **Median**: the middle landing speed (half land softer, half harder). **p90**: 90 % of landings
  are at or below this speed — it measures the bad tail.
- **Hop**: the motor stops the rocket in mid-air, pushes it back up, burns out, and it falls.
- **Paired comparison**: both solvers fly the *identical* 200 random flights per seed, so we compare
  flight-for-flight. **95 % CI (confidence interval)**: the range the true difference very likely
  lies in; if it doesn't include 0, the difference is real, not luck.
- **Holdout seeds**: fresh random flights, never looked at while designing, used once for the
  verdict — so the result can't be tuned to the test.
- **Predictor / Smith predictor**: estimate where you'll be after a known delay and act on that.
- **Lag**: the barometer (through its vent holes and filtering) reports where the rocket *was*, so the
  Kalman filter reads a bit high and slow while falling.

## What I did
1. **Phase 0 – understand & reproduce** (`00_current_system.md`): read the firmware and sim,
   reproduced your 2026-10-12 numbers *exactly* (identical summaries), measured the Kalman lag
   (effective ~17–20 ms: reads +0.27 m high and 0.18 m/s slow at the fire point).
2. **Headroom analysis**: for 248 flights I re-flew the identical flight with the fire command forced
   at ~100 different times (25 881 simulated flights) to see the best *any* timing rule could do.
3. **Phase 1 – research** (`01_research.md`, 18 sources, each marked flown / simulated / proposed).
4. **Phase 2 – candidates** (`02_candidates.md`): B lag compensation, C risk-aware aim point,
   D1 tilt-loss compensation, D2 predicted-tilt interlock; A (onboard predictor) analysed; E
   (impulse dumping) flagged as outside the solver.
5. **Phase 3 – rules first** (`03_metrics.md`, SHA-256 `2d5f3a8c…` written *before* any comparison).
6. **Phase 4 – build** (`src/core/landing_solver.h`, `tools/landing_table.py`, tests) in the copy.
7. **Phase 5 – evaluate**: dev ablation (17 configs × 600 flights), one-shot holdouts (2 × 4000
   flights), clean apogee sweeps, robustness matrix, plus clearly-labelled extra (post-hoc) checks.

## What the research found (details and links in `01_research.md`)
| Source | Status | Lesson for RocketFC |
|---|---|---|
| BPS.space Scout F (same Estes F15) | **Flown** (1st success Jul 2022) | Pure ignition timing was *abandoned* ("ignition time and exact thrust numbers simply weren't repeatable enough"); success came from changing thrust after ignition (TVC side-to-side in 2020, then ceramic exhaust paddles) and was still "more slam than hover". |
| Aryan Kapoor / JRD (2024) | Flown, partial | Altitude-triggered descent motor; no code/numbers found. |
| Mars Pathfinder solid retro-rockets | **Flown** | Couldn't throttle, so they aimed to stop **2–22 m above** the ground and dropped onto airbags — put the error on the side your hardware tolerates. |
| Soyuz soft-landing solids | **Flown** | Tiny burn fired ~1 m up by a proximity sensor — not our 3.45 s burn from 30–70 m. |
| Apollo LM, Falcon 9, G-FOLD, MechJeb, kOS | Flown / game | All keep **throttle margin** to absorb errors during the burn; a solid motor has none. Falcon 9 also can't hover ("zero velocity at exactly zero altitude"). |
| Smith predictor, baro-inertial fusion | Practice | Predict through known delays → candidate B. |
| Newsvendor / "critical fractile" | Theory | With a lopsided cost, aim at a quantile of the error, not the middle → candidate C. |
| Estes F15 data (ThrustCurve, certification rules) | Data | Certification allows impulse SD up to 6.7 %; an F15 figure of 1.7 % is **unverified** (NAR PDF blocked); two official curves differ in shape. Ignition-delay spread: **no data found** — must be measured. |

## Headroom: how good could ANY timing rule be? (dev flights, 228 powered landings)
| | median | mean | p90 |
|---|---|---|---|
| Current table (what happened) | 7.10 | 9.96 | 28.2 |
| Current table fed the **true** state (perfect lag fix) | 6.60 | 6.65 | 10.4 |
| Current table with the tilt interlock ignored | 6.28 | 6.42 | 9.9 |
| **Perfect fire time for each flight** (knows that flight's delay, mass, thrust) | **0.74** | 1.35 | 1.0 |

- The window of fire times that gives a soft landing is only **~10 ms** wide; the best fire time
  moves **±100 ms** from flight to flight with the unknowns. No rule can hit a 10 ms window it can't
  see. (Picture: `plots/headroom_examples.png`.)
- For the 73 % of landings that aren't tilt-blocked, the current table is already at the best
  constant-margin optimum; removing the lag *alone* makes them worse (it's an accidental margin).
- Tilt-blocked flights (27 % of landings, median **22.9 m/s**) need to fire 0.4 s to > 1.2 s
  earlier, i.e. at 15–30° of tilt.

### Error budget (how far the correct fire altitude moves, 1-D physics)
Ignition delay +75 ms → **+2.6 to +3.3 m**; descent mass +5 % → +2.3 m (and at 15 m/s the motor
can no longer stop the rocket at all); thrust +6.7 % → −2.2 to −5.0 m; CdA +20 % → −0.2 to −1.0 m;
Kalman lag → +0.5 m. The nominal flight fires at ~15 m/s — **right at the F15's energy limit** for
0.95 kg.

### Value of information (best possible timing-only result at 15 m/s; `plots/value_of_information.png`)
| What's known | median | p90 |
|---|---|---|
| Sim today (delay ±15 %, mass ±5 %, CdA ±20 %) | 6.4 | 9.6 |
| Delay measured to ±5 % | 6.2 | 8.4 |
| Mass weighed to ±1 % | 5.6 | 7.5 |
| **Both** | **3.7** | 5.5 |
| Delay ±10 ms, mass ±1 %, CdA ±10 % | 3.1 | 5.0 |
| …plus a realistic 1.7 % thrust spread | 4.4 | 7.3 |

## Candidates tried (dev seeds 7, 11, 13; 600 flights; paired vs the current table)
B = lag compensation, C = risk-aware aim, D1 = tilt-loss compensation, D2 = predicted-tilt
interlock, hpN = hop penalty N m/s in C, capN = D2 hard cap N°.

| config | median | p90 | hops | missed windows | paired Δmedian [95% CI] | paired Δp90 [95% CI] |
|---|---|---|---|---|---|---|
| baseline (current table) | 7.10 | 28.2 | 58 | 20 | — | — |
| B | 6.96 | 28.2 | 72 | 19 | -0.10 [-0.31, +0.34] | +0.00 [+0.00, +0.00] |
| C | 7.43 | 28.2 | 33 | 20 | +0.33 [-0.08, +0.88] | +0.00 [+0.00, +0.00] |
| B+C | 7.20 | 28.2 | 45 | 20 | +0.10 [-0.09, +0.58] | +0.00 [+0.00, +0.00] |
| B+C+D1 | 7.20 | 28.2 | 45 | 20 | +0.10 [-0.09, +0.58] | +0.00 [+0.00, +0.00] |
| D2 | 7.18 | 27.5 | 64 | 3 | -0.01 [-0.48, +0.18] | -8.54 [-9.36, -1.89] |
| B+D2 | 7.12 | 27.4 | 82 | 3 | -0.14 [-0.40, +0.35] | -8.54 [-9.36, -1.89] |
| C+D2 | 7.71 | 27.5 | 38 | 3 | +0.27 [-0.14, +0.82] | -8.54 [-9.36, -1.89] |
| B+C+D2 | 7.44 | 27.5 | 50 | 3 | +0.08 [-0.26, +0.53] | -8.54 [-9.36, -1.89] |
| B+C+D1+D2 | 7.44 | 27.5 | 50 | 3 | +0.08 [-0.25, +0.54] | -8.54 [-9.36, -1.89] |
| B+C_hp2 | 7.33 | 28.2 | 30 | 20 | +0.24 [-0.19, +0.76] | +0.00 [+0.00, +0.00] |
| B+C_hp5 | 7.81 | 28.2 | 16 | 20 | +0.72 [+0.08, +1.26] | +0.00 [+0.00, +0.00] |
| B+C_hp10 | 8.30 | 28.2 | 7 | 20 | +1.21 [+0.61, +1.84] | +0.00 [+0.00, +0.00] |
| B+C+D2_hp2 | 7.51 | 27.5 | 35 | 3 | +0.24 [-0.23, +0.73] | -8.54 [-9.36, -1.89] |
| B+C+D2_hp5 | 7.91 | 27.5 | 21 | 3 | +0.58 [-0.09, +1.17] | -8.54 [-9.36, -1.89] |
| B+C+D2_hp10 | 8.43 | 27.5 | 8 | 3 | +0.97 [+0.43, +1.65] | -8.54 [-9.36, -1.89] |
| B+C+D2_cap20 | 7.42 | 27.5 | 50 | 4 | +0.08 [-0.26, +0.53] | -7.21 [-8.69, -1.77] |
| B+C+D2_cap30 | 7.44 | 27.5 | 50 | 3 | +0.08 [-0.26, +0.53] | -8.54 [-9.36, -1.89] |

- **D1 changed nothing** (≤ 3 cm of altitude) → dropped by the pre-registered ablation rule.
- Every aim variant moves along the same **median ↔ hops trade-off**; the current table already sits
  on it. Nothing improves the median by 0.5 m/s without adding hops.

## Verdict on the holdout seeds (run once; full tables in `04_results.md`)
| Holdout (2 000 flights each, run once) | Δmedian [95 % CI] | Δp90 [95 % CI] | hops | missed windows | clean sweep | verdict |
|---|---|---|---|---|---|---|
| **Track 1** — B+C (seeds 101–110) | **+0.37** [+0.23, +0.54] (worse) | 0.00 [0.00, +0.54] | 154 → 115 ✅ | 67 → 70 ❌ | worse at low apogee ❌ | **FAIL** |
| **Track 2** — B+C+D2 (seeds 201–210) | −0.01 [−0.22, +0.25] | **−8.29** [−9.17, −5.82] ✅ | 173 → 183 ❌ | 77 → 13 ✅ | worse at low apogee ❌ | **FAIL** |

Both fail the pre-registered bar (criterion 1, a 0.5 m/s better median, plus others), so per your
rule **nothing was ported**. Plots: `plots/holdout_cdf.png`, `plots/holdout_paired_scatter.png`,
`plots/clean_apogee_sweep.png`.

A hidden cost of D2, found while checking these numbers: 65 flights that the current table
"missed" (it never fired; the chute opens at 4 m, too late — median 24.4 m/s) are fired by D2
tilted (~21°) and too late, and hit *harder* (28.6 m/s). They don't appear in the paired statistics
because they are only powered landings in one of the two runs.

## Robustness (does it survive a wrong model?)
Each condition flies 600 paired flights (seeds 101–103) with the plant "true" and the flight
software wrong, or the reverse; the current table is given the same wrong belief (its table
regenerated by the repo's own `burn_table.py`). Pass = never worse than the table (median ≤ +0.10,
p90 ≤ +0.25 m/s, hops and aborts ≤ table + 2).

**Track 2 (B+C+D2): FAIL — 11 of 17 conditions worse than the table** (full lines in
`04_results.md` / `data/robustness_track2.txt`). Pattern: p90 better almost everywhere (the D2
effect), but the median a little worse (+0.1 to +1.3 m/s) and more hops when the solver's mass,
drag or delay belief is wrong — i.e. the new aim is *more* sensitive to a wrong model than the
current table.

**Track 1 (B+C): FAIL — 14 of 17 conditions.** Fewer hops almost everywhere, but a worse median
(+0.1 to +1.4 m/s) unless the true rocket happens to need a later fire than believed (it wins only
when the delay is really longer, the motor stronger, or the rocket lighter than assumed).

**D2 only (post-hoc): FAIL — 13 of 17 conditions**, almost all on the **hop count** (+7 to +23 hops
per 600 flights), while its paired median is the same or slightly *better* in most conditions
(−0.06 to −0.49 m/s) and its p90 is ~7–9.5 m/s better in all but one. It is the same trade seen
everywhere: D2 turns "fired far too late → 20–28 m/s crash" into "fired on time → normal outcome",
and a normal on-time fire in this simulator hops about a quarter of the time (usually falling back
at 4–10 m/s). The pre-registered rule counts every hop as a failure regardless of speed, so a
5 m/s hop counts against D2 while a 25 m/s crash it prevents doesn't count in its favour. Whether
that trade is worth relaxing your 15° rule is your call, not the gate's.

**What the robustness runs say about the rocket (true for either solver):**
| condition | current table median | note |
|---|---|---|
| nominal | 7.1 | |
| solver believes mass −10 % / +10 % | 10.4 / 11.9 | weigh the rocket |
| believes delay 425 / 575 ms (true 500) | 8.2 / 8.2 | measure the delay |
| motor thrust −10 % / +10 % | 10.4 / **14.3** | measure the thrust curve |
| official certification curves as the true motor | 6.5–6.7 | your `.eng` is close |
| barometer lag 75 ms (sim default ≈ 20 ms) | 8.0 | measure the lag/venting |
| baro reads off by 10 % of dynamic pressure | 6.1 / 7.8, but **67–152 of 600 flights abort KF_UNHEALTHY** | static-port error trips the Kalman health check during the fall |

Sim limitation found: baro lags ≤ 10 ms are indistinguishable from zero (the lag is applied once per
10 ms baro sample), and the default "25 ms" behaves like ≈ 20 ms — consistent with the measured
17–20 ms.

## Extra checks after the verdict (post-hoc — they cannot change it)
These used fresh seeds 301–310 and are labelled *post-hoc*: they inform your decisions but
can't change the verdict.

**1. D2 alone** (today's table and aim; only the predicted-tilt interlock added):
| | median | p90 | hops | missed windows |
|---|---|---|---|---|
| current table | 6.84 | 27.4 | 179 | 50 |
| + D2 only | 6.86 (paired Δ −0.14 [−0.27, −0.00]) | 26.3 (paired Δ **−7.0** [−9.2, −4.8]) | **226** | **13** |
| every flight that reached the landing decision (incl. missed windows) | 7.02 → **6.90** | 27.2 → **26.0** | | |
Clean flights: identical up to 52 m apogee; **56–73 m apogee: 5.2–9.4 → 3.6–4.1 m/s**. (Picture of every variant
across apogee: `plots/clean_apogee_sweep_all.png`.)
Hops rise because flights that used to fire late (always hard, never a hop) now fire on time, and
on-time fires in this sim hop ~25 % of the time.

**2. "After you measure"** — mass known to ±1 %, ignition delay to ±5 % (sim dispersions narrowed;
the risk table regenerated for that spread):
| | median | p90 | hops | soft (< 2.5) | missed windows |
|---|---|---|---|---|---|
| current table, today's unknowns (same seeds) | 6.84 | 27.4 | 179 | 37 | 50 |
| **current table, after measuring** | **4.50** | 28.4 | **43** | **87** | 59 |
| B+C (table for the measured spread) | 4.71 (Δ +0.22 [+0.13, +0.24]) | 28.4 | 30 | 59 | 59 |
| D2 only | 4.55 (Δ −0.06 [−0.15, −0.02]) | 27.8 (Δ −6.1 [−8.9, −3.4]) | 43 | 89 | **8** |
**Measuring mass and delay improves the *unchanged* solver by 2.3 m/s in the median and cuts hops
by three-quarters — more than any solver change could.** Even then the new aim (C) stays slightly
worse than the current table + Kalman lag, which turns out to be a well-tuned margin.

## Why nothing won
- The pre-registered bar was a **0.5 m/s better median** *and* no more hops. The headroom analysis
  shows the median is set by per-flight unknowns that only become visible after ignition; the table
  plus the Kalman lag already sit at the timing optimum for typical flights.
- The candidate that targets the real failure mode (tilt) fixes the tail, not the middle.
- The risk-aware aim (C) is only as good as its assumed uncertainty: with the sim's wide
  dispersions it aims 1–3 m late, which reduces hops but makes clean flights land at ~5.5 m/s
  instead of 2–3.5 m/s.

## Remaining risks (true for the current table too)
- **The simulator isn't the rocket.** Descent tilt comes from an *estimated* aerodynamic model
  (CNα, CP); real weathervaning may be very different. Baro lag (25 ms) and bay venting are
  assumptions. Static-pressure error from airflow is not modelled and could be **metres** at 15–20 m/s.
- The sim's thrust curve is shared with the solver (fixed for the robustness tests by flying the two
  official certification curves as "truth").
- **62 % of simulated flights abort on tilt before or during descent** (38 % in COAST: the fin-less
  airframe is unstable nose-first with TVC off; 21 % in DESCENT). That dominates mission success
  far more than the landing solver.
- Latent bug found: `burntable::hIgnite(NaN)` performs `(int)NaN` (undefined behaviour) before the
  interlocks reject the fire. Harmless in practice so far; a one-line `isfinite` guard fixes it.
- The `config.h` comment says the FSM runs at 100 Hz; it actually runs in the 500 Hz loop.

## Recommendations
1. **Keep the current solver.** Optional tiny fixes outside this study's scope: an `isfinite` guard
   in `burntable::hIgnite`, and the stale "FSM at 100 Hz" comment in `config.h`.
2. **Measure before tuning anything** (list below) — then regenerate the table. In the sim, that alone
   is worth ~2 m/s of median touchdown speed and three-quarters fewer hops.
3. **Fly CHUTE_TEST first** and look at descent tilt and baro-vs-accelerometer altitude in the log.
   If the real rocket weathervanes past 15° near the ignition window, revisit D2 — ideally with a
   guard that keeps today's behaviour once it's already too late to help (untested idea).
4. **Mission-level levers matter more than the solver:** the fin-less airframe's tilt aborts
   (62 % of sim flights), and the energy margin — firing at ~15 m/s descent is at the F15's limit
   for 0.95 kg; a lower apogee (≈ 30–40 m, firing at ~11–13 m/s) leaves energy margin for a
   heavy/weak case (physics argument from the error budget — not tested in the Monte Carlo).
5. **For genuinely soft landings**, some post-ignition thrust control is needed (candidate E) —
   your call.

## What to measure on the real hardware (most valuable first)
1. **Ignition delay and its spread** — ThrustStand, the *exact* e-match + F15 + pyro-battery setup,
   10+ firings: time from fire command to first thrust. (Biggest single unknown: +75 ms = +3 m.)
2. **Descent mass** — kitchen scale, ±1 % (≈ ±10 g), with the F15 loaded and the ascent motor spent.
3. **F15 thrust curves** — ThrustStand runs → `.eng`; replace the "approximate" file; note the spread.
4. **Barometer lag and bay venting** — compare baro altitude to accelerometer-integrated altitude in
   a CHUTE_TEST descent log (sets `LAG_TAU_S`), and check for a speed-dependent offset (static error).
5. **Descent tilt** — CHUTE_TEST logs: how tilted is the rocket in freefall and is it falling or
   rising? This decides whether D2 is worth having.
6. **CdA** — drop test or log velocity decay; TVC tilt-recovery time after ignition.

## Where the work is
This folder (`SpaceX Rocket\landing-solver-study\`): `notes\` (this report, 00–04, plots),
`candidate\` (the tested solver as a patch with safe TABLE defaults, plus its new files — tests all
pass, Teensy build clean, ~5 ns per call on a PC), `scripts\` (driver, metrics, forced-fire scan,
analysis), `reference_curves\` (official F15 curves) and `data.zip` (every run's JSON).
The simulator analysis tooling (hop detection, abort phase, KF-error-at-fire and lag diagnostics in
`run.py`, off-by-default plant/sensor perturbation knobs, and a Monte Carlo batch driver) was ported
into `RocketFC\tools\sim` at your request — see the uncommitted diff there.
