# Overnight run 2026-11-20 (cloud routine): plan

Owner-approved tasks 1, 2, 4, 5, 6, 7 and 8, plus a T0 baseline. The run starts 2:30 AM CST (08:30 UTC). Hourly check-ins resume it until it is finished; the last one is at 14:30 CST (20:30 UTC).

**Read first**
- `README.md` (§9)
- `studies/README.md`
- `studies/e18w-port-check-2026-11-20/README.md`
- `studies/latch-dud-study-2026-11-07/README.md`
- `studies/motor-chute-study-2026-11-04/README.md`
- `studies/software-landing-study/` (`report.md`, `new-ideas-2026-10-19.md`, `patches/`)

## Ground rules
1. **Prove before changing.** No flight-behaviour change goes into `src/`. Every study variant is built in a scratch copy outside the repo (e.g. `/tmp/ws/<name>`) behind an off-by-default switch. It is compared, paired, against the unchanged code on the same seeds. Recommend a port only if it is measurably better on BOTH the holdout and fresh seeds (a lesson from 2026-10-25: a margin that won on dev lost on fresh seeds). Winning ports go in `studies/overnight-2026-11-20/patches/` as unapplied patches.
   - Exceptions, which go into the repo: task 1 (a bench-only CLI command; its refactor must be result-neutral) and task 2 (a new tool).
2. **Mass.** It is already in `main`: `MASS_PAD_KG 1.4770`, `MASS_DESCENT_KG 1.4563`. That is the owner's measured dry mass of 1.343 kg + 2 × (57.0 g E18W + 10 g adapter). The burn table is regenerated. Use these values; never change them.
3. **Apogee limit 100 ft (30.48 m).** Report the apogee (median / p99 / max) for every setup, and flag anything over the limit.
4. **Owner rules.**
   - On any abort the chute must open as soon as possible. Surviving the fall and chute fill are NOT criteria.
   - Success = powered landing + upright + intact, with a 3 m/s leg crash limit (owner's).
   - No thrust-shedding hardware or altitude concepts.
5. **Git.**
   - Work on branch `overnight-2026-11-20` (create it from `origin/main` if it doesn't exist; otherwise check it out and pull).
   - Push only that branch. Never push to `main`, never merge, never force-push.
   - At the end, open ONE PR into `main` and leave it open.
   - End every commit message and the PR body with the attribution lines you were given.
6. **Results.**
   - Raw `mc_batch.py` JSON stays out of git (8 MB / 2,000 flights).
   - After each job, commit a compact per-flight CSV, gzipped: `studies/overnight-2026-11-20/results/<job>.csv.gz`. Columns: seed, i, success, code, abort_reason, apogee_h, contact_vspeed_mps, touchdown_t, fire_t, chute release / open times when present, and the job's own metric columns.
   - Also commit a one-line summary in `results/summary.md`.
   - Pair from the CSVs, by (seed, i), with a two-sided sign test on rescued vs lost.
7. **Keep the report short.** For each task: what was tested, the numbers, why, and the recommended fix (or "no change").

## Sim settings (unless a task says otherwise)

**Command**
- `cd tools/sim && python mc_batch.py --seeds ... --n ... --mode land --disp mass_descent_kg=0.01 ignition_delay_s=0.05 --plant legs_cut_s=0.75 --out /tmp/res/<job>.json`
- That means measured spreads, and the band cut at the owner's ≤ 0.75 s target.
- Firmware variants: `--fw-overrides '{"NAME": "1.0f"}'`. `GAINS_BOOST` / `GAINS_LANDING` take a dict through core.py's struct patcher. Code variants use a scratch copy.

**Seeds**

| Set | Seeds | Flights | Use |
|---|---|---|---|
| Screen | 101–103 × 100 | 300 | screening |
| Dev | 101–103 × 200 | 600 | |
| Holdout | 701–710 × 200 | 2,000 | finalists only |
| Fresh | 801–810 × 200 | 2,000 | only to confirm a holdout winner |

- **Wind:** the default is a uniform −4…+4 m/s draw. Wind sweeps use `--wind W` (the other draws stay paired).
- **Compute:** check `nproc`. The rate is ≈ 0.45 flights/s per worker: 2,000 flights take ~20 min on 4 CPUs. Run sim jobs in the background (nohup) and do the code tasks meanwhile.

## Execution: the kitchen (cloud = Claude only, no free cooks)

| Role | Who | Does |
|---|---|---|
| Manager | Opus 5.5, the session model | Runs the plan and the ledger; designs every study variant; judges every result; runs git/gh. |
| Prep cook | Haiku 5.5 (Agent, `model: haiku`) | Workspace setup, launching listed sim jobs, summarizing CSVs, running test suites; reports raw output. |
| Line cook | Sonnet 5.5 (Agent, `model: sonnet`) | Code for tasks 1 and 2 and the scratch-copy study patches (5–8), from the manager's exact blueprint. |
| Head chef | Opus 5.5 (Agent, `model: opus`, fresh context) | Final YES/NO on the task 1–2 diff and on REPORT.md. The work is done only on YES. Fixes flow back down. |

## Ledger and hourly check-ins

`studies/overnight-2026-11-20/LEDGER.md`, on the branch, is the single source of truth:
- top lines: `status: RUNNING | DONE`, `heartbeat: <UTC ISO>`, `run: <n>`;
- then one line per task/job: todo / doing / done, plus its result line.

Every run, in this order:
1. **Date guard.** If the UTC date is not 2026-11-20, exit immediately and do nothing.
2. Fetch, then check out or create the branch, then read the ledger.
3. If `status: DONE`, exit immediately.
4. If `status: RUNNING` and the heartbeat is < 25 min old, another run is active: exit immediately.
5. Otherwise, take over: set the heartbeat and `run: n+1`, commit and push. Then continue from the first unfinished job. Re-run any job whose CSV isn't committed.
6. While working, refresh the heartbeat (commit + push) at least every 15 min and after every job.
7. **Wrap-up deadline.** A run that starts at or after 19:30 UTC, or that reaches 19:30 UTC while working, finishes its current job and stops new ones. It writes REPORT.md with what is done (marking skipped items), opens the PR and sets `status: DONE`.
8. When everything is done and the head chef says YES:
   - REPORT.md is written;
   - the PR is open;
   - `status: DONE` is committed and pushed.

## Tasks, in order (an earlier task finishes first, unless it is waiting on sims)

### T0: Baseline at 1.477 kg (today's main)
- Jobs:
  - holdout 2,000;
  - dev 600;
  - wind sweep W ∈ {0, 2, 4, 5, 6, 7} m/s (one job per W, `--wind W`), screen 300 each.
- Record: success, failure codes, apogee, contact speeds, aborts by phase.
- Every later task pairs against these, on the same seeds.

### Task 1: `chute wiggle` bench command (firmware, ships in the PR)
- **The gap:** `chute cycle` (src/hw/cli.h) still runs the old LOCK-dwell → RELEASE motion. The owner can't bench-test the real flight release sequence.
- **Add `chute wiggle`.** It runs the exact flight sequence once on the ground:
  - drive a real `ChuteDeploy` (src/core/chute_deploy.h): `start()`, then step `update()` every 2 ms with 0 g / altitude 100 m;
  - map each `servoCmd()` to the servo through ONE mapping shared with `Actuators::applyChute`. Factor out e.g. `Actuators::writeChuteCmd(ChuteServo)`, used by both; no duplicated angles or dwells;
  - stop when the sequence reaches OPEN after the last wiggle stroke, i.e. `releases() == 1 + CHUTE_WIG_CYCLES` and the servo at OPEN. Don't run the 1.5 s waits or the slow re-cycles.
- **Behaviour:**
  - disarmed only, guarded like the other moving commands;
  - feeds the watchdog;
  - prints every target with a millisecond timestamp, so the owner can line it up with video (110° @0, 55° @190, 110° @340, 55° @490, 180° @640 ms with today's config);
  - saves the last position like `chute cycle`;
  - prints the `chute set` / load / `chute lock` reminder.
- **Docs:**
  - help text;
  - the README bench table (with the owner's checks: 110° frees the door with the spring loaded, 55° doesn't re-catch it; LOCK→110° ≤ 190 ms, 55→110° ≤ 150 ms under load, else lengthen the `CHUTE_WIG_*_MS` dwells).
- **Tests:**
  - a PC check that drives `chute wiggle` through the HIL host (`tools/hil/test/`, mock Servo in `tools/hil/test/mock/Servo.h`; extend the mock to log `(ms, us)` writes if needed) and asserts the pulse sequence and timings (±4 ms);
  - full suites green: `tools/replay make test`, `tools/sim/test_bridge.py`, `tools/sim/test_landing_outcome.py`, `tools/hil/test_hil.py`, `tools/hil/test_live.py`;
  - the `applyChute` refactor must be result-neutral: the T0 holdout re-run with the task-1 tree is identical flight for flight. It's cheap: dev 600 is enough if identical.

### Task 2: drag-from-log tool `tools/fit_cda.py` (ships in the PR)
- **Input:** a RocketFC flight log CSV (`tools/sim/logfile.read_log`), plus optional `--mass` (default `MASS_DESCENT_KG` via `tools/burn_table.config_value`) and `--rho` (default 1.225), or `--alt-m` / `--temp-c` to compute ρ.
- **Method:** in unpowered flight the accelerometer reads only drag: `CdA = 2·m·|a_spec| / (ρ·v²)`.
  - Use COAST (after BURNOUT, ascending) and DESCENT (falling, before FIRE_LANDING or a chute release).
  - Use samples with |kf_vel| ≥ 4 m/s.
  - Report per phase: a least-squares fit, the median, sample count, and the fit's spread.
  - Warn when there are under ~50 samples, or the speed range is narrow.
  - Note: the coast is nose-first and the descent tail-first, so they may differ. Check how `tools/sim/plant.py` applies `cda_m2` and say so.
  - Check the log's accel units in `logfile.py`.
- **Validation (`tools/test_fit_cda.py`, assert-based):**
  - sim logs from `run.simulate(..., record=True)` + `logfile.write_log` at plant `cda_m2` 0.003 / 0.004 / 0.006, land and chute modes, 5 seeds each;
  - the fit recovers the plant value within ±15 % (median over seeds). If a phase can't (e.g. too few fast samples), document why and exclude it, with the test saying so.
- **Docs:** update the README `CDA_M2` row to point at the tool.

### Task 4: burn-table margin re-tune at 1.477 kg (report only)
- Sweep `BURN_TABLE_MARGIN_M` ∈ {0, 0.25, 0.5, 0.75, 1.0 (today), 1.25, 1.5} on dev 600 each.
- Show the failure-code mix per margin. LEGS_BROKEN at the 3 m/s limit is 19 % in a 600-flight check and is the top failure.
- **Gate:**
  - take the best margin to the holdout (paired vs 1.0);
  - if p < 0.05 better, confirm on fresh seeds 801–810;
  - recommend only if both are significant.

### Task 5: earlier missed-window abort, chute ASAP (report only)
- **Today:** in DESCENT, a missed ignition window aborts only below `MISSED_WINDOW_ALT_M` (4 m), so the chute comes out at ≤ 4 m (Nov 4 study: 3.7–5.7 m in 5–6 m/s wind).
- **Variants** (scratch copy, off by default):
  - (a) `MISSED_WINDOW_ALT_M` ∈ {5, 6, 8};
  - (b) a "window passed" rule: abort when `kfAlt < hIgnite(vDown) + BURN_TABLE_MARGIN_M − D` while the fire interlocks still fail, D ∈ {0.5, 1, 2} m.
- **Conditions:**
  - `--wind 5` and `--wind 6`, dev 600 each (where missed windows happen);
  - the default-wind holdout, to show nothing is lost.
- **Metrics:**
  - chute release height on missed-window flights (median, p10, share above 6 / 8 m);
  - landings rescued / lost, paired, per condition;
  - flights that aborted early but would have landed.
- **Gate:** release height up, and no significant landing loss on the holdout or in either wind (p > 0.05, lost ≤ gained + 1 %).

### Task 6: late-ignition risk (report only)
- **The risk:** a real ignition delay above ~780 ms is now declared a dud (`IGNITION_DELAY_MS` 620 + `DUD_WINDOW_MS` 200, confirm ~40 ms after thrust onset). The chute is released, TVC goes off, the legs stay stowed, and then the motor lights.
- **Plant-only true delay:** `--plant ignition_delay_s=X --disp ignition_delay_s=0.0` with X ∈ {0.62, 0.70, 0.76, 0.80, 0.84, 0.90, 1.00}, screen 300 each.
  - Report: false-dud rate; what happens after a late light (motor thrust with the chute out: tilt, max height, impact speed, landing outcome). The sim can't model exhaust damage to the canopy; say so.
- **Trade-off curve:** `DUD_WINDOW_MS` ∈ {200, 300, 400} × those delays: false-dud rate vs the chute-release height on real duds (scratch `PROOF_DUD`-style dud switch: `run.simulate(dud_landing_igniter=True)`; see `studies/e18w-port-check-2026-11-20/run_proof.sh`). The owner's rule is chute ASAP, so don't recommend a longer window unless the trade is overwhelming. Just show the curve.
- **One guard variant** (scratch, off by default): after a DUD_IGNITER abort, if real thrust appears (accel > `IGNITE_CONFIRM_G` above `TOUCHDOWN_ALT_M`), re-enable TVC attitude hold with landing gains until burnout. Measure tip-over / impact vs no guard.

### Task 7: re-test the shelved fixes on the E18W at 1.477 kg (report only)
- **Candidates**, all tested before only on the F10/F15, each hand-ported into a scratch copy, off by default:
  - (a) the D2 predicted-tilt interlock, from `studies/software-landing-study/patches/final_S3_D2.patch` (S3 is already in `src/`; take only the D2 part);
  - (b) the wind-matched ascent, from `new-ideas-2026-10-19.md` and `patches/ideas_full.patch`. That patch predates the 2026-10-20 sim fixes, so port the firmware logic only;
  - (c) `DRIFT_GAIN` ∈ {1, 2 (today), 3, 4};
  - (d) `DRIFT_MAX_TILT_DEG` ∈ {3, 5 (today), 8}.
- **Conditions:** the wind sweep W ∈ {0, 2, 4, 5, 6, 7} m/s (one job per W, `--wind W`), screen 300 each, paired to the T0 sweep. Winners also get the default-wind holdout, plus fresh seeds.
- Wind-matched ascent needed gimbal zero within ±0.2°: add a ±0.5° gimbal-error robustness run (use the sim's gimbal-offset dispersion if it exists, else a plant knob).
- **Goal:** raise the wind limit, today ~4 m/s, without losing calm-wind landings.
- **Gate:** significant on the holdout and fresh seeds, and not worse at 0–2 m/s.

### Task 8: TVC gain sweep for the E18W (report only)
- `GAINS_BOOST` and `GAINS_LANDING` in `src/config.h` are bench placeholders from older motors.
- Through `--fw-overrides` (the struct patcher):
  - one-at-a-time scales of kp and kd ∈ {0.5, 0.75, 1, 1.5, 2}, and ki ∈ {0, 1, 2}× for each gain set (screen 300 each);
  - then a 3 × 3 kp × kd grid around the best landing set.
- **Metrics:** success, failure codes, max tilt, tilt aborts, sideways contact speed, gimbal-saturation share.
- **Finalists:**
  - holdout + fresh;
  - robustness: `--wind 5`;
  - a ±30 % inertia_kgm2 (estimated, not measured);
  - a slower servo, if `tools/sim/servo.py` has a rate or lag knob.
- **Gate:** recommend only a set that wins on the holdout and fresh seeds and stays robust. Say clearly that the sim's inertia and servo model are estimates, so the gains still need bench and flight checks.

## Deliverables
- **Branch `overnight-2026-11-20`:**
  - task 1 (firmware CLI + test) and task 2 (tool + test), with all suites green;
  - `studies/overnight-2026-11-20/`: PLAN.md, LEDGER.md, scripts, `results/*.csv.gz`, `patches/` (unapplied, off-by-default), and **REPORT.md**.
- **REPORT.md** is short: one section per task with numbers, reasons and the recommended fix, plus a top "Recommended next steps" list.
- **The PR into `main`:** title "Overnight 2026-11-20: chute wiggle bench command, CdA tool, sim studies at 1.477 kg". The body summarizes REPORT.md and the test results.
- Do NOT merge.
