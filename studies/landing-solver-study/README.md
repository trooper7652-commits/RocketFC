# Landing-burn solver study (2026-10-12)

The result of `..\burn-solver-revision-prompt.md`. **Start with `notes\REPORT.md`.**

Verdict: no new solver beat the current `burn_table.h` rule under the pre-registered test, so the
RocketFC project was left unchanged. The biggest wins available are measurements (ignition delay,
descent mass, thrust curve, baro lag) and mission-level fixes, not the solver.

| Folder | What's in it |
|---|---|
| `notes\` | `REPORT.md` (plain-language report), `00_current_system.md`, `01_research.md` (sources with flown/simulated/proposed status), `02_candidates.md`, `03_metrics.md` (pre-registered rules, SHA-256 in `03_metrics.sha256`), `03_metrics_amendments.md`, `04_results.md`, `hashes_at_copy.txt`, `plots\` |
| `candidate\` | The tested candidate solver, **not adopted**: `landing_solver_candidate.patch` (applies with `patch -p1` from the RocketFC root; defaults to `LANDING_SOLVER = TABLE`, i.e. bit-identical to today, with the tilt-loss and predicted-tilt options off), plus the new files on their own: `landing_solver.h`, `landing_table.h`, `landing_table.py`. Also `sim_scaffold_patch.py`, which added the analysis instrumentation, perturbation knobs and test-only forced-fire hook to the workspace copies, and `sim_tooling_port_applied.diff`: exactly what was ported into `RocketFC\tools\sim` (uncommitted), apart from the new file `mc_batch.py`. |
| `scripts\` | Analysis tools: `mc.py`/`mc2.py` (Monte Carlo driver), `metrics.py` (pre-registered metrics and paired bootstrap), `oracle_scan.py` and `headroom.py` (forced-fire headroom analysis), `risk_table.py` and `value_of_info.py` (1-D physics), `ablate.py`, `robust.py`, `evaluate.py`. They expect the workspace layout described in REPORT.md. |
| `reference_curves\` | Third-party Estes F15 curves from ThrustCurve.org (two certification-derived RASP files and one user RockSim file), used as "true motor" perturbations. |
| `data.zip` | Every simulation run as JSON (about 150 MB unzipped), including the 25 881-flight forced-fire scan. |
