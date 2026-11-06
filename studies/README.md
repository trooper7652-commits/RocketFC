# Simulator design studies

Every study ran in throwaway copies of RocketFC built with the closed-loop simulator (`tools/sim`). The scripts here rebuild those copies. Nothing in a study changes the firmware until it is ported on purpose.

| Study | Question | Start with |
|---|---|---|
| `landing-solver-study` (2026-10-12) | Does a better landing-burn solver land softer? It doesn't: per-flight dispersions dominate. | `README.md` |
| `landing-outcome-study` (2026-10-17) | What share of flights land upright and intact, and why do they fail? | `report.md` |
| `landing-outcome-study-2026-10-24` | The same question, re-run on the fixed simulator | `report.md` |
| `software-landing-study` (2026-10-17..20) | Software-only fixes: drift loop, interlocks, GPS, wind-matched ascent | `report.md`, `new-ideas-2026-10-19.md` |
| `motor-study-2026-10-24` | Ascent motor search, leg timing, which fixes to port | `RESUME.md` |
| `landing-velocity-study-2026-10-25` | Error budget, low-thrust landing motors (E12/E12), casing eject, ballast fix | `casing-eject/README.md`, `casing-eject/fix/README.md` |
| `motor-chute-study-2026-11-04` | Motors at 1.4 kg (E18W/E18W), parachute size, clustered chutes | `README.md` |

**Raw data is not in the repo.** The raw Monte Carlo result files (`results.zip`, `results.json`, `data.zip`, `example_*.json`, about 357 MB together, one over GitHub's 100 MB limit) stay in the original study folders beside the repo. The study reports and logs here keep every number. Re-running a study's scripts rebuilds its results.
