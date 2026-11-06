"""Dev-seed ablation matrix (seeds 7, 11, 13 x 200). Each config = a workspace
copy (which landing_table.h it carries) + config.h overrides for the flight DLL.
Writes data/abl_<name>.json and prints pre-registered metrics + paired deltas
vs the unchanged baseline (data/base_dev.json)."""
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
L = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import metrics as M  # noqa: E402

RISK = {"LANDING_SOLVER": "LandingSolverMode::RISK"}


def cfg(lag, tilt, pred, cap=None):
    d = dict(RISK, LAG_TAU_S=f"{lag}f", LANDING_TILT_COMP="true" if tilt else "false",
             TILT_PREDICT_INTERLOCK="true" if pred else "false")
    if cap is not None:
        d["TILT_IGNITE_HARD_MAX_DEG"] = f"{cap}f"
    return d


CONFIGS = [
    # name, workspace, overrides
    ("nom_noB", "ws_nomtab", cfg(0.0, False, False)),      # sanity: ~ old table
    ("B", "ws_nomtab", cfg(0.017, False, False)),
    ("D2", "ws_nomtab", cfg(0.0, False, True)),
    ("B+D2", "ws_nomtab", cfg(0.017, False, True)),
    ("C", "ws_cand", cfg(0.0, False, False)),
    ("B+C", "ws_cand", cfg(0.017, False, False)),
    ("B+C+D1", "ws_cand", cfg(0.017, True, False)),
    ("C+D2", "ws_cand", cfg(0.0, False, True)),
    ("B+C+D2", "ws_cand", cfg(0.017, False, True)),
    ("B+C+D1+D2", "ws_cand", cfg(0.017, True, True)),
    ("B+C_hp2", "ws_hp2", cfg(0.017, False, False)),
    ("B+C_hp5", "ws_hp5", cfg(0.017, False, False)),
    ("B+C_hp10", "ws_hp10", cfg(0.017, False, False)),
    ("B+C+D2_hp2", "ws_hp2", cfg(0.017, False, True)),
    ("B+C+D2_hp5", "ws_hp5", cfg(0.017, False, True)),
    ("B+C+D2_hp10", "ws_hp10", cfg(0.017, False, True)),
    ("B+C+D2_cap20", "ws_cand", cfg(0.017, False, True, 20.0)),
    ("B+C+D2_cap30", "ws_cand", cfg(0.017, False, True, 30.0)),
]


def run(name, ws, ov, seeds=(7, 11, 13), tag="abl", extra=()):
    out = os.path.join(L, "data", f"{tag}_{name}.json")
    if not os.path.exists(out):
        subprocess.run([sys.executable, os.path.join(HERE, "mc.py"), "--ws",
                        os.path.join(L, ws, "RocketFC"), "--seeds", *map(str, seeds),
                        "--n", "200", "--fw-overrides", json.dumps(ov), "--out", out,
                        *extra], check=True)
    return M.load(out)


def main():
    only = sys.argv[1:]
    base = M.load(os.path.join(L, "data", "base_dev.json"))
    lines = [M.fmt_stats("BASELINE", M.stats(base))]
    for name, ws, ov in CONFIGS:
        if only and name not in only:
            continue
        rows = run(name, ws, ov)
        lines.append(M.fmt_stats(name, M.stats(rows)))
        lines.append("    " + M.fmt_paired("vs baseline", M.paired(base, rows)))
        print(lines[-2] + "\n" + lines[-1], flush=True)
    with open(os.path.join(L, "data", "ablation.txt"), "a", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n\n")


if __name__ == "__main__":
    main()
