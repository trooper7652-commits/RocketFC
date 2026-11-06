"""One-shot holdout for artificial throttling, fresh seeds 401-410 (never used),
apogee x1.10 (at today's apogee every dev config was 0 %, so a holdout there
can't show anything). Paired comparisons on identical flights."""
import json
import math
import os
import sys

import study

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")
SEEDS = list(range(401, 411))
BEST = {"DRIFT_GAIN": "2.0f", "TILT_PREDICT_INTERLOCK": "true", "GPS_VEL_GAIN": "0.3f"}
ENG = os.path.abspath(os.path.join(study.HERE, "motors", "F15_x1.10.eng"))
PL = {"gps_hz": 10.0, "ascent_eng_path": ENG}
MEAS = {"mass_descent_kg": 0.01, "ignition_delay_s": 0.05}
ATH = {"THR_TILT_MAX_DEG": "20.0f", "THR_K": "15.0f"}
CONFIGS = [
    ("today", None, "best, margin 0", dict(BEST)),
    ("today", None, "best + ATH, margin 0", dict(BEST, **ATH)),
    ("measured", MEAS, "best, margin 0", dict(BEST)),
    ("measured", MEAS, "best, margin +1 (no ATH)", dict(BEST, BURN_TABLE_MARGIN_M="1.0f")),
    ("measured", MEAS, "best + ATH, margin +1", dict(BEST, BURN_TABLE_MARGIN_M="1.0f", **ATH)),
]
rows_by = {}
out = {}
for scen, disp, lab, fw in CONFIGS:
    key = "holdath_{}_{}".format(scen, lab.replace(" ", "_").replace(",", "").replace("+", "p")
                                 .replace("(", "").replace(")", ""))
    rows = study.run(key, ws="cand", seeds=SEEDS, fw=fw, plant=PL, disp=disp)
    rows_by[(scen, lab)] = rows
    m, c = study.metrics(rows), study.metrics(rows, calm=True)
    out[scen + " | " + lab] = {"all": m, "calm": c}
    print(study.line(scen[:4] + " " + lab, m), flush=True)
    print("      calm (n={}): succ {:.2f}% [{:.1f}-{:.1f}]  k={}".format(
        c["n"], *(100 * x for x in c["rate"]), m["k"]), flush=True)


def pair(a, b):
    sa = [r["summary"]["landing_outcome"]["success"] for r in rows_by[a]]
    sb = [r["summary"]["landing_outcome"]["success"] for r in rows_by[b]]
    g = sum(y and not x for x, y in zip(sa, sb))
    lo = sum(x and not y for x, y in zip(sa, sb))
    n = g + lo
    # two-sided exact sign test
    p = min(1.0, 2 * sum(math.comb(n, i) for i in range(0, min(g, lo) + 1)) / 2 ** n) if n else 1.0
    return g, lo, p


print("paired (rescued, lost, sign-test p):")
for a, b in ((("today", "best, margin 0"), ("today", "best + ATH, margin 0")),
             (("measured", "best, margin 0"), ("measured", "best + ATH, margin +1")),
             (("measured", "best, margin +1 (no ATH)"), ("measured", "best + ATH, margin +1")),
             (("measured", "best, margin 0"), ("measured", "best, margin +1 (no ATH)"))):
    g, lo, p = pair(a, b)
    out["pair | {} -> {}".format(a, b)] = [g, lo, p]
    print("   {} -> {}: rescued {}, lost {}, p = {:.3f}".format(a, b, g, lo, p), flush=True)
json.dump(out, open(os.path.join(study.RES, "holdout_ath_summary.json"), "w"), indent=1, default=str)
