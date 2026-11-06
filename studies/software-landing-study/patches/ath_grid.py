"""Development grid for artificial throttling (ATH), seeds 101-103 x 200,
today's spreads. Every config includes S3 + D2 + GPS 0.3 (the best setup)."""
import json
import os
import sys

import study

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")
BEST = {"DRIFT_GAIN": "2.0f", "TILT_PREDICT_INTERLOCK": "true", "GPS_VEL_GAIN": "0.3f"}
ENG = os.path.abspath(os.path.join(study.HERE, "motors", "F15_x1.10.eng"))
PL = {"x1.00": {"gps_hz": 10.0}, "x1.10": {"gps_hz": 10.0, "ascent_eng_path": ENG}}
GRID = {
    "x1.00": [(th, mg, k) for th in (15, 20, 25) for mg in (0, 2, 4) for k in (6, 15)],
    "x1.10": [(th, mg, k) for th in (10, 15, 20) for mg in (0, 2) for k in (6, 15)],
}
summary = {}
for apo in ("x1.00", "x1.10"):
    base_key = "best_{}".format(apo.replace(".", ""))
    rows = study.run(base_key if apo == "x1.00" else "gps_g03", ws="cand", fw=BEST, plant=PL[apo])
    m, c = study.metrics(rows), study.metrics(rows, calm=True)
    summary[apo + " baseline"] = (m, c)
    print(study.line("{} baseline (no ATH)".format(apo), m), flush=True)
    print("      calm succ {:.1f}%".format(100 * c["rate"][0]), flush=True)
    for th, mg, k in GRID[apo]:
        fw = dict(BEST, THR_TILT_MAX_DEG="{:.1f}f".format(th),
                  BURN_TABLE_MARGIN_M="{:.1f}f".format(mg), THR_K="{:.1f}f".format(k))
        key = "ath_{}_th{}_m{}_k{}".format(apo.replace(".", ""), th, mg, k)
        rows = study.run(key, ws="cand", fw=fw, plant=PL[apo])
        m, c = study.metrics(rows), study.metrics(rows, calm=True)
        summary[key] = (m, c)
        print(study.line("{} th{} m+{} k{}".format(apo, th, mg, k), m), flush=True)
        print("      calm succ {:.1f}%".format(100 * c["rate"][0]), flush=True)
json.dump({k: {"all": v[0], "calm": v[1]} for k, v in summary.items()},
          open(os.path.join(study.RES, "ath_grid_summary.json"), "w"), indent=1, default=str)
