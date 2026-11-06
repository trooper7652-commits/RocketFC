"""One-shot holdout: seeds 201-210 x 200 flights, never looked at before."""
import json
import os
import sys

import study

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")
SEEDS = list(range(201, 211))
FINAL = {"DRIFT_GAIN": "2.0f", "TILT_PREDICT_INTERLOCK": "true"}
X110 = {"ascent_eng_path": os.path.abspath(os.path.join(study.HERE, "motors", "F15_x1.10.eng"))}
MEAS = {"mass_descent_kg": 0.01, "ignition_delay_s": 0.05}
CONFIGS = [
    ("today firmware, today apogee", None, None),
    ("FINAL software, today apogee", FINAL, None),
    ("today firmware, apogee x1.10", None, X110),
    ("FINAL software, apogee x1.10", FINAL, X110),
]
out = {}
for scen, disp in (("today", None), ("measured", MEAS)):
    for lab, fw, pl in CONFIGS:
        key = "hold_{}_{}".format(scen, lab.replace(" ", "_").replace(",", ""))
        rows = study.run(key, ws="cand", seeds=SEEDS, fw=fw, plant=pl, disp=disp)
        m, c = study.metrics(rows), study.metrics(rows, calm=True)
        out[scen + " | " + lab] = {"all": m, "calm": c}
        print(study.line(scen[:4] + " " + lab, m), flush=True)
        print("      calm (n={}): succ {:.1f}% [{:.1f}-{:.1f}]".format(
            c["n"], *(100 * x for x in c["rate"])), flush=True)
json.dump(out, open(os.path.join(study.RES, "holdout_summary.json"), "w"), indent=1, default=str)
