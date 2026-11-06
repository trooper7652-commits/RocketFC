"""One-shot holdout for the GPS-aided drift loop (seeds 201-210, run once)."""
import json
import os
import sys

import study

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")
SEEDS = list(range(201, 211))
FIN_GPS = {"DRIFT_GAIN": "2.0f", "TILT_PREDICT_INTERLOCK": "true", "GPS_VEL_GAIN": "0.3f"}
ENG = os.path.abspath(os.path.join(study.HERE, "motors", "F15_x1.10.eng"))
MEAS = {"mass_descent_kg": 0.01, "ignition_delay_s": 0.05}
CONFIGS = [
    ("today", None, "FINAL+GPS, today apogee", {"gps_hz": 10.0}),
    ("today", None, "FINAL+GPS, apogee x1.10", {"gps_hz": 10.0, "ascent_eng_path": ENG}),
    ("measured", MEAS, "FINAL+GPS, apogee x1.10", {"gps_hz": 10.0, "ascent_eng_path": ENG}),
]
prev = json.load(open(os.path.join(study.RES, "holdout_summary.json")))
out = {}
for scen, disp, lab, pl in CONFIGS:
    key = "holdgps_{}_{}".format(scen, lab.replace(" ", "_").replace(",", "").replace("+", "p"))
    rows = study.run(key, ws="cand", seeds=SEEDS, fw=FIN_GPS, plant=pl, disp=disp)
    m, c = study.metrics(rows), study.metrics(rows, calm=True)
    out[scen + " | " + lab] = {"all": m, "calm": c}
    print(study.line(scen[:4] + " " + lab, m), flush=True)
    print("      calm (n={}): succ {:.2f}% [{:.1f}-{:.1f}]  k={}  vh med {:.2f}".format(
        c["n"], *(100 * x for x in c["rate"]), m["k"], m["vh_med"]), flush=True)
json.dump(out, open(os.path.join(study.RES, "holdout_gps_summary.json"), "w"), indent=1, default=str)
print("previous (no GPS) for comparison:")
for k in ("today | FINAL software, today apogee", "today | FINAL software, apogee x1.10",
          "measured | FINAL software, apogee x1.10"):
    m = prev[k]["all"]
    print("   {:40s} succ {:.2f}% (k={})  calm {:.2f}%".format(
        k, 100 * m["rate"][0], m["k"], 100 * prev[k]["calm"]["rate"][0]))
