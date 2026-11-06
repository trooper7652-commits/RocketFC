import json, sys
sys.path.insert(0, ".")
import drv
M = json.load(open("motors/motors.json"))
SETS = [("F15", "fix", {}), ("F15x110", "fix", {"ascent_eng_path": M["F15x110"]["eng"]})]
for k in ("F44W", "E28T", "41F3611A", "F10"):
    SETS.append((k, M[k]["ws"], {"ascent_eng_path": M[k]["eng"]}))
for fwname, fw in (("today", None), ("S3D2", {"DRIFT_GAIN": "2.0f", "TILT_PREDICT_INTERLOCK": "true"})):
    print("== firmware:", fwname)
    for name, ws, plant in SETS:
        rows = drv.run("scr_%s_%s" % (name, fwname), ws=ws, seeds=(1, 2, 3, 4, 5), n=1, fw=fw,
                       plant=plant or None, disp=None, plant_list=[{}])
        rr = drv.ok(rows)
        s = [r["summary"] for r in rr]
        print("  %-9s apogee %s | fall@fire %s | contact vz %s | tilt max %s | outcome %s" % (
            name,
            " ".join("%.1f" % x["apogee_h"] for x in s),
            " ".join("%.1f" % -x["fire_v_true"] if x.get("fire_v_true") is not None else "-" for x in s),
            " ".join("%.1f" % abs(x["ground"]["contact"]["vel"][2]) if (x.get("ground") or {}).get("contact") else "-" for x in s),
            " ".join("%.0f" % x["max_tilt_deg"] for x in s),
            ",".join(x["landing_outcome"]["code"][:8] for x in s)), flush=True)
