"""Dev grid: motor x firmware set, measured spreads, seeds 101-103 (600 flights each)."""
import json, sys
sys.path.insert(0, ".")
import drv
M = json.load(open("motors/motors.json"))
MOTORS = [("F15", "fix", None), ("F15x110", "fix", M["F15x110"]["eng"])] + \
         [(k, M[k]["ws"], M[k]["eng"]) for k in ("F10", "E28T", "41F3611A")]
S3 = {"DRIFT_GAIN": "2.0f"}
S3D2 = dict(S3, TILT_PREDICT_INTERLOCK="true")
S3D2M = dict(S3D2, BURN_TABLE_MARGIN_M="1.0f")
WM = dict(S3D2M, BOOST_DRIFT_GAIN="1.0f", WIND_MATCH="true", WIND_EST="true", WIND_EST_PRIOR_MPS="1.0f")
FW = [("today", None), ("S3", S3), ("S3D2", S3D2), ("S3D2M", S3D2M), ("S3D2MW", WM)]
only = sys.argv[1:] or None
for mname, ws, eng in MOTORS:
    if only and mname not in only: continue
    base = None
    for fname, fw in FW:
        rows = drv.run("dev_%s_%s" % (mname, fname), ws=ws, fw=fw, plant={"ascent_eng_path": eng} if eng else None)
        m = drv.show("%s / %s" % (mname, fname), rows)
        if base is None: base = rows
        else: print("        vs today: rescued %d lost %d p=%.4f" % drv.paired(base, rows), flush=True)
