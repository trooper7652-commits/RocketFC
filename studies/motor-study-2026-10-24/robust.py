"""F10 robustness on dev seeds: fixes added one at a time (no D2), under stress cases."""
import json, sys
sys.path.insert(0, ".")
import drv
M = json.load(open("motors/motors.json"))
ENG = M["F10"]["eng"]
S3 = {"DRIFT_GAIN": "2.0f"}
S3M = dict(S3, BURN_TABLE_MARGIN_M="1.0f")
S3MW = dict(S3M, BOOST_DRIFT_GAIN="1.0f", WIND_MATCH="true", WIND_EST="true", WIND_EST_PRIOR_MPS="1.0f")
CHAIN = [("today", None), ("S3", S3), ("S3M", S3M), ("S3MW", S3MW)]
CASES = [("nominal", drv.MEAS, {}, None),
         ("todayspreads", {}, {}, None),
         ("impulse5", drv.MEAS, {}, ["ascent_thrust_scale:0.95:1.05"]),
         ("gimbal05", drv.MEAS, {}, ["gimbal_offset_x_deg:-0.5:0.5", "gimbal_offset_y_deg:-0.5:0.5"]),
         ("noise2x", drv.MEAS, {"accel_noise_thrust": 1.6}, None)]
for cname, disp, pl, ext in CASES:
    prev = None
    for fname, fw in CHAIN:
        rows = drv.run("rb_F10_%s_%s" % (cname, fname), ws="m_F10", fw=fw, disp=disp, ext=ext,
                       plant=dict({"ascent_eng_path": ENG}, **pl))
        drv.show("F10 %s / %s" % (cname, fname), rows)
        if prev is not None:
            print("        vs previous step: rescued %d lost %d p=%.4f" % drv.paired(prev, rows), flush=True)
        prev = rows
