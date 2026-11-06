import json, sys
sys.path.insert(0, ".")
import drv
M = json.load(open("motors/motors.json"))
ENG = M["F10"]["eng"]
S3 = {"DRIFT_GAIN": "2.0f"}
S3M = dict(S3, BURN_TABLE_MARGIN_M="1.0f")
S3MW = dict(S3M, BOOST_DRIFT_GAIN="1.0f", WIND_MATCH="true", WIND_EST="true", WIND_EST_PRIOR_MPS="1.0f")
GOFF = ["gimbal_offset_x_deg:-1:1", "gimbal_offset_y_deg:-1:1"]
for g in (0.5, 1.0):
    prev = None
    for fname, fw in [("today", None), ("S3", S3), ("S3M", S3M), ("S3MW", S3MW)]:
        rows = drv.run("rbg_F10_g%.1f_%s" % (g, fname), ws="m_F10", fw=fw, ext=GOFF,
                       plant={"ascent_eng_path": ENG, "gimbal_offset_x_deg": g, "gimbal_offset_y_deg": g})
        drv.show("F10 gimbal +-%.1f / %s" % (g, fname), rows)
        if prev is not None:
            print("        vs previous step: rescued %d lost %d p=%.4f" % drv.paired(prev, rows), flush=True)
        prev = rows
