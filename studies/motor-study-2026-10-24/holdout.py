"""Pre-registered holdout, fresh seeds 701-710 (2,000 flights per row, run once).
Bar for each fix, added one at a time on top of the last winner:
  F10 ascent: rescued > lost with sign test p < 0.05, aborts and crashes not up > 1 point;
  F15 ascent (today's hardware): aborts and crashes not up > 1 point."""
import json, sys
sys.path.insert(0, ".")
import drv
M = json.load(open("motors/motors.json"))
F10 = {"ascent_eng_path": M["F10"]["eng"]}
S3 = {"DRIFT_GAIN": "2.0f"}
S3M = dict(S3, BURN_TABLE_MARGIN_M="1.0f")
S3MW = dict(S3M, BOOST_DRIFT_GAIN="1.0f", WIND_MATCH="true", WIND_EST="true", WIND_EST_PRIOR_MPS="1.0f")
S3D2 = dict(S3, TILT_PREDICT_INTERLOCK="true")
H = drv.HOLD


def verdict(prev, rows, need_gain):
    a, b = drv.metrics(prev), drv.metrics(rows)
    resc, lost, p = drv.paired(prev, rows)
    d_ab, d_cr = 100 * (b["abort"] - a["abort"]), 100 * (b["crash_all"] - a["crash_all"])
    ok = d_ab <= 1.0 and d_cr <= 1.0 and (not need_gain or (resc > lost and p < 0.05))
    print("        paired: rescued %d lost %d p=%.4g | aborts %+.1f pt, crashes %+.1f pt -> %s" % (
        resc, lost, p, d_ab, d_cr, "PASS" if ok else "FAIL"), flush=True)
    return ok


def chain(tag, ws, plant, steps, need_gain, disp=drv.MEAS):
    prev = drv.run("ho_%s_today" % tag, ws=ws, seeds=H, plant=plant, disp=disp)
    drv.show("HOLDOUT %s / today" % tag, prev)
    res = {}
    for name, fw in steps:
        rows = drv.run("ho_%s_%s" % (tag, name), ws=ws, seeds=H, fw=fw, plant=plant, disp=disp)
        drv.show("HOLDOUT %s / %s" % (tag, name), rows)
        res[name] = verdict(prev, rows, need_gain)
        if res[name]:
            prev = rows
    return res


print("== F10 ascent, measured spreads")
r10 = chain("F10", "m_F10", F10, [("S3", S3), ("S3M", S3M), ("S3MW", S3MW)], True)
print("== D2 on its own, on top of S3 (F10)")
s3 = drv.run("ho_F10_S3", ws="m_F10", seeds=H, fw=S3, plant=F10)
d2 = drv.run("ho_F10_S3D2", ws="m_F10", seeds=H, fw=S3D2, plant=F10)
drv.show("HOLDOUT F10 / S3D2", d2)
verdict(s3, d2, True)
print("== F15 ascent (today's hardware), measured spreads: must not get worse")
r15 = chain("F15", "fix", None, [("S3", S3), ("S3M", S3M), ("S3MW", S3MW)], False)
print("== Reference: F10 with today's (unmeasured) spreads")
a = drv.run("ho_F10t_today", ws="m_F10", seeds=H, plant=F10, disp={})
b = drv.run("ho_F10t_S3M", ws="m_F10", seeds=H, fw=S3M, plant=F10, disp={})
drv.show("HOLDOUT F10 today-spreads / today", a)
drv.show("HOLDOUT F10 today-spreads / S3M", b)
print("        paired: rescued %d lost %d p=%.4g" % drv.paired(a, b))
json.dump({"F10": r10, "F15": r15}, open("results/holdout_verdicts.json", "w"), indent=1)
