import json, math, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import flyone as F

R = os.path.join(F.SCR, "sw_study", "results")
DLL = F.dll_for(F.IMU_BEST)
d = json.load(open(os.path.join(R, "hold_measured_FINAL_software_apogee_x1.10.json")))
rows = [r for r in d["rows"] if "error" not in r["summary"] and r["summary"].get("fire_t")]
pick = []
for lo, hi in [(0, 0.5), (0.5, 1.0), (1.0, 2.0)]:
    sub = [r for r in rows if lo <= abs(r["dispersions"]["wind_mps"]) < hi][:3]
    pick += sub
plant = {"ascent_eng_path": F.X110}
for r in pick:
    rr, summ, vc, disp = F.fly(DLL, r["seed"], r["i"], plant=plant, disp=F.MEASURED)
    # dead reckoning exactly like flight_core: from launch (state != IDLE/ARMED)
    vx = vy = 0.0
    out = []
    prev = None
    for k, row in enumerate(rr):
        st = row["state"]
        if st in ("IDLE", "ARMED"):
            vx = vy = 0.0
        else:
            aw = F.qrot(row["quat_est"], (row["ax"], row["ay"], row["az"]))
            vx += aw[0] * 0.002
            vy += aw[1] * 0.002
        if k > 0:
            tvx = (row["x_true"] - rr[k - 1]["x_true"]) / 0.002
            tvy = (row["y_true"] - rr[k - 1]["y_true"]) / 0.002
        else:
            tvx = tvy = 0.0
        if st != prev or k == len(rr) - 1:
            te = F.tilt_of(row["quat_true"]); ee = F.tilt_of(row["quat_est"])
            # attitude error: angle between true and est body z
            zt = F.qrot(row["quat_true"], (0, 0, 1)); ze = F.qrot(row["quat_est"], (0, 0, 1))
            dot = max(-1, min(1, sum(a * b for a, b in zip(zt, ze))))
            out.append("%s@%.2f vtrue=(%.2f,%.2f) vest=(%.2f,%.2f) tilt %.1f att_err %.2f" % (
                st, row["t"], tvx, tvy, vx, vy, te, math.degrees(math.acos(dot))))
            prev = st
    print("seed %d i %d wind %.2f -> %s | fire tilt %.1f fire hs %.2f td hs %.2f" % (
        r["seed"], r["i"], vc.wind_mps, summ["landing_outcome"]["code"],
        summ["fire_tilt_deg"] or -1, summ["fire_hspeed_true"] or -1, summ["touchdown_hspeed_mps"] or -1))
    for o in out:
        print("    ", o)
