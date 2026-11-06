"""Part A, steps 1-2: filter the OpenRocket motor database for this rocket and
pre-screen every candidate with a point-mass model at 1.4 kg liftoff.

Filter (see README):
  - diameter <= 29 mm (24 mm and smaller fly in an adapter, +10 g ejected with
    the casing), class <= G (<= 160 N*s), propellant <= 62.5 g;
  - ascent motor (aft in the stack): a plugged ("P") version must exist. A -0
    booster vents forward at burnout and would light the landing motor.
    Reloads without a listed P are kept, flagged "plug?";
  - landing motor (forward): "P" or "-0", reloads flagged "plug?";
  - thrustcurve.org entries only (the RASAero rows duplicate them), the best
    curve per motor (cert > mfr > user, then most points).

Pre-screen:
  - ascent: vertical flight from 1.4 kg (CdA 0.004, sea level): apogee,
    burnout speed, coast time, thrust/weight;
  - landing: descent mass 1.4 - 0.025 kg ascent propellant, casing 0.035 kg
    pushed out at thrust onset, 620 ms ignition delay; for apogees 8-32 m,
    the best impact speed with perfect fire timing, and how wide (in fire
    height) the <= 2.5 m/s window is.

Writes motors/<key>.eng and prescreen.json.
"""
import json
import math
import multiprocessing
import os
import sqlite3
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DB = os.path.join(os.path.dirname(HERE), "motor-study-2026-10-24", "mdb", "motors.db")
RFC_TOOLS = os.path.join(os.path.dirname(HERE), "RocketFC", "tools")
sys.path.insert(0, RFC_TOOLS)
import burn_table as bt  # noqa: E402  (Motor, sim_burn)

M_PAD = 1.4
G = 9.80665
RHO = 1.225
CDA = 0.0040
DELAY = 0.62
ADAPTER_KG = 0.010
OUT_DIR = os.path.join(HERE, "motors")


def key_of(mfr, desig):
    k = (mfr.split()[0] + "_" + desig).replace("/", "-").replace(" ", "")
    return "".join(ch for ch in k if ch.isalnum() or ch in "-_.")


def load_candidates():
    c = sqlite3.connect(DB)
    rows = c.execute("""
        select m.id, f.name, m.designation, m.common_name, m.diameter, m.length,
               m.total_impulse, m.avg_thrust, m.max_thrust, m.burn_time,
               m.propellant_weight, m.total_weight, m.type, m.delays, m.case_info
        from motors m join manufacturers f on f.id = m.manufacturer_id
        where m.source = 'thrustcurve.org' and m.diameter <= 29.5
          and m.total_impulse <= 160 and m.propellant_weight <= 62.5
          and m.total_impulse >= 20""").fetchall()
    out = []
    for (mid, mfr, desig, common, dia, length, imp, avg, mx, bt_s, prop_g, tot_g,
         typ, delays, case) in rows:
        dl = [d.strip().upper() for d in (delays or "").split(",") if d.strip()]
        plugged = "P" in dl
        booster = "0" in dl
        reload_ = (typ or "").lower() == "reload"
        if not (plugged or booster or reload_):
            continue
        curves = c.execute("""
            select t.id, t.source, (select count(*) from thrust_data d where d.curve_id = t.id)
            from thrust_curves t where t.motor_id = ?""", (mid,)).fetchall()
        if not curves:
            continue
        rank = {"cert": 0, "mfr": 1, "user": 2}
        curves.sort(key=lambda r: (rank.get(r[1], 3), -r[2]))
        cid = curves[0][0]
        pts = c.execute("select time_seconds, force_newtons from thrust_data "
                        "where curve_id = ? order by time_seconds", (cid,)).fetchall()
        if len(pts) < 4 or not tot_g or not prop_g:
            continue
        out.append(dict(id=mid, mfr=mfr, desig=desig, common=common, dia=dia,
                        length=length, impulse=imp, avg=avg, max=mx, burn=bt_s,
                        prop_g=prop_g, total_g=tot_g, type=typ, delays=delays,
                        case=case, plugged=plugged, booster=booster, reload=reload_,
                        curve_source=curves[0][1], points=pts,
                        key=key_of(mfr, desig)))
    return out


def write_eng(m):
    pts = []
    for t, f in m["points"]:
        if t <= 0.0:
            continue
        if pts and t <= pts[-1][0]:
            continue
        pts.append((t, max(0.0, f)))
    if pts[-1][1] > 0.0:
        pts.append((pts[-1][0] + 0.01, 0.0))
    path = os.path.join(OUT_DIR, m["key"] + ".eng")
    dl = "P" if m["plugged"] else ("0" if m["booster"] else "plug?")
    with open(path, "w", encoding="ascii", errors="replace", newline="\n") as f:
        f.write("; %s %s from the OpenRocket motor database (thrustcurve.org, %s curve)\n"
                % (m["mfr"], m["desig"], m["curve_source"]))
        f.write("%s %d %d %s %.4f %.4f %s\n" % (
            m["key"], round(m["dia"]), round(m["length"] or 0), dl,
            m["prop_g"] / 1000.0, m["total_g"] / 1000.0, m["mfr"].split()[0]))
        for t, F in pts:
            f.write("%.4f %.3f\n" % (t, F))
    return path


def ascent_screen(eng):
    mot = bt.Motor(eng)
    m0 = M_PAD
    h = v = t = 0.0
    dt = 0.001
    lifted = False
    t_lift = None
    vmax = 0.0
    v_bo = h_bo = None
    while t < 30.0:
        T = mot.thrust(t)
        m = m0 - mot.mass_burned(t)
        a = T / m - G - 0.5 * RHO * CDA * v * abs(v) / m
        if not lifted:
            if a > 0:
                lifted, t_lift = True, t
            else:
                a, v = 0.0, 0.0
        v += a * dt
        h += v * dt
        t += dt
        vmax = max(vmax, v)
        if v_bo is None and t >= mot.burn_time:
            v_bo, h_bo = v, h
        if lifted and v <= 0.0 and t > mot.burn_time:
            break
        if lifted and h < 0:
            break
    first = [F for (tt, F) in mot.points if 0 < tt <= 0.3]
    return {"apogee": h, "v_burnout": v_bo, "h_burnout": h_bo,
            "coast_s": max(0.0, t - mot.burn_time), "vmax": vmax,
            "t_lift": t_lift,
            "tw_avg": mot.total_impulse / mot.burn_time / (m0 * G),
            "tw_first03": (sum(first) / len(first) / (m0 * G)) if first else 0.0}


def landing_screen(eng, ascent_prop=0.025, casing=0.035):
    mot = bt.Motor(eng)
    m_desc = M_PAD - ascent_prop
    m_l = m_desc - casing
    res = {"tw_avg": mot.total_impulse / mot.burn_time / (m_l * G),
           "dv_spare": mot.total_impulse / m_l - G * mot.burn_time, "apogees": {}}
    for A in range(8, 33):
        # freefall from apogee A; at each height h, fire now -> impact speed
        best = 1e9
        window = 0.0
        h, v = float(A), 0.0
        dt = 0.004
        while h > 0.3:
            v -= (G - 0.5 * RHO * CDA * v * v / m_desc) * dt
            h += v * dt
            vi, _, _ = bt_sim_burn(mot, m_desc, casing, h, -v)
            best = min(best, abs(vi))
            if abs(vi) <= 2.5:
                window += abs(v) * dt
        res["apogees"][A] = {"best_vi": best, "window_m": window}
    return res


def bt_sim_burn(motor, m_descent, eject_kg, h0, v_down, dt=0.004):
    """burn_table.sim_burn with the casing pushed out at thrust onset."""
    h, v, t = h0, -abs(v_down), 0.0
    while t < DELAY + motor.burn_time + 30.0:
        tb = t - DELAY
        T = motor.thrust(tb)
        m = m_descent - motor.mass_burned(tb) - (eject_kg if tb >= 0.0 else 0.0)
        a = T / m - G - 0.5 * RHO * CDA * v * abs(v) / m
        v += a * dt
        h += v * dt
        t += dt
        if h <= 0.0:
            return -v, t, 0.0
    return -v, t, 0.0


def _land_job(args):
    key, eng = args
    return key, landing_screen(eng)


if __name__ == "__main__":
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    os.makedirs(OUT_DIR, exist_ok=True)
    cands = load_candidates()
    for m in cands:
        m["eng"] = write_eng(m)
    print("candidates after the filter:", len(cands))
    for m in cands:
        m["ascent"] = ascent_screen(m["eng"])
    asc = [m for m in cands if (m["plugged"] or m["reload"])
           and 8.0 <= m["ascent"]["apogee"] <= 32.0 and m["ascent"]["tw_avg"] >= 1.15]
    asc.sort(key=lambda m: m["ascent"]["apogee"])
    print("\nASCENT candidates (1.4 kg, apogee 8-32 m, avg T/W >= 1.15):")
    print("  %-26s %4s %6s %5s %6s %6s %6s %5s %s" % ("motor", "dia", "N*s", "burn", "T/W", "T/W.3", "apogee", "coast", "plug"))
    for m in asc:
        a = m["ascent"]
        print("  %-26s %4.0f %6.1f %5.2f %6.2f %6.2f %6.1f %5.2f %s" % (
            m["key"], m["dia"], m["impulse"], m["burn"], a["tw_avg"], a["tw_first03"],
            a["apogee"], a["coast_s"], "P" if m["plugged"] else "plug?"))
    land_c = [m for m in cands if 0.9 <= m["impulse"] / m["burn"] / ((M_PAD - 0.06) * G) <= 2.2]
    with multiprocessing.Pool(19) as pool:
        lres = dict(pool.map(_land_job, [(m["key"], m["eng"]) for m in land_c]))
    for m in land_c:
        m["landing"] = lres[m["key"]]
    print("\nLANDING candidates (descent ~1.34 kg after the casing): best |v_impact| with perfect timing / soft window (m of fire height)")
    hdr = "  %-26s %5s %6s" % ("motor", "T/W", "dv") + "".join("  A=%-2d     " % A for A in (12, 16, 20, 24, 28))
    print(hdr)
    land_c.sort(key=lambda m: m["landing"]["tw_avg"])
    for m in land_c:
        L = m["landing"]
        cells = "".join("  %4.1f/%4.2f" % (L["apogees"][A]["best_vi"], L["apogees"][A]["window_m"])
                        for A in (12, 16, 20, 24, 28))
        tag = "P" if m["plugged"] else ("-0" if m["booster"] else "plug?")
        print("  %-26s %5.2f %6.1f%s  %s" % (m["key"], L["tw_avg"], L["dv_spare"], cells, tag))
    for m in cands:
        m.pop("points", None)
    json.dump(cands, open(os.path.join(HERE, "prescreen.json"), "w"), indent=1)
