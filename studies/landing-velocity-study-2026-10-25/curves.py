"""Per-fall-speed robustness of the top landing motors, plus which ascent motor
reaches the apogee each one needs (calibrated 1-D apogee model)."""
import math
import os
import sqlite3
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import landing_screen as S  # noqa: E402

PICK = ["AeroTech F10", "AeroTech E12-RCJ", "Ellis E12", "Estes E16", "AeroTech E15W"]


def apogee(motor, m_dry, prop, dt=0.002):
    """Calibrated 1-D ascent apogee (same model as the motor study, x0.995)."""
    import numpy as np  # noqa: F401
    I = motor.total_impulse; h = v = t = J = 0.0
    while True:
        T = motor.thrust(t); m = m_dry + prop * max(0.0, 1 - J / I)
        a = (T - math.copysign(0.5 * S.RHO * S.CDA * v * v, v)) / m - S.G
        if h <= 0 and a < 0 and v <= 0:
            a = 0.0; v = 0.0
        v += a * dt; h += v * dt; J += T * dt; t += dt
        if (t > motor.burn_time and v <= 0) or t > 60:
            return h * 0.995, h - 0.0


def main():
    con = sqlite3.connect("file:" + S.DB + "?mode=ro", uri=True)
    q = """select m.id, m.designation, f.name, m.diameter, m.total_weight, m.propellant_weight, m.delays, m.burn_time
           from motors m join manufacturers f on f.id=m.manufacturer_id
           where (abs(m.diameter-29)<0.6 or abs(m.diameter-24)<0.6) and m.type != 'hybrid'"""
    allm = con.execute(q).fetchall()
    bym = {"%s %s" % (r[2].split()[0], r[1]): r for r in allm}
    f15 = S.bt.Motor(os.path.join(HERE, "ws", "base", "tools", "motors", "Estes_F15.eng"))
    v_list = list(range(5, 26))
    for name in ["F15 (today)"] + PICK:
        if name == "F15 (today)":
            motor, tw, dia = f15, S.F15_TOTAL, 29
        else:
            mid, des, mfr, dia, tw, pw, dl, bt_s = bym[name]
            tw = tw / 1000 if tw > 5 else tw
            motor = S.bt.Motor(os.path.join(HERE, "motors", "%d.eng" % mid))
        m_desc = S.BASE + tw + (S.ADAPTER if dia < 26 else 0.0)
        res = S.screen(motor, m_desc, v_list)
        line = " ".join("%d:%2.0f" % (r[0], 100 * r[2]) for r in res)
        best = max(res, key=lambda r: r[2])
        need = best[1] + S.fall_distance(best[0], m_desc)
        print("%-18s desc %.3f kg | land%% by fall speed  %s" % (name, m_desc, line))
        # ascent motors (long burn, legal with this landing motor) and their apogees
        prop_land = motor.prop_kg
        opts = []
        for r in allm:
            amid, ades, amfr, adia, atw, apw, adl, abt = r
            if atw is None or apw is None or not abt:
                continue
            atw = atw / 1000 if atw > 5 else atw; apw = apw / 1000 if apw > 5 else apw
            if apw > 0.0625 or apw + prop_land > 0.125 or abt < 2.5:
                continue
            p = os.path.join(HERE, "motors", "%d.eng" % amid)
            if not os.path.exists(p):
                continue
            am = S.bt.Motor(p)
            # descent mass carries the spent ascent casing: replace the F10 casing (43 g) in BASE
            m_d = m_desc - 0.043 + (atw - apw) + (S.ADAPTER if adia < 26 else 0.0)
            ap, _ = apogee(am, m_d, apw)
            opts.append((abs(ap - need), "%s %s" % (amfr.split()[0], ades), ap, abt, adl))
        opts.sort()
        print("    best fall %d m/s -> apogee needed ~%.0f m; closest long-burn ascents: %s" % (
            best[0], need, "; ".join("%s %.0f m (%.1f s burn, delays %s)" % (n, ap, b, d) for _, n, ap, b, d in opts[:4])))


if __name__ == "__main__":
    main()
