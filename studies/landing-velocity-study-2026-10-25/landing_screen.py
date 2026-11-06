"""Part 2 step 1: 1-D screen of landing motors for robustness to per-flight errors.

For each legal 24/29 mm motor and fall speed at the fire command, simulate the
burn once per random error draw (delay +-5 % of 620 ms, thrust +-3 % uniform
(sigma 1.7 %), mass +-1 %, CdA +-20 %), all draws vectorised with numpy. Because
nothing depends on absolute height until the ground, one trajectory per draw
gives the contact for every fire height at once. A flight "lands" in the proxy
if the first contact is under 3 m/s AND the motor isn't still pushing harder
than the weight then (that would lift it back off). The best fire height is the
one with the most landings; the F15 is the reference.
"""
import math
import os
import sqlite3
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "ws", "base", "tools"))
import burn_table as bt  # noqa: E402

DB = r"C:\Users\troop\OneDrive\Documents\Inventor\SpaceX Rocket\motor-study-2026-10-24\mdb\motors.db"
G, RHO, CDA, DELAY = 9.80665, 1.225, 0.004, 0.620
F15_TOTAL = 0.102
M_DESC_NOW = bt.config_value("MASS_DESCENT_KG", os.path.join(HERE, "ws", "base", "src", "config.h"))
BASE = M_DESC_NOW - F15_TOTAL          # descent mass without the landing motor
ADAPTER = 0.010
N_DRAW, CRASH = 200, 3.0
DT = 0.004
RNG = np.random.default_rng(12345)
DRAWS = {k: RNG.uniform(-1, 1, N_DRAW) for k in ("delay", "thrust", "mass", "cda")}


def curve(m):
    ts = np.array([p[0] for p in m.points])
    fs = np.array([p[1] for p in m.points])
    cum = np.array(m.cum)
    return ts, fs, cum


def screen(motor, m_desc, v_list):
    """-> list of (v_fire, best_h, success_frac, p90_impact) for each fall speed."""
    ts, fs, cum = curve(motor)
    I, prop = motor.total_impulse, motor.prop_kg
    nv = len(v_list)
    # draws x speeds, flattened
    dly = np.tile(DELAY * (1 + 0.05 * DRAWS["delay"]), nv)
    thr = np.tile(1 + 0.03 * DRAWS["thrust"], nv)
    mas = np.tile(m_desc * (1 + 0.01 * DRAWS["mass"]), nv)
    cda = np.tile(CDA * (1 + 0.20 * DRAWS["cda"]), nv)
    v = -np.repeat(np.asarray(v_list, float), N_DRAW)        # up-positive
    h = np.zeros_like(v)
    t_end = DELAY * 1.06 + motor.burn_time + 6.0
    steps = int(t_end / DT)
    H = np.empty((steps, v.size)); V = np.empty_like(H); Tw = np.empty_like(H)
    for k in range(steps):
        t = k * DT
        tb = t - dly
        T = np.interp(tb, ts, fs, left=0.0, right=0.0) * thr
        m = mas - prop * np.interp(tb, ts, cum, left=0.0, right=I) / I
        a = T / m - G - 0.5 * RHO * cda * v * np.abs(v) / m
        v = v + a * DT
        h = h + v * DT
        H[k], V[k], Tw[k] = h, v, T / (m * G)
    # Deepest drop so far, per draw: never rises, so the first contact for a fire
    # height h0 is where it first reaches -h0 -- a binary search per column.
    neg = -np.minimum.accumulate(H, axis=0)
    heights = np.arange(0.5, 120.0, 0.25)
    ncol = v.size
    idx = np.empty((heights.size, ncol), dtype=np.int64)
    for c in range(ncol):
        idx[:, c] = np.searchsorted(neg[:, c], heights, side="left")
    hit = idx < steps
    idxc = np.minimum(idx, steps - 1)
    cols = np.arange(ncol)
    vi = np.abs(V[idxc, cols])
    push = Tw[idxc, cols] > 1.0                  # still out-pushing the weight at contact
    ok = hit & (vi < CRASH) & ~push
    out = []
    for j, vf in enumerate(v_list):
        blk = slice(j * N_DRAW, (j + 1) * N_DRAW)
        frac = ok[:, blk].mean(axis=1)
        b = int(np.argmax(frac))
        vb = np.where(hit[b, blk], vi[b, blk], 99.0)
        out.append((vf, float(heights[b]), float(frac[b]), float(np.percentile(vb, 90))))
    return out


def fall_distance(v_target, m):
    """Height lost falling from rest (apogee) until falling at v_target (with drag)."""
    v = d = 0.0
    while v < v_target:
        a = G - 0.5 * RHO * CDA * v * v / m
        if a <= 1e-6:
            return float("inf")
        v += a * 0.002
        d += v * 0.002
    return d


def eng_from_db(con, mid, path):
    rank = {"cert": 0, "mfr": 1, "user": 2, None: 3}
    cid, src = min(con.execute("select id, source from thrust_curves where motor_id=?", (mid,)),
                   key=lambda r: rank.get(r[1], 3))
    pts = [p for p in con.execute("select time_seconds, force_newtons from thrust_data where curve_id=? "
                                  "order by time_seconds", (cid,)) if p[0] > 0 or p[1] > 0]
    if pts[-1][1] != 0.0:
        pts.append((pts[-1][0] + 0.001, 0.0))
    return pts, src


def main():
    con = sqlite3.connect("file:" + DB + "?mode=ro", uri=True)
    rows = con.execute("""select m.id, m.designation, f.name, m.type, m.diameter, m.length, m.total_impulse,
                                 m.avg_thrust, m.total_weight, m.propellant_weight, m.delays, m.sparky
                          from motors m join manufacturers f on f.id = m.manufacturer_id
                          where (abs(m.diameter-29)<0.6 or abs(m.diameter-24)<0.6) and m.type != 'hybrid'
                            and m.total_impulse <= 160 and coalesce(m.avg_thrust,0) <= 80""").fetchall()
    mdir = os.path.join(HERE, "motors"); os.makedirs(mdir, exist_ok=True)
    v_list = list(range(5, 26))
    results = []
    f15 = bt.Motor(os.path.join(HERE, "ws", "base", "tools", "motors", "Estes_F15.eng"))
    cands = [("F15 (today)", "Estes", "SU", 29, 114, f15, F15_TOTAL, "0,2,4,6,8", "ref")]
    for mid, des, mfr, typ, dia, ln, ti, avg, tw, pw, dl, sp in rows:
        if tw is None or pw is None or sp:
            continue
        tw = tw / 1000 if tw > 5 else tw; pw = pw / 1000 if pw > 5 else pw
        if pw > 0.0625 or pw + 0.040 > 0.125:
            continue
        pts, src = eng_from_db(con, mid, None)
        path = os.path.join(mdir, "%d.eng" % mid)
        with open(path, "w", encoding="ascii", errors="replace", newline="\n") as f:
            f.write("%s %d %d P %.4f %.4f X\n" % (des.replace(" ", "_"), round(dia), round(ln or 0), pw, tw))
            for t, F in pts:
                f.write("%.4f %.4f\n" % (t, F))
        m_desc = BASE + tw + (ADAPTER if dia < 26 else 0.0)
        tw_ratio = (avg or 0) / (m_desc * G)
        if tw_ratio < 0.9 or tw_ratio > 4.0:
            continue
        cands.append(("%s %s" % (mfr.split()[0], des), mfr, typ, dia, ln, bt.Motor(path), tw, dl or "", src))
    print("screening %d motors" % len(cands), flush=True)
    for name, mfr, typ, dia, ln, motor, tw, dl, src in cands:
        m_desc = BASE + tw + (ADAPTER if dia < 26 else 0.0)
        res = screen(motor, m_desc, v_list)
        vf, h0, frac, p90 = max(res, key=lambda r: r[2])
        tail = motor.points[-3][1] if len(motor.points) >= 3 else 0.0
        noej = any(x.strip() in ("0", "P") for x in str(dl).replace(";", ",").split(","))
        apo = h0 + fall_distance(vf, m_desc)
        results.append((frac, name, typ, dia, ln, motor.total_impulse, motor.burn_time,
                        motor.total_impulse / motor.burn_time / (m_desc * G), vf, h0, apo, p90,
                        m_desc, noej, src, dl))
        print("  %-26s land %5.1f %% at fall %2d m/s (fire %.1f m, apogee ~%.0f m)" % (name, 100 * frac, vf, h0, apo),
              flush=True)
    results.sort(reverse=True)
    print("\n| Motor | type | mm | len | N*s | burn s | avg T/W | 1-D land % | fall @ fire | apogee needed | p90 impact | no-ejection option | delays |")
    print("|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for r in results[:25]:
        frac, name, typ, dia, ln, ti, bt_s, twr, vf, h0, apo, p90, md, noej, src, dl = r
        print("| %s | %s | %.0f | %s | %.1f | %.2f | %.2f | **%.1f** | %d m/s | %.0f m | %.1f m/s | %s | %s |" % (
            name, typ, dia, ln, ti, bt_s, twr, 100 * frac, vf, apo, p90, "yes" if noej else "no", dl))


if __name__ == "__main__":
    main()
