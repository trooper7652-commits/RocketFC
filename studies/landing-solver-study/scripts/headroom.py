"""Headroom analysis of the oracle fire-time scans (ANALYSIS ONLY).

For each baseline powered landing / missed window, using the forced-fire
curves (touchdown speed vs. fire time; the pre-fire trajectory is identical to
the baseline flight, so (h, v, tilt, KF h, KF v) at each scanned time are the
baseline flight's own values):

  natural        : what the current table did
  table+truth    : the current table fed the TRUE state (= a perfect lag fix)
  table+truth+dh : same with a constant altitude margin dh (best dh over all
                   flights = best possible "perfect estimator + fixed margin")
  KF window      : first time the current table on KF state says fire,
                   ignoring tilt (= what relaxing the tilt interlock buys)
  oracle_ilock   : best fire time among times with tilt < 15 deg
  oracle_free    : best fire time, no constraint (perfect knowledge)
"""
import json
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
L = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import metrics as M  # noqa: E402


def load_table(path):
    s = open(path, encoding="utf-8", errors="replace").read()
    vmin = float(re.search(r"V_MIN = ([\d.]+)f", s).group(1))
    vstep = float(re.search(r"V_STEP = ([\d.]+)f", s).group(1))
    body = s[s.index("H_IGNITE[N] = {"):]
    body = body[:body.index("};")]
    vals = np.array([float(x) for x in re.findall(r"(-?\d+\.\d+)f", body)])

    def hi(v):
        v = np.asarray(v, dtype=float)
        idx = np.clip((v - vmin) / vstep, 0, len(vals) - 1)
        return np.interp(idx, np.arange(len(vals)), vals)
    return hi


def td_at(pts, t):
    """touchdown speed if fired at time t (linear interp between scanned
    points; a point that aborted / never landed counts as nan)."""
    ts = np.array([p["fire_t"] if p["fire_t"] is not None else np.nan for p in pts])
    td = np.array([p["td"] if p["td"] is not None else np.nan for p in pts])
    ok = ~np.isnan(ts)
    ts, td = ts[ok], td[ok]
    if t is None or len(ts) == 0 or t < ts[0] or t > ts[-1]:
        return np.nan
    i = int(np.searchsorted(ts, t))
    if i == 0:
        return float(td[0])
    t0, t1 = ts[i - 1], ts[i]
    if np.isnan(td[i - 1]) or np.isnan(td[i]):
        return float(td[i])
    # the cliff makes interpolation across a hop boundary meaningless: take
    # the nearer sample instead of blending
    hop = [pts[k]["hop"] for k in range(len(pts)) if ok[k]]
    if hop[i - 1] != hop[i]:
        return float(td[i - 1] if (t - t0) < (t1 - t) else td[i])
    f = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
    return float(td[i - 1] + f * (td[i] - td[i - 1]))


def first_cross(pts, cond):
    """earliest scanned fire time where cond(point) holds (pre-fire state is
    the baseline's own trajectory); linear refinement between samples."""
    prev = None
    for p in pts:
        if p["fire_t"] is None or p["h"] is None:
            continue
        c = cond(p)
        if c is None:
            prev = p
            continue
        if c <= 0:
            if prev is None:
                return p["fire_t"]
            cp = cond(prev)
            if cp is None or cp <= 0:
                return p["fire_t"]
            f = cp / (cp - c)
            return prev["fire_t"] + f * (p["fire_t"] - prev["fire_t"])
        prev = p
    return None


def main():
    hi = load_table(os.path.join(L, "ws_base", "RocketFC", "src", "core", "burn_table.h"))
    base = {f'{r["seed"]}:{r["i"]}': r["summary"] for r in M.load(os.path.join(L, "data", "base_dev.json"))}
    curves = json.load(open(os.path.join(L, "data", "oracle_curves.json")))
    rows = []
    for k, pts in curves.items():
        s = base[k]
        pts = [p for p in pts if p["fire_t"] is not None]
        if not pts:
            continue
        good = [p for p in pts if p["td"] is not None]
        if not good:
            continue
        best = min(good, key=lambda p: p["td"])
        il = [p for p in good if p["tilt"] is not None and p["tilt"] < 15.0]
        best_il = min(il, key=lambda p: p["td"]) if il else None
        t_truth = first_cross(pts, lambda p: p["h"] - hi(-p["v"]) if -p["v"] >= 2.0 else None)
        t_kfwin = first_cross(pts, lambda p: p["kh"] - hi(-p["kv"]) if -p["kv"] >= 2.0 else None)
        soft = [p for p in good if p["td"] < 2.5]
        rows.append({
            "key": k, "powered": M.is_powered(s), "abort": s.get("abort_reason"),
            "tilt_limited": bool(s.get("fire_tilt_prev_deg") is not None and s["fire_tilt_prev_deg"] >= 15.0),
            "natural": s["touchdown_speed_mps"] if M.is_powered(s) else np.nan,
            "natural_hop": bool(s.get("hop")),
            "t_nat": s.get("fire_t"),
            "truth": td_at(pts, t_truth), "t_truth": t_truth,
            "kfwin": td_at(pts, t_kfwin), "t_kfwin": t_kfwin,
            "oracle_free": best["td"], "t_best": best["fire_t"], "best_hop": best["hop"],
            "best_tilt": best["tilt"],
            "oracle_ilock": best_il["td"] if best_il else np.nan,
            "soft_window_ms": 2.0 * len([p for p in soft]) if False else None,
            "pts": pts,
        })
    # soft window width (ms) using the 2 ms fine grid where available: measure
    # contiguous span of fire times with td < 2.5 m/s (sum of sample spacings)
    for r in rows:
        ts = np.array([p["fire_t"] for p in r["pts"] if p["td"] is not None])
        td = np.array([p["td"] for p in r["pts"] if p["td"] is not None])
        w = 0.0
        for i in range(1, len(ts)):
            if td[i] < 2.5 and td[i - 1] < 2.5:
                w += ts[i] - ts[i - 1]
        r["soft_window_ms"] = 1000.0 * w

    def summ(name, vals):
        v = np.array([x for x in vals if x is not None and not np.isnan(x)])
        return (f"{name:34s} n={len(v):3d} median {np.median(v):5.2f}  mean {np.mean(v):5.2f}  "
                f"p90 {np.percentile(v, 90):5.2f}  soft<2.5 {int(np.sum(v < 2.5)):3d}")

    pw = [r for r in rows if r["powered"]]
    out = []
    out.append(f"flights analysed: {len(rows)} ({len(pw)} powered landings, "
               f"{len(rows) - len(pw)} missed windows)")
    for label, sel in (("ALL powered", pw),
                       ("not tilt-limited", [r for r in pw if not r["tilt_limited"]]),
                       ("tilt-limited", [r for r in pw if r["tilt_limited"]])):
        out.append(f"\n== {label} ({len(sel)}) ==")
        out.append(summ("natural (current table on KF)", [r["natural"] for r in sel]))
        out.append(summ("table on TRUE state (perfect lag fix)", [r["truth"] for r in sel]))
        out.append(summ("table on KF, tilt interlock ignored", [r["kfwin"] for r in sel]))
        out.append(summ("best fire time with tilt<15 (oracle)", [r["oracle_ilock"] for r in sel]))
        out.append(summ("best fire time, no constraint (oracle)", [r["oracle_free"] for r in sel]))
    mw = [r for r in rows if not r["powered"]]
    if mw:
        out.append(f"\n== missed windows ({len(mw)}) ==")
        out.append(summ("table on KF, tilt ignored", [r["kfwin"] for r in mw]))
        out.append(summ("best fire time, no constraint", [r["oracle_free"] for r in mw]))
    sw = np.array([r["soft_window_ms"] for r in pw])
    out.append(f"\nsoft-landing window width (fire times giving < 2.5 m/s), powered flights: "
               f"zero for {int(np.sum(sw == 0))}/{len(sw)}; median of non-zero {np.median(sw[sw > 0]) if np.any(sw > 0) else 0:.0f} ms")
    off = np.array([r["t_best"] - r["t_nat"] for r in pw])
    out.append(f"best fire time minus natural fire time (powered): median {1000*np.median(off):+.0f} ms, "
               f"5-95% [{1000*np.percentile(off,5):+.0f}, {1000*np.percentile(off,95):+.0f}] ms")
    off2 = np.array([r["t_best"] - r["t_truth"] for r in pw if r["t_truth"] is not None])
    out.append(f"best fire time minus table-on-truth time: median {1000*np.median(off2):+.0f} ms, "
               f"5-95% [{1000*np.percentile(off2,5):+.0f}, {1000*np.percentile(off2,95):+.0f}] ms "
               f"(spread = what no timing rule without per-flight knowledge can remove)")
    # best constant altitude margin on top of the truth-fed table
    best_dh = None
    for dh in np.arange(-1.0, 3.01, 0.1):
        vals = []
        for r in pw:
            t = first_cross(r["pts"], lambda p, dh=dh: p["h"] - (hi(-p["v"]) + dh) if -p["v"] >= 2.0 else None)
            vals.append(td_at(r["pts"], t))
        v = np.array([x for x in vals if not np.isnan(x)])
        rec = (float(np.mean(v)), float(np.median(v)), float(np.percentile(v, 90)), dh, len(v))
        if best_dh is None or rec[0] < best_dh[0]:
            best_dh = rec
        out.append(f"   truth-fed table + margin {dh:+.1f} m: mean {rec[0]:5.2f} median {rec[1]:5.2f} p90 {rec[2]:5.2f} (n={rec[4]})")
    out.append(f"best constant margin on perfect state: {best_dh[3]:+.1f} m -> mean {best_dh[0]:.2f}, "
               f"median {best_dh[1]:.2f}, p90 {best_dh[2]:.2f}")
    txt = "\n".join(out)
    print(txt)
    with open(os.path.join(L, "data", "headroom.txt"), "w", encoding="utf-8") as f:
        f.write(txt + "\n")
    with open(os.path.join(L, "data", "headroom_rows.json"), "w", encoding="utf-8") as f:
        json.dump([{k: v for k, v in r.items() if k != "pts"} for r in rows], f, default=float)


if __name__ == "__main__":
    main()
