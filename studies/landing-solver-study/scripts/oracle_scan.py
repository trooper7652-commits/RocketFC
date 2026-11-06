"""Headroom ("oracle") analysis -- ANALYSIS ONLY, never part of any solver.

For every powered landing and every MISSED_WINDOW flight of the baseline dev
run, re-fly the identical flight (same dispersions, same sensor-noise seed) on
the ws_oracle DLL with the landing fire command forced at a grid of times,
and record the outcome. Everything before the forced fire is bit-identical to
the baseline flight, so the per-time fire state (true h, v, tilt; KF h, v)
also samples the baseline's own pre-fire trajectory.

Pass 1: 20 ms grid over [t_nat-1.2, t_nat+0.3] (powered) or
        [t_abort-2.0, t_abort] (missed window).
Pass 2: 2 ms grid +-20 ms around (a) the best coarse point and (b) the
        hop boundary (latest coarse fire time that still hopped).
Output: data/oracle_curves.json  {key: [{t, td, hop, ...}, ...]}
"""
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
L = os.path.dirname(HERE)
WS = os.path.join(L, "ws_oracle", "RocketFC")
sys.path.insert(0, HERE)
import metrics as M  # noqa: E402


def run(reqs, tag):
    req = os.path.join(L, "data", f"oracle_{tag}.req.json")
    out = os.path.join(L, "data", f"oracle_{tag}.json")
    with open(req, "w", encoding="utf-8") as f:
        json.dump(reqs, f)
    subprocess.run([sys.executable, os.path.join(HERE, "mc.py"), "--ws", WS,
                    "--n", "200", "--oracle-ms", "@" + req, "--out", out],
                   check=True)
    return M.load(out)


def ms_grid(t0, t1, step_ms):
    a = int(round(t0 * 1000.0 / 2.0)) * 2
    b = int(round(t1 * 1000.0 / 2.0)) * 2
    return list(range(a, b + 1, step_ms))


def main():
    base = M.load(os.path.join(L, "data", sys.argv[1] if len(sys.argv) > 1
                               else "base_dev.json"))
    targets = []
    for r in base:
        s = r["summary"]
        if M.is_powered(s):
            targets.append((r["seed"], r["i"], ms_grid(s["fire_t"] - 1.2,
                                                      s["fire_t"] + 0.3, 20)))
        elif s.get("abort_reason") == "MISSED_WINDOW":
            targets.append((r["seed"], r["i"], ms_grid(s["abort_t"] - 2.0,
                                                      s["abort_t"], 20)))
    reqs = [[sd, i, ms] for sd, i, g in targets for ms in g]
    print(f"pass 1: {len(targets)} flights, {len(reqs)} forced flights", flush=True)
    rows1 = run(reqs, "pass1")

    curves = {}
    for r in rows1:
        k = f"{r['seed']}:{r['i']}"
        s = r["summary"]
        curves.setdefault(k, []).append(pt(r["dispersions"]["oracle_fire_ms"], s))

    reqs2 = []
    for k, pts in curves.items():
        sd, i = (int(x) for x in k.split(":"))
        ok = [p for p in pts if p["td"] is not None]
        if not ok:
            continue
        best = min(ok, key=lambda p: p["td"])
        centers = {best["ms"]}
        hopped = [p for p in ok if p["hop"]]
        if hopped:
            centers.add(max(p["ms"] for p in hopped))
        have = {p["ms"] for p in pts}
        for c in centers:
            for ms in range(c - 20, c + 21, 2):
                if ms not in have:
                    have.add(ms)
                    reqs2.append([sd, i, ms])
    print(f"pass 2: {len(reqs2)} forced flights", flush=True)
    rows2 = run(reqs2, "pass2")
    for r in rows2:
        k = f"{r['seed']}:{r['i']}"
        curves[k].append(pt(r["dispersions"]["oracle_fire_ms"], r["summary"]))
    for k in curves:
        curves[k].sort(key=lambda p: p["ms"])
    with open(os.path.join(L, "data", "oracle_curves.json"), "w",
              encoding="utf-8") as f:
        json.dump(curves, f)
    print("wrote oracle_curves.json:", len(curves), "flights", flush=True)


def pt(ms, s):
    return {"ms": ms, "fire_t": s.get("fire_t"),
            "td": s.get("touchdown_speed_mps") if not s.get("abort_reason") else None,
            "abort": s.get("abort_reason"), "hop": s.get("hop"),
            "hop_apex": s.get("hop_apex_h"),
            "h": s.get("fire_h_true"), "v": s.get("fire_v_true"),
            "kh": s.get("fire_kf_alt"), "kv": s.get("fire_kf_vel"),
            "tilt": s.get("fire_tilt_deg"), "hs": s.get("touchdown_hspeed_mps")}


if __name__ == "__main__":
    main()
