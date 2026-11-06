#!/usr/bin/env python3
"""Generate the risk-aware landing table (src/core/landing_table.h).

burn_table.py answers "at what altitude must I fire so a NOMINAL rocket stops
exactly at the ground?". Real flights aren't nominal: the ignition delay, the
descent mass and the motor's thrust differ from flight to flight, and none of
them can be seen before ignition. Aiming exactly at zero speed is also
lopsided: fire a little early and the rocket stops in mid-air, climbs on the
remaining thrust and falls back hard (a "hop"); fire a little late and it
lands a bit fast.

So for each descent speed this tool simulates thousands of plausible rockets
(drawn from the uncertainty you specify), each one all the way to the ground
-- including hop, climb, burnout and fall -- and picks the fire altitude with
the lowest EXPECTED touchdown speed (optionally with an extra penalty per hop).
Altitude never enters the physics, so one simulation per rocket scores every
candidate altitude at once (touchdown = the first moment it has fallen that
far).

It also writes, per row, how much farther a rocket falls when it lights up
tilted (thrust is along the body, so only cos(tilt) of it slows the fall
until the TVC straightens the vehicle), as metres per rad^2 of tilt.

The uncertainty widths are [MEASURE] inputs: the defaults reproduce the
closed-loop simulator's Monte-Carlo dispersions plus a modest thrust spread.
Re-run after measuring mass (scale), ignition delay (ThrustStand) and thrust.

Usage:
    python tools/landing_table.py                    # defaults from src/config.h
    python tools/landing_table.py --delay-pct 5 --mass-pct 1

Needs numpy.
"""
import argparse
import datetime
import multiprocessing
import os
import re

import numpy as np

G = 9.80665
RHO = 1.225


class Curve:
    """RASP .eng thrust curve on a dense grid for vectorized lookup."""

    def __init__(self, path):
        pts = [(0.0, 0.0)]
        header = None
        with open(path, encoding="utf-8", errors="replace") as f:
            for line in f:
                s = line.strip()
                if not s or s.startswith(";"):
                    continue
                if header is None:
                    header = s.split()
                    self.name = header[0]
                    self.prop = float(header[4])
                    continue
                t, F = s.split()[:2]
                pts.append((float(t), float(F)))
        pts.sort()
        t = np.array([p[0] for p in pts])
        f = np.array([p[1] for p in pts])
        self.burn = float(t[-1])
        self.dt = 0.0005
        self.grid = np.arange(0.0, self.burn + self.dt, self.dt)
        self.F = np.interp(self.grid, t, f, left=0.0, right=0.0)
        self.I = np.concatenate([[0.0], np.cumsum(0.5 * (self.F[1:] + self.F[:-1]) * self.dt)])
        self.Itot = float(self.I[-1])

    def thrust(self, tau):
        return np.interp(tau, self.grid, self.F, left=0.0, right=0.0)

    def burned_frac(self, tau):
        return np.interp(tau, self.grid, self.I, left=0.0, right=self.Itot) / self.Itot


def simulate(curve, speed, delay, mass, cda, tscale, theta_on=None, tau_r=0.15,
             dt=0.002, tmax=14.0):
    """Vertical flight from the fire command for J rockets at once (RK4).
    Returns X (fall displacement, +up) and V (+up), float32 [steps, J]."""
    J = len(speed)
    x = np.zeros(J)
    v = -np.abs(speed).astype(float)
    k = 0.5 * RHO * cda
    n = int(tmax / dt) + 1
    X = np.empty((n, J), dtype=np.float32)
    V = np.empty((n, J), dtype=np.float32)

    def acc(t, vv):
        tau = t - delay
        m = mass - curve.prop * curve.burned_frac(tau)
        T = tscale * curve.thrust(tau)
        if theta_on is not None:
            T = T * np.cos(theta_on * np.exp(-np.maximum(tau, 0.0) / tau_r))
        return (T - k * vv * np.abs(vv)) / m - G

    t = 0.0
    for i in range(n):
        X[i] = x
        V[i] = v
        a1 = acc(t, v)
        v2 = v + 0.5 * dt * a1
        a2 = acc(t + 0.5 * dt, v2)
        v3 = v + 0.5 * dt * a2
        a3 = acc(t + 0.5 * dt, v3)
        v4 = v + dt * a3
        a4 = acc(t + dt, v4)
        x = x + dt * (v + 2 * v2 + 2 * v3 + v4) / 6.0
        v = v + dt * (a1 + 2 * a2 + 2 * a3 + a4) / 6.0
        t += dt
    return X, V


def outcome_curves(X, V, levels, hop_min=0.25):
    """Touchdown speed and hop flag for every rocket and every contact depth
    (m fallen at first ground contact). Returns (speed[J, L], hop[J, L])."""
    n, J = X.shape
    spd = np.full((J, len(levels)), np.nan)
    hop = np.zeros((J, len(levels)), dtype=bool)
    for j in range(J):
        x = X[:, j].astype(float)
        vv = V[:, j].astype(float)
        depth = -np.minimum.accumulate(x)
        idx = np.searchsorted(depth, levels, side="left")
        ok = (idx > 0) & (idx < n)
        i1 = idx[ok]
        i0 = i1 - 1
        d0, d1 = depth[i0], depth[i1]
        f = np.where(d1 > d0, (levels[ok] - d0) / np.maximum(d1 - d0, 1e-12), 1.0)
        spd[j, ok] = np.abs(vv[i0] + f * (vv[i1] - vv[i0]))
        stop = np.argmax(vv >= 0.0)  # first moment it stopped falling
        if vv[stop] >= 0.0:
            hop[j] = levels > depth[stop] + hop_min
    return spd, hop


def stop_depth(curve, speed, delay, mass, cda, theta_on=None, tau_r=0.15,
               slowest=False):
    """Exact zero-uncertainty fall distance until the rocket stops. If it
    can't stop: nan, or with slowest=True the fall distance at its slowest
    moment (the best a timing-only solver can aim at)."""
    X, V = simulate(curve, np.array([speed]), np.array([delay]), np.array([mass]),
                    np.array([cda]), np.array([1.0]),
                    None if theta_on is None else np.array([theta_on]), tau_r,
                    dt=0.0005, tmax=delay + curve.burn + 0.5)
    x, v = X[:, 0].astype(float), V[:, 0].astype(float)
    i = int(np.argmax(v >= 0.0))
    if v[i] < 0.0:
        if not slowest:
            return float("nan")
        j = int(np.argmax(v))  # least negative = slowest fall
        return float(-x[j])
    f = -v[i - 1] / max(v[i] - v[i - 1], 1e-12)
    return float(-(x[i - 1] + f * (x[i] - x[i - 1])))


def solve_row(args):
    (speed, eng, mass, cda, delay, unc, hop_penalty, tilt_ref, tau_r, J, seed,
     h_floor) = args
    curve = Curve(eng)
    rng = np.random.default_rng(seed)
    d = delay * (1.0 + rng.uniform(-unc["delay"], unc["delay"], J))
    m = mass * (1.0 + rng.uniform(-unc["mass"], unc["mass"], J))
    c = cda * (1.0 + rng.uniform(-unc["cda"], unc["cda"], J))
    ts = 1.0 + unc["thrust_sd"] * rng.standard_normal(J)
    eh = unc["eh_mean"] + unc["eh_sd"] * rng.standard_normal(J)
    ev = unc["ev_mean"] + unc["ev_sd"] * rng.standard_normal(J)
    true_speed = np.maximum(speed + ev, 0.0)
    X, V = simulate(curve, true_speed, d, m, c, ts)
    nom = stop_depth(curve, speed, delay, mass, cda)
    ref = nom if np.isfinite(nom) else stop_depth(curve, speed, delay, mass, cda,
                                                  slowest=True)
    # Never aim below the missed-window floor: the flight computer aborts
    # there, so "fire at the last moment" is not an option it can take.
    levels = np.arange(max(h_floor, ref - 10.0), max(h_floor, ref - 10.0) + 20.0, 0.01)
    spd, hop = outcome_curves(X, V, levels)
    # decision H = level the solver believes; the true contact depth is H - eh
    score = np.empty(len(levels))
    hop_rate = np.empty(len(levels))
    for k, H in enumerate(levels):
        true_level = H - eh
        jj = np.clip(np.round((true_level - levels[0]) / 0.01).astype(int), 0, len(levels) - 1)
        s = spd[np.arange(J), jj]
        h = hop[np.arange(J), jj]
        score[k] = np.nanmean(s) + hop_penalty * np.mean(h)
        hop_rate[k] = np.mean(h)
    k = int(np.nanargmin(score))
    H_fire = float(levels[k])
    # tilt coefficient: extra stopping distance per rad^2 of tilt at thrust onset
    tilt_coef = 0.0
    if np.isfinite(nom):
        tilted = stop_depth(curve, speed, delay, mass, cda, theta_on=tilt_ref, tau_r=tau_r)
        if np.isfinite(tilted):
            tilt_coef = (tilted - nom) / (tilt_ref * tilt_ref)
    return speed, H_fire, (nom if np.isfinite(nom) else float("nan")), tilt_coef, \
        float(hop_rate[k]), float(score[k])


def config_defaults(path):
    vals = {"MASS_DESCENT_KG": 0.95, "CDA_M2": 0.004, "IGNITION_DELAY_MS": 500.0}
    try:
        text = open(path, encoding="utf-8").read()
    except OSError:
        return vals
    for name in vals:
        m = re.search(r"constexpr\s+float\s+" + name + r"\s*=\s*([-+\d.eE]+)f?\s*;", text)
        if m:
            vals[name] = float(m.group(1))
    return vals


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    cfg = config_defaults(os.path.join(here, "..", "src", "config.h"))
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--eng", default=os.path.join(here, "motors", "Estes_F15.eng"))
    ap.add_argument("--mass", type=float, default=cfg["MASS_DESCENT_KG"])
    ap.add_argument("--cda", type=float, default=cfg["CDA_M2"])
    ap.add_argument("--delay-ms", type=float, default=cfg["IGNITION_DELAY_MS"])
    ap.add_argument("--delay-pct", type=float, default=15.0,
                    help="[MEASURE] ignition-delay spread, +- percent (uniform)")
    ap.add_argument("--mass-pct", type=float, default=5.0,
                    help="[MEASURE] descent-mass uncertainty, +- percent (uniform)")
    ap.add_argument("--cda-pct", type=float, default=20.0,
                    help="[MEASURE] drag uncertainty, +- percent (uniform)")
    ap.add_argument("--thrust-sd-pct", type=float, default=1.7,
                    help="[MEASURE] motor-to-motor thrust spread, 1-sigma percent")
    ap.add_argument("--eh-mean", type=float, default=0.0,
                    help="estimator altitude error left after lag compensation (m)")
    ap.add_argument("--eh-sd", type=float, default=0.06)
    ap.add_argument("--ev-mean", type=float, default=0.0,
                    help="estimator descent-speed error left after lag compensation (m/s)")
    ap.add_argument("--ev-sd", type=float, default=0.04)
    ap.add_argument("--hop-penalty", type=float, default=0.0,
                    help="extra cost per hop, m/s (0 = plain expected touchdown speed)")
    ap.add_argument("--tilt-ref-deg", type=float, default=15.0)
    ap.add_argument("--tilt-tau", type=float, default=0.15,
                    help="[ESTIMATE] TVC tilt recovery time constant after thrust onset (s)")
    ap.add_argument("--h-floor", type=float, default=4.5,
                    help="lowest fire altitude allowed (m): above MISSED_WINDOW_ALT_M")
    ap.add_argument("--vmin", type=float, default=2.0)
    ap.add_argument("--vmax", type=float, default=25.0)
    ap.add_argument("--vstep", type=float, default=0.5)
    ap.add_argument("--draws", type=int, default=2000)
    ap.add_argument("--out", default=os.path.join(here, "..", "src", "core", "landing_table.h"))
    args = ap.parse_args()

    unc = {"delay": args.delay_pct / 100.0, "mass": args.mass_pct / 100.0,
           "cda": args.cda_pct / 100.0, "thrust_sd": args.thrust_sd_pct / 100.0,
           "eh_mean": args.eh_mean, "eh_sd": args.eh_sd,
           "ev_mean": args.ev_mean, "ev_sd": args.ev_sd}
    n = int(round((args.vmax - args.vmin) / args.vstep)) + 1
    speeds = [args.vmin + i * args.vstep for i in range(n)]
    jobs = [(v, args.eng, args.mass, args.cda, args.delay_ms / 1000.0, unc,
             args.hop_penalty, args.tilt_ref_deg * np.pi / 180.0, args.tilt_tau,
             args.draws, 1000 + i, args.h_floor) for i, v in enumerate(speeds)]
    with multiprocessing.Pool(min(len(jobs), max(1, (os.cpu_count() or 2) - 1))) as pool:
        res = pool.map(solve_row, jobs)

    print("  v_down   h_fire   h_nominal  late_by  tilt_coef  hop_rate  E[v_td]")
    for v, hf, hn, tc, hr, sc in res:
        print(f"  {v:6.2f}  {hf:7.3f}  {hn:9.3f}  {hn - hf:+7.3f}  {tc:9.3f}  {hr:8.3f}  {sc:7.2f}")

    def rows(vals, fmt):
        out = []
        for i in range(0, len(vals), 8):
            out.append("  " + " ".join(fmt.format(x) for x in vals[i:i + 8]))
        return "\n".join(out)

    hf = [r[1] for r in res]
    hn = [r[2] if np.isfinite(r[2]) else -1.0 for r in res]
    tc = [r[3] for r in res]
    stamp = datetime.date.today().isoformat()
    with open(os.path.abspath(args.out), "w", encoding="utf-8", newline="\n") as f:
        f.write(f"""#pragma once
// GENERATED by tools/landing_table.py on {stamp} -- DO NOT EDIT BY HAND.
//   motor {os.path.basename(args.eng)}, mass {args.mass:.3f} kg, CdA {args.cda:.4f} m^2,
//   ignition delay {args.delay_ms:.0f} ms
//   uncertainty: delay +-{args.delay_pct:g} %, mass +-{args.mass_pct:g} %, CdA +-{args.cda_pct:g} %,
//   thrust sd {args.thrust_sd_pct:g} %, estimator residual h {args.eh_mean:+.2f}+-{args.eh_sd:.2f} m,
//   v {args.ev_mean:+.2f}+-{args.ev_sd:.2f} m/s; hop penalty {args.hop_penalty:g} m/s;
//   {args.draws} draws per row; tilt recovery tau {args.tilt_tau:g} s
// Per descent speed (m/s, downward-positive):
//   H_FIRE    fire altitude (m) minimizing the expected touchdown speed
//   H_NOMINAL exact stop altitude of the nominal rocket (-1 = can't stop)
//   TILT_COEF extra fall distance per rad^2 of tilt at thrust onset (m/rad^2)
namespace landingtable {{

constexpr float V_MIN = {args.vmin}f;
constexpr float V_STEP = {args.vstep}f;
constexpr int N = {n};

constexpr float H_FIRE[N] = {{
{rows(hf, "{:8.3f}f,")}
}};

constexpr float H_NOMINAL[N] = {{
{rows(hn, "{:8.3f}f,")}
}};

constexpr float TILT_COEF[N] = {{
{rows(tc, "{:8.3f}f,")}
}};

// Linear interpolation in a table, clamped to its domain.
inline float lookup(const float* t, float vDown) {{
  if (!(vDown > V_MIN)) return t[0];  // also catches NaN
  const float idx = (vDown - V_MIN) / V_STEP;
  if (idx >= (float)(N - 1)) return t[N - 1];
  const int i = (int)idx;
  const float f = idx - (float)i;
  return t[i] * (1.0f - f) + t[i + 1] * f;
}}
inline float hFire(float vDown) {{ return lookup(H_FIRE, vDown); }}
inline float tiltCoef(float vDown) {{ return lookup(TILT_COEF, vDown); }}

}}  // namespace landingtable
""")
    print("wrote", os.path.abspath(args.out))


if __name__ == "__main__":
    main()
