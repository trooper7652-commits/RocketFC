"""Pre-registered metrics for the landing-solver study (see notes/03_metrics.md).

Definitions (fixed before any candidate comparison):
  powered landing: fire command issued (fire_t set), plant touched down,
      no abort, no sim error.
  touchdown speed: |vertical velocity| at first ground contact (run.py's
      touchdown_speed_mps).
  p90: numpy.percentile(speeds, 90) with linear interpolation.
  soft: speed < 2.5 m/s (burn_table.py's "GOOD" criterion); also < 1.0 m/s.
  hop: after landing-thrust onset the TRUE vertical velocity reached >= 0
      while TRUE altitude > 0.25 m (run.py's `hop`); severe hop = hop with
      touchdown > 5 m/s.
  aborts: counted by reason over ALL flights.
Paired comparison: flights matched by (seed, i); speed statistics use flights
that are powered landings in BOTH runs; bootstrap resamples those pairs
(10,000 resamples, numpy default_rng(20261012)) and reports 95 % percentile
CIs for d = candidate - baseline of median, p90 and mean.
"""
import json
from collections import Counter

import numpy as np

SOFT = 2.5
VERY_SOFT = 1.0
SEVERE_HOP = 5.0
N_BOOT = 10000
BOOT_SEED = 20261012


def load(path):
    with open(path, encoding="utf-8") as f:
        d = json.load(f)
    return d["rows"] if isinstance(d, dict) and "rows" in d else d


def is_powered(s):
    return ("error" not in s and s.get("fire_t") is not None
            and s.get("touchdown_speed_mps") is not None
            and not s.get("abort_reason"))


def p90(x):
    return float(np.percentile(np.asarray(x, dtype=float), 90))


def stats(rows):
    S = [r["summary"] for r in rows]
    pw = [s for s in S if is_powered(s)]
    v = np.array([s["touchdown_speed_mps"] for s in pw], dtype=float)
    hops = [s for s in pw if s.get("hop")]
    out = {
        "flights": len(S),
        "errors": sum(1 for s in S if "error" in s),
        "powered": len(pw),
        "median": float(np.median(v)) if len(v) else float("nan"),
        "mean": float(np.mean(v)) if len(v) else float("nan"),
        "p90": p90(v) if len(v) else float("nan"),
        "soft": int(np.sum(v < SOFT)),
        "very_soft": int(np.sum(v < VERY_SOFT)),
        "hops": len(hops),
        "severe_hops": sum(1 for s in hops if s["touchdown_speed_mps"] > SEVERE_HOP),
        "aborts": dict(Counter(s["abort_reason"] for s in S
                               if "error" not in s and s.get("abort_reason"))),
        "abort_states": dict(Counter(f'{s["abort_reason"]}@{s.get("abort_state")}'
                                     for s in S if "error" not in s
                                     and s.get("abort_reason"))),
        "hspeed_median": float(np.median([s["touchdown_hspeed_mps"] for s in pw
                                          if s.get("touchdown_hspeed_mps") is not None]))
        if pw else float("nan"),
    }
    return out


def key(r):
    return (r.get("seed"), r.get("i"))


def paired(base_rows, cand_rows):
    b = {key(r): r["summary"] for r in base_rows}
    c = {key(r): r["summary"] for r in cand_rows}
    ks = sorted(k for k in b if k in c and is_powered(b[k]) and is_powered(c[k]))
    xb = np.array([b[k]["touchdown_speed_mps"] for k in ks], dtype=float)
    xc = np.array([c[k]["touchdown_speed_mps"] for k in ks], dtype=float)
    rng = np.random.default_rng(BOOT_SEED)
    n = len(ks)
    idx = rng.integers(0, n, size=(N_BOOT, n))
    bb, cc = xb[idx], xc[idx]
    d_med = np.median(cc, axis=1) - np.median(bb, axis=1)
    d_p90 = np.percentile(cc, 90, axis=1) - np.percentile(bb, 90, axis=1)
    d_mean = cc.mean(axis=1) - bb.mean(axis=1)

    def ci(a):
        return [float(np.percentile(a, 2.5)), float(np.percentile(a, 97.5))]

    only_b = sum(1 for k in b if k in c and is_powered(b[k]) and not is_powered(c[k]))
    only_c = sum(1 for k in b if k in c and is_powered(c[k]) and not is_powered(b[k]))
    return {
        "pairs": n, "powered_only_base": only_b, "powered_only_cand": only_c,
        "d_median": float(np.median(xc) - np.median(xb)), "d_median_ci": ci(d_med),
        "d_p90": p90(xc) - p90(xb), "d_p90_ci": ci(d_p90),
        "d_mean": float(xc.mean() - xb.mean()), "d_mean_ci": ci(d_mean),
        "better": int(np.sum(xc < xb - 1e-9)), "worse": int(np.sum(xc > xb + 1e-9)),
        "same": int(np.sum(np.abs(xc - xb) <= 1e-9)),
    }


def fmt_stats(name, s):
    ab = ", ".join(f"{k}:{v}" for k, v in sorted(s["aborts"].items()))
    return (f"{name:28s} powered {s['powered']:4d}/{s['flights']:<5d} "
            f"median {s['median']:5.2f}  mean {s['mean']:5.2f}  p90 {s['p90']:5.2f}  "
            f"soft {s['soft']:3d}  <1 {s['very_soft']:3d}  hops {s['hops']:3d} "
            f"(severe {s['severe_hops']:3d})  aborts [{ab}]")


def fmt_paired(name, p):
    return (f"{name:28s} pairs {p['pairs']:4d}  dMedian {p['d_median']:+6.2f} "
            f"[{p['d_median_ci'][0]:+.2f},{p['d_median_ci'][1]:+.2f}]  "
            f"dP90 {p['d_p90']:+6.2f} [{p['d_p90_ci'][0]:+.2f},{p['d_p90_ci'][1]:+.2f}]  "
            f"dMean {p['d_mean']:+6.2f} [{p['d_mean_ci'][0]:+.2f},{p['d_mean_ci'][1]:+.2f}]  "
            f"better/worse/same {p['better']}/{p['worse']}/{p['same']}  "
            f"(powered only-base {p['powered_only_base']}, only-cand {p['powered_only_cand']})")


def clean_table(rows):
    """Clean apogee sweep: mean over noise seeds per ascent-thrust entry."""
    by = {}
    for r in rows:
        s = r["summary"]
        j = r["i"]
        by.setdefault(j, []).append(s)
    out = []
    for j in sorted(by):
        ss = by[j]
        ap = np.mean([s["apogee_h"] for s in ss])
        sp = [s["touchdown_speed_mps"] for s in ss if is_powered(s)]
        out.append({"j": j, "apogee": float(ap),
                    "td_mean": float(np.mean(sp)) if sp else float("nan"),
                    "n_powered": len(sp),
                    "hops": sum(1 for s in ss if is_powered(s) and s.get("hop")),
                    "aborts": [s.get("abort_reason") for s in ss if s.get("abort_reason")],
                    "tilt_limited": sum(1 for s in ss if s.get("fire_tilt_prev_deg") is not None
                                        and s["fire_tilt_prev_deg"] >= 15.0)})
    return out
