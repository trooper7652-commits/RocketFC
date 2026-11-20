"""Summarize the proof batch (results/*.json from run_proof.sh) against the plan's gate."""
import json
import math
import os
import statistics as st

R = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")


def load(name):
    p = os.path.join(R, name + ".json")
    if not os.path.exists(p):
        return None
    rows = json.load(open(p, encoding="utf-8"))["rows"]
    errs = sum(1 for r in rows if "error" in r["summary"])
    assert errs == 0, (name, errs)
    return rows


def q(v, p):
    v = sorted(v)
    return v[min(len(v) - 1, int(p * len(v)))] if v else float("nan")


def ok(r):
    return bool(r["summary"]["landing_outcome"]["success"])


def sign_p(up, down):
    n = up + down
    if n == 0:
        return 1.0
    k = min(up, down)
    return min(1.0, 2 * sum(math.comb(n, i) for i in range(k + 1)) / 2 ** n)


def land(name, base=None):
    rows = load(name)
    if rows is None:
        return print("%-10s (missing)" % name)
    n = len(rows)
    s = sum(ok(r) for r in rows)
    dud = sum(1 for r in rows if r["summary"]["abort_reason"] == "DUD_IGNITER")
    ab = sum(1 for r in rows if r["summary"]["abort_reason"])
    ap = [r["summary"]["apogee_h"] for r in rows if r["summary"]["apogee_h"]]
    line = "%-10s success %4d/%d = %5.1f %%  aborts %3d  false DUD %d  apogee med %.1f p99 %.1f max %.1f m" % (
        name, s, n, 100.0 * s / n, ab, dud, st.median(ap), q(ap, 0.99), max(ap))
    if base:
        b = {(r["seed"], r["i"]): ok(r) for r in load(base)}
        up = sum(1 for r in rows if ok(r) and not b[(r["seed"], r["i"])])
        down = sum(1 for r in rows if not ok(r) and b[(r["seed"], r["i"])])
        line += "  vs %s: +%d/-%d p=%.2f" % (base, up, down, sign_p(up, down))
    print(line)


def released_before_impact(r):
    s = r["summary"]
    return s["chute_release_t"] is not None and s["touchdown_t"] is not None and \
        s["chute_release_t"] < s["touchdown_t"]


def freed_before_impact(r):
    s = r["summary"]
    return s["chute_open_t"] is not None and s["touchdown_t"] is not None and \
        s["chute_open_t"] < s["touchdown_t"]


def dud(name):
    rows = load(name)
    if rows is None:
        return print("%-10s (missing)" % name)
    n = len(rows)
    d = sum(1 for r in rows if r["summary"]["abort_reason"] == "DUD_IGNITER")
    rel = [r for r in rows if released_before_impact(r)]
    lead = [r["summary"]["touchdown_t"] - r["summary"]["chute_release_t"] for r in rel]
    print("%-10s DUD_IGNITER %d/%d  chute released before impact %d/%d  (median %.2f s before impact)" % (
        name, d, n, len(rel), n, st.median(lead) if lead else float("nan")))


def chute(name):
    rows = load(name)
    if rows is None:
        return print("%-10s (missing)" % name)
    n = len(rows)
    fr = [r for r in rows if freed_before_impact(r)]
    t = [r["summary"]["chute_open_t"] - r["summary"]["chute_release_t"] for r in fr]
    tdv = [r["summary"]["touchdown_speed_mps"] for r in rows if r["summary"]["touchdown_speed_mps"] is not None]
    rel = [r["summary"]["chute_releases"] for r in rows]
    print("%-10s canopy freed before impact %d/%d  freed %.2f s after 1st release (med)  "
          "impact med %.2f m/s  releases med %d" % (
              name, len(fr), n, st.median(t) if t else float("nan"),
              st.median(tdv) if tdv else float("nan"), st.median(rel)))


print("== landing, holdout 701-710 x 200, measured spreads, band 0.75 s")
land("A_hold")
land("B_hold", "A_hold")
print("== landing, ignition delay +-15 %")
land("A_hold15")
land("B_hold15", "A_hold15")
print("== dud landing igniter, dev 600")
dud("A_dud")
dud("B_dud")
print("== chute mode, stuck latch (count-based latch), dev 600")
for s in (0, 1, 2):
    chute("A_chute_s%d" % s)
    chute("B_chute_s%d" % s)
print("== chute mode, servo-travel latch (repass, R 80 deg, 375 deg/s), copy B firmware, dev 600")
for s in (0, 1, 2):
    chute("C_latch_s%d" % s)
