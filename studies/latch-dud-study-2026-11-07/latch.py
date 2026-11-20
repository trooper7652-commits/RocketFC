"""Stuck chute latch: how fast must a retry be (Step A), and which retry
scheme gets there (Step B). Chute-test mode (release at apogee), E18W/E18W
1.4 kg, 1.4 m annular canopy (Cd 2.2) with the sized fill model, dev seeds
101-103 x 200, measured spreads, wind -4..+4 m/s.

usage: python latch.py A | B | B430 | land
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "motor-chute-study-2026-11-04"))
import mclib as L  # noqa: E402

WS = "ld_base2"
CH = {"chute_diameter_m": 1.4, "chute_cd": 2.2, "chute_fill_n": 8}

# Fast-cycle timings are servo travel: LOCK 40 -> OPEN 180 is 140 deg,
# 0.37 s at 375 deg/s (DS3218-type at 5 V), 0.33 s at 430 deg/s (6.8 V).
def variants(rate):
    full = "400.0f" if rate < 400 else "350.0f"
    v = {"current": {},
         "conf900": {"CHUTE_CONFIRM_MS": "900.0f"},
         "conf600": {"CHUTE_CONFIRM_MS": "600.0f"}}
    for k in (1, 2, 3):
        v["full%d" % k] = {"CHUTE_FAST_CYCLES": str(k), "CHUTE_FAST_FIRST_MS": full,
                           "CHUTE_FAST_OPEN_MS": full, "CHUTE_FAST_LOCK_MS": full}
    for k in (2, 3, 4):
        v["wig%d" % k] = {"CHUTE_FAST_CYCLES": str(k), "CHUTE_FAST_FIRST_MS": full,
                          "CHUTE_FAST_OPEN_MS": "120.0f", "CHUTE_FAST_LOCK_MS": "100.0f"}
    v["full2+conf900"] = dict(v["full2"], CHUTE_CONFIRM_MS="900.0f")
    v["wig3+conf900"] = dict(v["wig3"], CHUTE_CONFIRM_MS="900.0f")
    return v


def le3_paired(base, rows):
    """Flights that become <= 3 m/s vs flights that stop being, + sign-test p."""
    def ok(r):
        c = (r["summary"].get("ground") or {}).get("contact")
        return bool(c) and abs(c["vel"][2]) <= 3.0
    a = {(r["seed"], r["i"]): ok(r) for r in base}
    b = {(r["seed"], r["i"]): ok(r) for r in rows}
    up = sum(1 for k in a if b.get(k) and not a[k])
    down = sum(1 for k in a if a[k] and not b.get(k))
    m = up + down
    p = min(1.0, 2 * sum(math.comb(m, i) for i in range(min(up, down) + 1)) / 2 ** m) if m else 1.0
    return up, down, p


def cell(m):
    return "%.2f / %.2f m/s, %.0f %% ≤ 3" % (m["vz_med"], m["vz_p90"], 100 * m["le3"])


def step_a():
    print("\n== Step A: canopy freed d s after the first release (stuck latch freed late), chute mode")
    print("| delay | fill n = 8: impact med / p90, ≤ 3 m/s | free height med | fill n = 4 |")
    print("|---|---|---|---|")
    for d in (0.0, 0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 1.9):
        r8 = L.run("ld_A_d%.2f_n8" % d, WS, mode="chute", plant=dict(CH, chute_free_delay_s=d))
        r4 = L.run("ld_A_d%.2f_n4" % d, WS, mode="chute",
                   plant=dict(CH, chute_free_delay_s=d, chute_fill_n=4))
        m8, m4 = L.chute_metrics(r8), L.chute_metrics(r4)
        print("| %.2f s | %s | %.1f m | %s |" % (d, cell(m8), m8["dh_med"], cell(m4)), flush=True)


def step_b(rate=375.0):
    vs = variants(rate)
    res = {}
    for name, fw in vs.items():
        for stuck, back in ((0, 30), (1, 30), (1, 135), (2, 30), (2, 135)):
            plant = dict(CH, latch_rate_dps=rate, chute_stuck_releases=stuck,
                         latch_backswing_deg=back)
            lab = "ld_B_r%g_%s_s%d_b%d" % (rate, name, stuck, back)
            res[(name, stuck, back)] = L.run(lab, WS, mode="chute", plant=plant, fw=fw or None)
    print("\n== Step B: retry schemes, latch servo %g deg/s, chute mode (impact med / p90, share ≤ 3 m/s)" % rate)
    print("Retry counted: b30 = any 30 deg shake frees a sticky pin; b135 = needs a full swing back to LOCK.")
    print("| scheme | stuck 0 | unconfirmed (stuck 0) | stuck 1, b30 | stuck 1, b135 | stuck 2, b30 | stuck 2, b135 |")
    print("|---|---|---|---|---|---|---|")
    for name in vs:
        cells = []
        for key in ((0, 30), (1, 30), (1, 135), (2, 30), (2, 135)):
            m = L.chute_metrics(res[(name,) + key])
            c = cell(m)
            if name != "current":
                up, down, p = le3_paired(res[("current",) + key], res[(name,) + key])
                c += " (+%d/−%d%s)" % (up, down, ", p<0.01" if p < 0.01 else "")
            cells.append(c)
        unc = L.chute_metrics(res[(name, 0, 30)])["unconfirmed"]
        print("| %s | %s | %.1f %% | %s |" % (name, cells[0], 100 * unc, " | ".join(cells[1:])), flush=True)


def land_check(names=("full2", "wig3", "full2+conf900")):
    """Landing mode (chute only on an abort): the schemes must not change
    anything for a healthy latch, and help an abort with a stuck one."""
    vs = variants(375.0)
    print("\n== Landing mode, latch 375 deg/s: success (paired vs current) | aborted flights: impact med, ≤ 3 m/s")
    for stuck in (0, 1):
        base = L.run("ld_L_current_s%d" % stuck, WS,
                     plant=dict(CH, latch_rate_dps=375.0, chute_stuck_releases=stuck))
        for name in ("current",) + tuple(names):
            rows = base if name == "current" else L.run(
                "ld_L_%s_s%d" % (name, stuck), WS,
                plant=dict(CH, latch_rate_dps=375.0, chute_stuck_releases=stuck), fw=vs[name])
            k = sum(r["summary"]["landing_outcome"]["success"] for r in rows)
            resc, lost, p = L.paired(base, rows)
            ab = [r for r in rows if r["summary"].get("abort_reason")]
            m = L.chute_metrics(ab)
            print("stuck %d | %-14s | %d/%d (+%d/−%d) | %d aborts: %.2f m/s med, %.0f %% ≤ 3" % (
                stuck, name, k, len(rows), resc, lost, len(ab), m["vz_med"], 100 * m["le3"]), flush=True)


if __name__ == "__main__":
    what = sys.argv[1]
    if what == "A":
        step_a()
    elif what == "B":
        step_b(375.0)
    elif what == "B430":
        step_b(430.0)
    elif what == "land":
        land_check(tuple(sys.argv[2:]) or ("full2", "wig3", "full2+conf900"))
