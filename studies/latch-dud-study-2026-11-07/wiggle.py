"""Angle-target latch wiggles (the owner's 70<->150 deg idea) vs a stuck latch.

The latch lets the door go at R = 70-90 deg (owner, 2026-11-07). A wiggle
can't re-catch the door. Chute mode, E18W/E18W 1.4 kg, 1.4 m annular canopy.

  screen   dev seeds 101-103 x 100, latch servo 375 deg/s (5 V):
           cycles 70<->150, 55<->150, 55<->110 (K = 2, 3 down-strokes) plus
           today's retry and the earlier binary 143<->180 wiggle;
           R = 70 / 80 / 90; stuck 0 / 1 / 2; retry rule "repass" (back up
           through R) and "shake" (any 30 deg stroke)
  speed    the best cycles at R = 80: servo 430 deg/s (6.8 V dwells) and
           300 deg/s (slower than expected, 5 V dwells)
  hold     holdout 701-710 x 200, NAME at R = 80, stuck 0 / 1 / 2, repass

usage: python wiggle.py screen | speed NAME [NAME..] | hold NAME
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "motor-chute-study-2026-11-04"))
import mclib as L  # noqa: E402
from latch import le3_paired  # noqa: E402

WS = "ld_base2"
CH = {"chute_diameter_m": 1.4, "chute_cd": 2.2, "chute_fill_n": 8}
LOCK = 40.0


def ms(x):
    return "%.1ff" % (5.0 * round(x * 1000.0 / 5.0 + 0.5))  # round up to 5 ms


def cycle(low, high, k, rate):
    """fw overrides + plant angles for an angle-target wiggle, dwells = travel time."""
    fw = {"CHUTE_WIG_CYCLES": str(k), "CHUTE_WIG_LOW_DEG": "%.1ff" % low,
          "CHUTE_WIG_HIGH_DEG": "%.1ff" % high,
          "CHUTE_WIG_FIRST_MS": ms((high - LOCK) / rate),
          "CHUTE_WIG_DOWN_MS": ms((high - low) / rate),
          "CHUTE_WIG_UP_MS": ms((high - low) / rate)}
    return fw, {"latch_wig_low_deg": low, "latch_wig_high_deg": high}


def schemes(rate=375.0):
    s = {"today": ({}, {}),
         "old143-180": ({"CHUTE_FAST_CYCLES": "3", "CHUTE_FAST_FIRST_MS": "400.0f",
                         "CHUTE_FAST_OPEN_MS": "120.0f", "CHUTE_FAST_LOCK_MS": "100.0f"}, {})}
    for low, high in ((70, 150), (55, 150), (55, 110)):
        for k in (2, 3):
            s["%d-%d k%d" % (low, high, k)] = cycle(low, high, k, rate)
    return s


def runs(name, fw, pl, R, stuck, model, rate=375.0, seeds=L.DEV, n=100, dwell_rate=None, tag=""):
    plant = dict(CH, **pl, latch_rate_dps=rate, chute_stuck_releases=stuck,
                 latch_release_deg=float(R), latch_model=model, latch_backswing_deg=30.0)
    lab = "ld_W%s_r%g_%s_R%d_s%d_%s_n%d_%s" % (tag, rate, name.replace(" ", "_"), R, stuck,
                                              model, n, "-".join(map(str, seeds)))
    return L.run(lab, WS, seeds=seeds, n=n, mode="chute", plant=plant, fw=fw or None)


def try_times(rows):
    """Median time of each release attempt after the first release command."""
    ts = [r["summary"].get("latch_try_t") or [] for r in rows]
    out = []
    for i in range(3):
        v = [t[i] for t in ts if len(t) > i and t[i] is not None]
        out.append("%.2f" % L.q(v, 0.5) if v else "-")
    return "/".join(out)


def cell(m):
    return "%.2f m/s, %.0f %%" % (m["vz_med"], 100 * m["le3"])


def screen():
    sc = schemes()
    for R in (70, 80, 90):
        print("\n== R = %d deg (the latch lets go here), servo 375 deg/s; impact median, share ≤ 3 m/s" % R)
        print("| scheme | healthy latch | 1 stuck, re-pass | 2 stuck, re-pass | tries at (s, 2 stuck, re-pass) "
              "| 1 stuck, shake | 2 stuck, shake |")
        print("|---|---|---|---|---|---|---|")
        base = {}
        for name, (fw, pl) in sc.items():
            res = {(0, "repass"): runs(name, fw, pl, R, 0, "repass")}
            for stuck in (1, 2):
                for model in ("repass", "shake"):
                    res[(stuck, model)] = runs(name, fw, pl, R, stuck, model)
            if name == "today":
                base = res
            cells = []
            for key in ((0, "repass"), (1, "repass"), (2, "repass")):
                c = cell(L.chute_metrics(res[key]))
                if name != "today":
                    up, down, p = le3_paired(base[key], res[key])
                    c += " (+%d/−%d)" % (up, down)
                cells.append(c)
            cells.append(try_times(res[(2, "repass")]))
            for key in ((1, "shake"), (2, "shake")):
                cells.append(cell(L.chute_metrics(res[key])))
            print("| %s | %s |" % (name, " | ".join(cells)), flush=True)


def speed(names):
    print("\n== servo speed, R = 80, re-pass: impact median, share ≤ 3 m/s (tries at, 2 stuck)")
    for name in names:
        low, high, k = (int(x) for x in name.replace(" k", "-").split("-"))
        for rate, dwell in ((430.0, 430.0), (375.0, 375.0), (300.0, 375.0)):
            fw, pl = cycle(low, high, k, dwell)
            res = [runs(name, fw, pl, 80, s, "repass", rate=rate, tag="d%g" % dwell) for s in (0, 1, 2)]
            print("%-12s servo %g deg/s (dwells for %g): healthy %s | 1 stuck %s | 2 stuck %s (%s)" % (
                name, rate, dwell, cell(L.chute_metrics(res[0])), cell(L.chute_metrics(res[1])),
                cell(L.chute_metrics(res[2])), try_times(res[2])), flush=True)


def hold(name):
    sc = schemes()
    fw, pl = sc[name]
    print("\n== holdout 701-710 x 200 (2000 flights), R = 80, re-pass, servo 375 deg/s")
    for stuck in (0, 1, 2):
        base = runs("today", {}, {}, 80, stuck, "repass", seeds=L.HOLD, n=200)
        rows = runs(name, fw, pl, 80, stuck, "repass", seeds=L.HOLD, n=200)
        mb, mr = L.chute_metrics(base), L.chute_metrics(rows)
        up, down, p = le3_paired(base, rows)
        lo, hi = L.wilson(round(mr["le3"] * mr["n"]), mr["n"])
        print("stuck %d: today %s | %s %s (95 %% CI %.1f-%.1f %%), paired +%d/−%d" % (
            stuck, cell(mb), name, cell(mr), lo, hi, up, down), flush=True)


if __name__ == "__main__":
    what = sys.argv[1]
    if what == "screen":
        screen()
    elif what == "speed":
        speed(sys.argv[2:])
    elif what == "hold":
        hold(" ".join(sys.argv[2:]))
