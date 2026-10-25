#!/usr/bin/env python3
"""Landing success rate: Monte Carlo + verdicts from landing_outcome.py.

Flies the same random flights as `mc_batch.py` / `sweep.py monte-carlo`
(identical draws, seed + i per flight) with the legs ground model and reports:

  - the success rate in two scenarios, each with its 95 % Wilson confidence
    range ("most likely X %, very likely between L and H %"):
      today     -- the sim's current spreads (mass +-5 %, ignition delay
                   +-15 %, wind up to 4 m/s, ...)
      measured  -- after weighing the rocket (+-1 %) and timing the e-match on
                   the ThrustStand (+-5 %)
  - why flights fail: the headline cause, and how often each cause appears at
    all among the flights that made a powered landing
  - success rate vs. the crash speed (a rule, re-judged without re-flying)
  - sensitivity to the [MEASURE] guesses, one at a time on 600 flights

Run (about 3 minutes on 20 cores):
    python tools/sim/landing_report.py --out "../landing-outcome-study"
"""
import argparse
import collections
import copy
import dataclasses
import json
import math
import multiprocessing
import os
import sys
import time

SIM_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SIM_DIR)
import landing_outcome  # noqa: E402
import mc_batch  # noqa: E402

HEADLINE_SEEDS = list(range(101, 111))
SENS_SEEDS = [101, 102, 103]
N_PER_SEED = 200
MEASURED = {"mass_descent_kg": 0.01, "ignition_delay_s": 0.05}
SENSITIVITY = [
    ("CG height 0.50 m", {"landed_cg_height_m": 0.50}),
    ("CG height 0.70 m", {"landed_cg_height_m": 0.70}),
    ("grass friction 0.4", {"ground_mu": 0.4}),
    ("grass friction 0.8", {"ground_mu": 0.8}),
    ("band burn 1.0 s (not 1.5)", {"legs_cut_s": 1.0}),
    ("band burn 0.5 s (not 1.5)", {"legs_cut_s": 0.5}),
]
CRASH_SWEEP = [1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0]

# Reference palette (dataviz skill, light mode).
SURFACE, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
BLUE, ORANGE = "#2a78d6", "#eb6834"

CAUSE_TEXT = {
    "SUCCESS": "Landed upright, intact",
    "NO_POWERED_LANDING": "No powered landing (abort / missed fire)",
    "HOPPED": "Hopped in mid-air and fell back",
    "LEGS_NOT_DOWN": "Legs not down at touchdown",
    "LEGS_BROKEN": "Hit faster than the crash speed",
    "LIFTED_OFF": "Motor lifted it back off the ground",
    "TIPPED_OVER": "Tipped over",
    "BLOWN_OVER": "Blown over after standing",
    "NOT_SETTLED": "Still moving when the sim stopped",
    "NO_TOUCHDOWN": "Never reached the ground",
}


EXAMPLES_INTRO = """## Watch it in 3D

Just the landing, same physics (run from the RocketFC folder):

- Lands upright: `python tools/sim/flight3d.py --drop -1` (1 m/s straight down)
- Falls over: `python tools/sim/flight3d.py --drop -1,1.5` (1 m/s down, 1.5 m/s
  sideways -- milder than a typical simulated landing, whose median is 2.9 m/s sideways)

A full Monte Carlo flight for each verdict:
"""


def wilson(k, n, z=1.96):
    """95 % Wilson score interval for k successes out of n (stays sensible
    at 0 % and 100 %, unlike p +- 1.96 sqrt(p(1-p)/n))."""
    if n == 0:
        return 0.0, 0.0, 0.0
    p = k / n
    den = 1.0 + z * z / n
    mid = (p + z * z / (2 * n)) / den
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return p, max(0.0, mid - half), min(1.0, mid + half)


def fly(base, disp_fields, seeds, pool, label):
    tasks = []
    for seed in seeds:
        tasks += mc_batch.build_jobs(base, disp_fields, seed, N_PER_SEED, [])
    dll = base_dll()
    jobs = [(dataclasses.asdict(vc), "land", dll, s) for vc, _, s, _ in tasks]
    t0 = time.time()
    summaries = pool.map(mc_batch._run_one, jobs, chunksize=4)
    errs = sum(1 for s in summaries if "error" in s)
    print("  {}: {} flights in {:.0f} s ({} errors)".format(
        label, len(summaries), time.time() - t0, errs))
    return [{"seed": s - i, "i": i, "dispersions": d, "summary": summ}
            for (vc, d, s, i), summ in zip(tasks, summaries)]


_DLL = None


def base_dll():
    global _DLL
    if _DLL is None:
        import core
        _DLL = core.default_dll()
    return _DLL


def stats(rows, crash_speed=None):
    outs = [landing_outcome.classify(r["summary"], crash_speed=crash_speed)
            if crash_speed is not None else r["summary"]["landing_outcome"]
            for r in rows if "error" not in r["summary"]]
    n = len(outs)
    k = sum(1 for o in outs if o["success"])
    primary = collections.Counter(o["code"] for o in outs)
    powered = [o for o in outs if "NO_POWERED_LANDING" not in o["factors"]]
    any_factor = collections.Counter(f for o in powered for f in set(o["factors"]))
    tipped = [o for o in powered if o["code"] in ("TIPPED_OVER", "BLOWN_OVER")
              or "TIPPED_OVER" in o["factors"]]
    blame = collections.Counter(o["primary"] for o in tipped if o["primary"])
    hops_ok = sum(1 for o in outs if o["factors"] and set(o["factors"]) <= {"HOPPED"})
    return {"n": n, "k": k, "ci": wilson(k, n), "primary": primary,
            "powered": len(powered), "any_factor": any_factor,
            "tip_blame": blame, "n_tipped": len(tipped), "k_hops_ok": k + hops_ok}


def pct(x):
    return "{:.1f} %".format(100.0 * x)


def fmt_rate(st):
    p, lo, hi = st["ci"]
    return "most likely **{}** ({} of {}), very likely between {} and {}".format(
        pct(p), st["k"], st["n"], pct(lo), pct(hi))


def tip_limits(rows):
    """(between two legs, toward a leg) sideways tip-over speed (m/s) for the
    legs geometry these flights used, at a real contact's mass and inertia."""
    for r in rows:
        g = r["summary"].get("ground") or {}
        c = g.get("contact")
        if c:
            p = g["params"]
            _, v_edge, _ = landing_outcome.edge_limits(
                p["leg_foot_radius_m"], p["landed_cg_height_m"], c["mass_kg"], c["inertia_kgm2"])
            return v_edge, v_edge * math.sqrt(2.0)
    return float("nan"), float("nan")


def contact_quantiles(rows):
    """Median/p90 of the contact speeds over powered landings."""
    vz, vh = [], []
    for r in rows:
        g = r["summary"].get("ground") or {}
        c = g.get("contact")
        o = r["summary"]["landing_outcome"]
        if c and "NO_POWERED_LANDING" not in o["factors"]:
            vz.append(abs(c["vel"][2]))
            vh.append(math.hypot(c["vel"][0], c["vel"][1]))
    q = lambda a, p: sorted(a)[min(len(a) - 1, int(p * len(a)))] if a else float("nan")
    return {"vz_med": q(vz, .5), "vz_p90": q(vz, .9), "vh_med": q(vh, .5),
            "vh_p90": q(vh, .9), "n": len(vz)}


# --------------------------------------------------------------- plots
def _style(ax):
    ax.set_facecolor(SURFACE)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=INK2, labelsize=9)
    ax.grid(axis="x", color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def plot_causes(today, meas, path):
    import matplotlib.pyplot as plt
    codes = [c for c in landing_outcome.CODES
             if today["primary"].get(c) or meas["primary"].get(c)]
    codes.sort(key=lambda c: today["primary"].get(c, 0))
    fig, ax = plt.subplots(figsize=(8.5, 0.55 * len(codes) + 1.4), facecolor=SURFACE)
    _style(ax)
    y = range(len(codes))
    hgt = 0.38
    for off, st, col, name in ((hgt / 2 + 0.01, today, BLUE, "Today's guesses"),
                               (-hgt / 2 - 0.01, meas, ORANGE, "After measuring")):
        vals = [100.0 * st["primary"].get(c, 0) / st["n"] for c in codes]
        ax.barh([i + off for i in y], vals, height=hgt, color=col, label=name)
        for i, v in zip(y, vals):
            ax.text(v + 0.6, i + off, "{:.1f} %".format(v), va="center",
                    fontsize=8, color=INK2)
    ax.set_yticks(list(y))
    ax.set_yticklabels([CAUSE_TEXT[c] for c in codes], color=INK, fontsize=9)
    ax.set_xlabel("share of all 2,000 flights (headline cause)", color=INK2, fontsize=9)
    ax.set_title("Why landings fail", color=INK, fontsize=12, loc="left")
    ax.legend(frameon=False, fontsize=9, loc="lower right")
    fig.tight_layout()
    fig.savefig(path, dpi=130, facecolor=SURFACE)
    plt.close(fig)


def plot_crash(sweep_today, sweep_meas, path):
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(8.5, 4.2), facecolor=SURFACE)
    _style(ax)
    ax.grid(axis="y", color=GRID, linewidth=0.8)
    for sw, col, name in ((sweep_today, BLUE, "Today's guesses"),
                          (sweep_meas, ORANGE, "After measuring")):
        xs = [c for c, _ in sw]
        ps = [100.0 * st["ci"][0] for _, st in sw]
        lo = [100.0 * st["ci"][1] for _, st in sw]
        hi = [100.0 * st["ci"][2] for _, st in sw]
        ax.fill_between(xs, lo, hi, color=col, alpha=0.15, linewidth=0)
        ax.plot(xs, ps, color=col, linewidth=2, marker="o", markersize=5, label=name)
    ax.axvline(3.0, color=INK2, linewidth=1, linestyle=":")
    ax.text(3.05, ax.get_ylim()[1] * 0.95, "3 m/s placeholder", color=INK2,
            fontsize=8, va="top")
    ax.set_xlabel("crash speed: vertical contact speed the legs survive (m/s)",
                  color=INK2, fontsize=9)
    ax.set_ylabel("landing success rate (%)", color=INK2, fontsize=9)
    ax.set_title("Success rate vs. how hard a hit the legs survive (band = 95 % range)",
                 color=INK, fontsize=12, loc="left")
    ax.legend(frameon=False, fontsize=9, loc="upper left")
    fig.tight_layout()
    fig.savefig(path, dpi=130, facecolor=SURFACE)
    plt.close(fig)


# --------------------------------------------------------------- report
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True, help="folder for report.md, plots, data")
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    args = ap.parse_args()
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    os.makedirs(args.out, exist_ok=True)
    import matplotlib
    matplotlib.use("Agg")

    mc_batch._init_worker(SIM_DIR)
    import sweep
    import vehicle
    base = vehicle.default_vehicle_config(base_dll())
    base.ground_model = "legs"
    disp_today = list(sweep.DISPERSION_FIELDS)
    disp_meas = [(a, MEASURED.get(a, w)) for a, w in disp_today]

    t0 = time.time()
    print("flying:")
    with multiprocessing.Pool(args.workers, initializer=mc_batch._init_worker,
                              initargs=(SIM_DIR,)) as pool:
        today = fly(base, disp_today, HEADLINE_SEEDS, pool, "today's guesses")
        meas = fly(base, disp_meas, HEADLINE_SEEDS, pool, "after measuring")
        sens = []
        for name, over in SENSITIVITY:
            vc = copy.deepcopy(base)
            for k, v in over.items():
                setattr(vc, k, v)
            sens.append((name, over, fly(vc, disp_today, SENS_SEEDS, pool, name)))

    st_t, st_m = stats(today), stats(meas)
    sens_base = stats([r for r in today if r["seed"] in SENS_SEEDS])
    sweep_t = [(c, stats(today, c)) for c in CRASH_SWEEP]
    sweep_m = [(c, stats(meas, c)) for c in CRASH_SWEEP]
    cq_t, cq_m = contact_quantiles(today), contact_quantiles(meas)
    tip_edge, tip_foot = tip_limits(today)

    plot_causes(st_t, st_m, os.path.join(args.out, "causes.png"))
    plot_crash(sweep_t, sweep_m, os.path.join(args.out, "crash_speed_sweep.png"))

    # Example flights for the 3D view: the first flight with each verdict.
    examples = {}
    for scenario, rows in (("today", today), ("measured", meas)):
        for r in rows:
            o = r["summary"]["landing_outcome"]
            if o["code"] not in examples:
                examples[o["code"]] = {"seed": r["seed"], "i": r["i"],
                                       "scenario": scenario, "label": o["label"]}

    L = []
    w = L.append
    w("# RocketFC landing success rate (simulator)\n")
    w("*Generated {} by `tools/sim/landing_report.py` -- {:.0f} s, {} flights.*\n".format(
        time.strftime("%Y-%m-%d %H:%M"), time.time() - t0,
        len(today) + len(meas) + sum(len(x[2]) for x in sens)))
    w("## The answer\n")
    w("Success = a powered landing that ends **upright and intact**: the F15 lit, nothing aborted, "
      "no hop, legs down at touchdown, contact under {:.1f} m/s, and still standing {:.0f} s after "
      "burnout.\n".format(base.crash_speed_mps, base.settle_s))
    w("| Scenario | Landing success rate (2,000 flights, 95 % range) |")
    w("|---|---|")
    w("| **Today's guesses** (mass +-5 %, e-match delay +-15 %, wind up to 4 m/s) | {} |".format(fmt_rate(st_t)))
    w("| **After measuring** (mass +-1 %, delay +-5 %) | {} |".format(fmt_rate(st_m)))
    w("")
    w("The range is how sure the simulator is *of its own answer* (sampling): with 2,000 flights "
      "the true rate of this simulated rocket is very likely inside it. It does **not** cover the "
      "simulator being wrong about the real rocket -- the sensitivity table below shows how much "
      "the guessed numbers move it.\n")

    w("## Why landings fail\n")
    w("![causes](causes.png)\n")
    w("Headline cause = the first thing that went wrong, in time order. Count of all {} flights:\n".format(st_t["n"]))
    w("| Cause | Today | After measuring |")
    w("|---|---|---|")
    for c in landing_outcome.CODES:
        a, b = st_t["primary"].get(c, 0), st_m["primary"].get(c, 0)
        if a or b:
            w("| {} | {} ({}) | {} ({}) |".format(CAUSE_TEXT[c], a, pct(a / st_t["n"]),
                                                  b, pct(b / st_m["n"])))
    w("")
    w("Among flights that made a powered landing ({} today, {} after measuring), how often each "
      "problem happened at all (a flight can have several):\n".format(st_t["powered"], st_m["powered"]))
    w("| Problem | Today | After measuring |")
    w("|---|---|---|")
    for c in ("HOPPED", "LEGS_NOT_DOWN", "LEGS_BROKEN", "LIFTED_OFF", "TIPPED_OVER", "BLOWN_OVER"):
        a, b = st_t["any_factor"].get(c, 0), st_m["any_factor"].get(c, 0)
        w("| {} | {} | {} |".format(CAUSE_TEXT[c],
                                    pct(a / max(st_t["powered"], 1)), pct(b / max(st_m["powered"], 1))))
    w("")
    w("Contact speeds of the powered landings -- vertical: median {:.1f} / p90 {:.1f} m/s today, "
      "{:.1f} / {:.1f} after measuring; **sideways: median {:.1f} / p90 {:.1f} m/s today**, "
      "{:.1f} / {:.1f} after measuring. The legs tip over at roughly {:.1f} m/s sideways (between "
      "two legs) to {:.1f} m/s (toward a leg) with foot radius {:.2f} m and CG height {:.2f} m.\n".format(
          cq_t["vz_med"], cq_t["vz_p90"], cq_m["vz_med"], cq_m["vz_p90"],
          cq_t["vh_med"], cq_t["vh_p90"], cq_m["vh_med"], cq_m["vh_p90"],
          tip_edge, tip_foot, base.leg_foot_radius_m, base.landed_cg_height_m))
    if st_t["n_tipped"]:
        w("What pushed the {} falls over hardest (today; biggest cause vs. its own limit at "
          "contact):\n".format(st_t["n_tipped"]))
        w("| Push | Falls |")
        w("|---|---|")
        for k, v in st_t["tip_blame"].most_common():
            w("| {} | {} ({}) |".format(k, v, pct(v / st_t["n_tipped"])))
        w("")
    w("If a mid-air hop that still ends upright counted as success, the rates would be "
      "{} today and {} after measuring.\n".format(
          pct(st_t["k_hops_ok"] / st_t["n"]), pct(st_m["k_hops_ok"] / st_m["n"])))

    w("## How hard a hit can the legs take?\n")
    w("![crash speed](crash_speed_sweep.png)\n")
    w("Crash speed is a [MEASURE] placeholder (3 m/s, about a 46 cm drop). Same flights, judged "
      "again at each limit:\n")
    w("| Crash speed | Today | After measuring |")
    w("|---|---|---|")
    for (c, a), (_, b) in zip(sweep_t, sweep_m):
        w("| {:.1f} m/s (drop from {:.0f} cm) | {} [{}-{}] | {} [{}-{}] |".format(
            c, 100 * c * c / (2 * 9.81), pct(a["ci"][0]), pct(a["ci"][1]), pct(a["ci"][2]),
            pct(b["ci"][0]), pct(b["ci"][1]), pct(b["ci"][2])))
    w("")

    w("## Sensitivity to the guessed numbers\n")
    w("Today's guesses, 600 flights (seeds 101-103), one change at a time vs. the same 600 "
      "flights with the defaults (foot radius {:.2f} m, CG {:.2f} m, friction {:.1f}, band burn "
      "{:.1f} s):\n".format(base.leg_foot_radius_m, base.landed_cg_height_m, base.ground_mu,
                            base.legs_cut_s))
    w("| Change | Success rate [95 % range] | Legs not down | Tipped over |")
    w("|---|---|---|---|")
    rows_ = [("defaults", sens_base)] + [(name, stats(rr)) for name, _, rr in sens]
    for name, st in rows_:
        w("| {} | {} [{}-{}] | {} | {} |".format(
            name, pct(st["ci"][0]), pct(st["ci"][1]), pct(st["ci"][2]),
            pct(st["any_factor"].get("LEGS_NOT_DOWN", 0) / max(st["powered"], 1)),
            pct(st["any_factor"].get("TIPPED_OVER", 0) / max(st["powered"], 1))))
    w("")

    w("## What the simulator does and doesn't know\n")
    w("- The legs are rigid and never break in the physics; \"crash\" is the speed rule above.")
    w("- Leg geometry, CG height, grass stiffness/friction and the band-burn time are estimates "
      "tagged [MEASURE]/[ESTIMATE] in `tools/sim/vehicle.py`.")
    w("- The TVC keeps flying after touchdown exactly as the firmware does; the F15 keeps "
      "burning (it can't be shut off), so if it is still pushing harder than the weight at "
      "contact, it lifts the rocket back off.")
    w("- Field slope is random 0-{:.0f} deg per flight; wind is steady (no gusts).".format(
        base.ground_slope_max_deg))
    w("- Nothing in the flight firmware was changed.\n")
    w(EXAMPLES_INTRO)
    for k, e in examples.items():
        w("- {}: `python tools/sim/flight3d.py --sim {}:{}{}` -- {}".format(
            CAUSE_TEXT[k], e["seed"], e["i"],
            " --measured" if e["scenario"] == "measured" else "", e["label"]))
    w("")

    with open(os.path.join(args.out, "report.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(L))
    with open(os.path.join(args.out, "results.json"), "w", encoding="utf-8") as f:
        json.dump({"today": today, "measured": meas,
                   "sensitivity": [{"name": n, "override": o, "rows": rr} for n, o, rr in sens],
                   "examples": examples}, f)
    print("today:    " + fmt_rate(st_t).replace("**", ""))
    print("measured: " + fmt_rate(st_m).replace("**", ""))
    print("wrote " + os.path.join(args.out, "report.md"))


if __name__ == "__main__":
    main()
