#!/usr/bin/env python3
"""Parameter sweeps and Monte-Carlo dispersion runs over the closed-loop
simulator: vary one or two VehicleConfig fields over a range, or randomize
every dispersion at once, run N flights headless in parallel, and plot
outcome maps (touchdown speed, max tilt, abort rate, landing-burn ignition
error, peak KF error).

Scoring reuses tvc_pid_tuner.py's metric definitions and weights (ITAE,
overshoot, oscillation, gimbal thrash, saturation -- see
../../tvc-sim/tvc_pid_tuner.py section 7) via closed_loop_backend, so "good"
means the same thing whether you're looking at a sweep map here or a single
tuning run there.

Run:
    python tools/sim/sweep.py grid --param wind_mps --range -5 5 --n 11
    python tools/sim/sweep.py monte-carlo --n 200 --seed 7
"""
import argparse
import copy
import dataclasses
import json
import multiprocessing
import os
import random
import sys

import core
import run as run_mod
import vehicle

SIM_DIR = os.path.dirname(os.path.abspath(__file__))

# Fields varied by --dispersions monte-carlo mode, as (attr, relative_pct)
# -- each is jittered by +/- that fraction of its nominal value (uniform),
# except disturbance/wind terms which get an absolute random draw since
# their nominal is often 0.
DISPERSION_FIELDS = [
    ("mass_descent_kg", 0.05), ("inertia_kgm2", 0.15), ("cda_m2", 0.20),
    ("cg_from_nose_m", 0.05), ("gimbal_pivot_from_nose_m", 0.03),
    ("servo_slew_dps", 0.10), ("servo_lag_tau_s", 0.25),
    ("ignition_delay_s", 0.15),
]


def _run_one(args):
    vcfg_dict, mode, dll_path, seed = args
    vcfg = vehicle.VehicleConfig(**vcfg_dict)
    try:
        result = run_mod.simulate(vcfg, mode=mode, dll_path=dll_path, seed=seed,
                                  record=False)
        return result["summary"]
    except Exception as e:
        return {"error": str(e)}


def _vcfg_to_dict(vcfg):
    return dataclasses.asdict(vcfg)


def grid_sweep(param, values, mode="land", dll_path=None, base_vcfg=None,
              seed=42, workers=None):
    """Vary ONE VehicleConfig field over `values`, run each headless, return
    a list of (value, summary) pairs."""
    dll_path = dll_path or core.default_dll()
    base_vcfg = base_vcfg or vehicle.default_vehicle_config(dll_path)
    jobs = []
    for v in values:
        vc = copy.deepcopy(base_vcfg)
        setattr(vc, param, v)
        jobs.append((_vcfg_to_dict(vc), mode, dll_path, seed))

    workers = workers or max(1, (os.cpu_count() or 2) - 1)
    with multiprocessing.Pool(workers) as pool:
        summaries = pool.map(_run_one, jobs)
    return list(zip(values, summaries))


def grid_sweep_2d(param_x, values_x, param_y, values_y, mode="land",
                  dll_path=None, base_vcfg=None, seed=42, workers=None):
    """Vary TWO fields over a grid. Returns a list of (vx, vy, summary)."""
    dll_path = dll_path or core.default_dll()
    base_vcfg = base_vcfg or vehicle.default_vehicle_config(dll_path)
    jobs = []
    coords = []
    for vx in values_x:
        for vy in values_y:
            vc = copy.deepcopy(base_vcfg)
            setattr(vc, param_x, vx)
            setattr(vc, param_y, vy)
            jobs.append((_vcfg_to_dict(vc), mode, dll_path, seed))
            coords.append((vx, vy))

    workers = workers or max(1, (os.cpu_count() or 2) - 1)
    with multiprocessing.Pool(workers) as pool:
        summaries = pool.map(_run_one, jobs)
    return [(vx, vy, s) for (vx, vy), s in zip(coords, summaries)]


def monte_carlo(n, mode="land", dll_path=None, base_vcfg=None, seed=42,
                workers=None, dispersion_fields=None):
    """Randomize every dispersion field independently (uniform +/- pct
    around nominal), run N flights, return a list of (dispersions_dict,
    summary) pairs."""
    dll_path = dll_path or core.default_dll()
    base_vcfg = base_vcfg or vehicle.default_vehicle_config(dll_path)
    dispersion_fields = dispersion_fields or DISPERSION_FIELDS
    rng = random.Random(seed)

    jobs = []
    dispersions_list = []
    for i in range(n):
        vc = copy.deepcopy(base_vcfg)
        disp = {}
        for attr, pct in dispersion_fields:
            nominal = getattr(vc, attr)
            factor = 1.0 + rng.uniform(-pct, pct)
            setattr(vc, attr, nominal * factor)
            disp[attr] = factor
        # independent wind draw (nominal is usually 0, so no relative jitter)
        vc.wind_mps = rng.uniform(-4.0, 4.0)
        disp["wind_mps"] = vc.wind_mps
        dispersions_list.append(disp)
        jobs.append((_vcfg_to_dict(vc), mode, dll_path, seed + i))

    workers = workers or max(1, (os.cpu_count() or 2) - 1)
    with multiprocessing.Pool(workers) as pool:
        summaries = pool.map(_run_one, jobs)
    return list(zip(dispersions_list, summaries))


def _plot_grid(results, param, outpath):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    values = [v for v, s in results if "error" not in s]
    touchdown = [s.get("touchdown_speed_mps") or float("nan") for v, s in results if "error" not in s]
    tilt = [s["max_tilt_deg"] for v, s in results if "error" not in s]
    aborts = [1 if s.get("abort_reason") else 0 for v, s in results if "error" not in s]

    fig, axes = plt.subplots(3, 1, figsize=(8, 9), sharex=True)
    axes[0].plot(values, touchdown, "o-")
    axes[0].axhline(1.0, color="r", linestyle="--", alpha=0.5, label="TOUCHDOWN_VEL_MS")
    axes[0].set_ylabel("touchdown speed (m/s)")
    axes[0].legend(fontsize=8)
    axes[1].plot(values, tilt, "o-", color="orange")
    axes[1].set_ylabel("max tilt (deg)")
    axes[2].plot(values, aborts, "o-", color="red")
    axes[2].set_ylabel("aborted (1=yes)")
    axes[2].set_xlabel(param)
    fig.suptitle("Sweep: {}".format(param))
    fig.tight_layout()
    fig.savefig(outpath, dpi=120)
    print("wrote", outpath)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    g = sub.add_parser("grid", help="vary one parameter over a range")
    g.add_argument("--param", required=True, help="VehicleConfig field name")
    g.add_argument("--range", nargs=2, type=float, required=True, metavar=("LO", "HI"))
    g.add_argument("--n", type=int, default=9)
    g.add_argument("--mode", choices=["land", "chute"], default="land")
    g.add_argument("--out", default=None, help="Output PNG path")

    mc = sub.add_parser("monte-carlo", help="randomize all dispersions")
    mc.add_argument("--n", type=int, default=100)
    mc.add_argument("--mode", choices=["land", "chute"], default="land")
    mc.add_argument("--seed", type=int, default=42)
    mc.add_argument("--out", default=None, help="Output JSON path")

    args = ap.parse_args()
    dll = core.default_dll()

    if args.cmd == "grid":
        lo, hi = args.range
        values = [lo + i * (hi - lo) / max(1, args.n - 1) for i in range(args.n)]
        results = grid_sweep(args.param, values, mode=args.mode, dll_path=dll)
        for v, s in results:
            print("{}={:.4g}: touchdown={:.2f} m/s, max_tilt={:.1f} deg, abort={}".format(
                args.param, v, s.get("touchdown_speed_mps") or float("nan"),
                s.get("max_tilt_deg", float("nan")), s.get("abort_reason")))
        out = args.out or os.path.join(SIM_DIR, "sweep_{}.png".format(args.param))
        _plot_grid(results, args.param, out)

    elif args.cmd == "monte-carlo":
        results = monte_carlo(args.n, mode=args.mode, dll_path=dll, seed=args.seed)
        n_abort = sum(1 for _, s in results if s.get("abort_reason"))
        speeds = [s.get("touchdown_speed_mps") for _, s in results
                 if s.get("touchdown_speed_mps") is not None]
        print("{} runs: {} aborted ({:.0f}%)".format(len(results), n_abort,
                                                      100.0 * n_abort / len(results)))
        if speeds:
            print("touchdown speed: min={:.2f} mean={:.2f} max={:.2f} m/s".format(
                min(speeds), sum(speeds) / len(speeds), max(speeds)))
        out = args.out or os.path.join(SIM_DIR, "monte_carlo_results.json")
        with open(out, "w") as f:
            json.dump([{"dispersions": d, "summary": s} for d, s in results], f, indent=2)
        print("wrote", out)

    return 0


if __name__ == "__main__":
    sys.exit(main())
