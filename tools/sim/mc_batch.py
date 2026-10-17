#!/usr/bin/env python3
"""Batch Monte Carlo / robustness driver for the closed-loop simulator.

Reproduces sweep.py's monte-carlo draws EXACTLY (same RNG order, same
DISPERSION_FIELDS, per-flight seed = seed + i), so its flights pair one-for-one
with `sweep.py monte-carlo` and with each other across solver builds. On top:

  --seeds 7 11 13       several seeds in one run (n flights each)
  --fw-overrides JSON   config.h constants for the FLIGHT build only; the plant
                        keeps the unpatched values -> "the software believes the
                        wrong mass/CdA/delay" tests
  --plant k=v ...       change the plant's nominal vehicle (VehicleConfig
                        fields, e.g. landing_thrust_scale=1.1 baro_q_coeff=0.1,
                        landing_eng_path="C:/curves/other.eng")
  --disp attr=w ...     narrow/widen one dispersion (same draws, rescaled), e.g.
                        mass_descent_kg=0.01 ignition_delay_s=0.05 ("measured")
  --ext attr:lo:hi ...  extra multiplicative dispersions from an independent
                        RNG stream (standard draws stay paired)
  --plant-list JSON     dispersion-free flights, one per {k: v} dict per seed
                        (e.g. an apogee sweep over ascent_eng_path)

Output: JSON {"meta": ..., "rows": [{seed, i, flight_seed, dispersions,
summary}]}; summaries include run.py's landing diagnostics (hop, fire state,
tilt-limited fire, abort phase, Kalman lag).

Run:
    python tools/sim/mc_batch.py --seeds 7 --n 200 --out mc_seed7.json
"""
import argparse
import copy
import dataclasses
import json
import multiprocessing
import os
import random
import sys
import time

SIM_DIR = os.path.dirname(os.path.abspath(__file__))


def _init_worker(sim_dir):
    sys.path.insert(0, sim_dir)
    os.chdir(sim_dir)


def _run_one(args):
    vdict, mode, dll, seed = args
    import run as run_mod
    import vehicle
    vc = vehicle.VehicleConfig(**vdict)
    try:
        r = run_mod.simulate(vc, mode=mode, dll_path=dll, seed=seed, record=False)
        return r["summary"]
    except Exception as e:  # keep the batch alive; reported in the output
        return {"error": repr(e)}


def parse_kv(items):
    out = {}
    for it in items or []:
        k, v = it.split("=", 1)
        try:
            out[k] = json.loads(v)
        except ValueError:
            out[k] = v
    return out


def build_jobs(base, disp_fields, seed, n, ext):
    """Standard draws exactly as sweep.monte_carlo; ext from its own stream."""
    rng = random.Random(seed)
    jobs = []
    for i in range(n):
        vc = copy.deepcopy(base)
        disp = {}
        for attr, pct in disp_fields:
            nominal = getattr(vc, attr)
            factor = 1.0 + rng.uniform(-pct, pct)
            setattr(vc, attr, nominal * factor)
            disp[attr] = factor
        vc.wind_mps = rng.uniform(-4.0, 4.0)
        disp["wind_mps"] = vc.wind_mps
        jobs.append([vc, disp, seed + i, i])
    if ext:
        rng2 = random.Random(seed * 7919 + 1)
        for vc, disp, _, _ in jobs:
            for attr, lo, hi in ext:
                val = rng2.uniform(lo, hi)
                setattr(vc, attr, getattr(vc, attr) * val)
                disp[attr] = val
    return jobs


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seeds", type=int, nargs="*", default=[7])
    ap.add_argument("--n", type=int, default=200)
    ap.add_argument("--mode", choices=["land", "chute"], default="land")
    ap.add_argument("--fw-overrides", default="{}")
    ap.add_argument("--plant", nargs="*", default=[])
    ap.add_argument("--disp", nargs="*", default=[])
    ap.add_argument("--ext", nargs="*", default=[])
    ap.add_argument("--plant-list", default=None)
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    _init_worker(SIM_DIR)
    import core
    import sweep
    import vehicle

    truth_dll = core.default_dll()
    fw = json.loads(args.fw_overrides)
    flight_dll = core.build_with_overrides(fw) if fw else truth_dll
    base = vehicle.default_vehicle_config(truth_dll)  # plant truth nominal
    for k, v in parse_kv(args.plant).items():
        if not hasattr(base, k):
            raise SystemExit("unknown VehicleConfig field: " + k)
        setattr(base, k, v)
    disp_fields = list(sweep.DISPERSION_FIELDS)
    for d in args.disp:
        a, w = d.split("=")
        idx = [i for i, (name, _) in enumerate(disp_fields) if name == a]
        if not idx:
            raise SystemExit("not a dispersion field: " + a)
        disp_fields[idx[0]] = (a, float(w))
    ext = []
    for e in args.ext:
        a, lo, hi = e.split(":")
        ext.append((a, float(lo), float(hi)))

    t0 = time.time()
    tasks = []
    if args.plant_list:
        for j, pv in enumerate(json.loads(args.plant_list)):
            for seed in args.seeds:
                vc = copy.deepcopy(base)
                for k, v in pv.items():
                    setattr(vc, k, v)
                tasks.append((vc, {"plant": pv}, seed, j))
    else:
        for seed in args.seeds:
            for vc, disp, s, i in build_jobs(base, disp_fields, seed, args.n, ext):
                tasks.append((vc, disp, s, i))

    jobs = [(dataclasses.asdict(vc), args.mode, flight_dll, s) for vc, _, s, _ in tasks]
    with multiprocessing.Pool(args.workers, initializer=_init_worker,
                              initargs=(SIM_DIR,)) as pool:
        summaries = pool.map(_run_one, jobs, chunksize=1)

    rows = [{"seed": s - i if not args.plant_list else s, "i": i, "flight_seed": s,
             "dispersions": disp, "summary": summ}
            for (vc, disp, s, i), summ in zip(tasks, summaries)]
    meta = {"flight_dll": flight_dll, "fw_overrides": fw, "plant": parse_kv(args.plant),
            "disp": args.disp, "ext": args.ext, "seeds": args.seeds, "n": args.n,
            "plant_list": args.plant_list, "elapsed_s": time.time() - t0}
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump({"meta": meta, "rows": rows}, f)
    nerr = sum(1 for r in rows if "error" in r["summary"])
    print(f"wrote {args.out}: {len(rows)} flights, {nerr} errors, {meta['elapsed_s']:.1f} s")


if __name__ == "__main__":
    main()
