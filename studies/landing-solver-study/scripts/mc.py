"""Monte Carlo / batch driver for the landing-solver study (analysis only).

Reproduces tools/sim/sweep.py's monte_carlo() draws EXACTLY (same RNG order,
same DISPERSION_FIELDS imported from the workspace, same per-flight seed
seed+i), so a run here pairs flight-for-flight with sweep.py output. Adds:
  - truth vs. flight-software separation: the plant's nominal vehicle comes
    from --truth-overrides (default: the workspace's own unpatched config),
    while the flight DLL may be built with --fw-overrides (model-mismatch
    tests: the solver assumes the wrong mass/CdA/delay, the plant doesn't);
  - --plant k=v nominal plant changes applied before dispersion draws;
  - --ext extra dispersions drawn from an INDEPENDENT RNG stream so the
    standard draws stay identical and pairing is preserved;
  - --clean: dispersion-free flights (one per --plant-list entry).

usage:
  python mc.py --ws <ws>/RocketFC --seeds 7 11 13 --n 200 --out file.json
               [--fw-overrides '{"BURN_TABLE_MARGIN_M": "0.5f"}']
               [--plant landing_thrust_scale=1.1 baro_lag_s=0.05]
               [--ext landing_thrust_scale:0.92:1.08]
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


def _init_worker(sim_dir):
    sys.path.insert(0, sim_dir)
    os.chdir(sim_dir)


def _run_one(args):
    vdict, mode, dll, seed, oracle_ms = args
    import run as run_mod
    import vehicle
    vc = vehicle.VehicleConfig(**vdict)
    try:
        r = run_mod.simulate(vc, mode=mode, dll_path=dll, seed=seed,
                             record=False, oracle_fire_ms=oracle_ms)
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
                # multiplicative on top of any nominal --plant value
                setattr(vc, attr, getattr(vc, attr) * val)
                disp[attr] = val
    return jobs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ws", required=True, help="workspace RocketFC root")
    ap.add_argument("--seeds", type=int, nargs="*", default=[7])
    ap.add_argument("--n", type=int, default=200)
    ap.add_argument("--mode", default="land")
    ap.add_argument("--fw-overrides", default="{}",
                    help="JSON of config.h constants for the FLIGHT DLL")
    ap.add_argument("--plant", nargs="*", default=[],
                    help="k=v nominal plant/vehicle changes (truth side)")
    ap.add_argument("--ext", nargs="*", default=[],
                    help="attr:lo:hi extra multiplicative dispersions")
    ap.add_argument("--clean", action="store_true",
                    help="no dispersions: one flight per seed")
    ap.add_argument("--plant-list", default=None,
                    help="JSON list of {k: v} dicts; one clean flight each")
    ap.add_argument("--oracle-ms", default=None,
                    help="JSON list of [seed, i, fire_ms] (ws_oracle only)")
    ap.add_argument("--workers", type=int, default=19)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    sim_dir = os.path.join(os.path.abspath(args.ws), "tools", "sim")
    _init_worker(sim_dir)
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
    ext = []
    for e in args.ext:
        a, lo, hi = e.split(":")
        ext.append((a, float(lo), float(hi)))

    t0 = time.time()
    tasks, meta_rows = [], []
    if args.plant_list:
        for j, pv in enumerate(json.loads(args.plant_list)):
            for seed in args.seeds:
                vc = copy.deepcopy(base)
                for k, v in pv.items():
                    setattr(vc, k, v)
                tasks.append((vc, {"plant": pv}, seed, j, None))
    elif args.oracle_ms:
        # Re-create each listed flight's exact dispersions, then force fire.
        src = args.oracle_ms
        if src.startswith("@"):
            with open(src[1:], encoding="utf-8") as f:
                src = f.read()
        want = json.loads(src)
        cache = {}
        for seed, i, fire_ms in want:
            if seed not in cache:
                cache[seed] = build_jobs(base, sweep.DISPERSION_FIELDS, seed,
                                         max(args.n, i + 1), ext)
            vc, disp, s, _ = cache[seed][i]
            tasks.append((vc, dict(disp, oracle_fire_ms=fire_ms), s, i, fire_ms))
    else:
        for seed in args.seeds:
            if args.clean:
                tasks.append((copy.deepcopy(base), {}, seed, 0, None))
                continue
            for vc, disp, s, i in build_jobs(base, sweep.DISPERSION_FIELDS,
                                             seed, args.n, ext):
                tasks.append((vc, disp, s, i, None))

    jobs = [(dataclasses.asdict(vc), args.mode, flight_dll, s, om)
            for vc, _, s, _, om in tasks]
    with multiprocessing.Pool(args.workers, initializer=_init_worker,
                              initargs=(sim_dir,)) as pool:
        summaries = pool.map(_run_one, jobs, chunksize=1)

    rows = []
    for (vc, disp, s, i, om), summ in zip(tasks, summaries):
        rows.append({"seed": s - i if not args.plant_list else s, "i": i,
                     "flight_seed": s, "dispersions": disp, "summary": summ})
    meta = {"ws": os.path.abspath(args.ws), "flight_dll": flight_dll,
            "fw_overrides": fw, "plant": parse_kv(args.plant), "ext": args.ext,
            "seeds": args.seeds, "n": args.n, "clean": args.clean,
            "plant_list": args.plant_list, "elapsed_s": time.time() - t0}
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump({"meta": meta, "rows": rows}, f)
    nerr = sum(1 for r in rows if "error" in r["summary"])
    print(f"wrote {args.out}: {len(rows)} flights, {nerr} errors, "
          f"{meta['elapsed_s']:.1f} s")


if __name__ == "__main__":
    main()
