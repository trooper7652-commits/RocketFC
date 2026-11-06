"""Robustness study: dud landing igniter (FIRE_LANDING issued, motor never lights).
Does the chute still save the rocket at the fix's 19.5 m apogee?"""
import json, multiprocessing, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
N, SEED = 200, 101


def _init(sim):
    sys.path.insert(0, sim)
    sys.path.insert(0, os.path.dirname(sim))
    os.chdir(sim)


def _one(args):
    vdict, dll, seed = args
    import run, vehicle
    try:
        return run.simulate(vehicle.VehicleConfig(**vdict), mode="land", dll_path=dll, seed=seed,
                            record=False, dud_landing_igniter=True)["summary"]
    except Exception as e:
        return {"error": repr(e)}


def batch(ws, plant, fw):
    sim = os.path.join(HERE, "ws", ws, "tools", "sim")
    _init(sim)
    import core, vehicle, mc_batch, sweep, dataclasses
    truth = core.default_dll()
    dll = core.build_with_overrides(fw) if fw else truth
    base = vehicle.default_vehicle_config(truth)
    for k, v in plant.items():
        setattr(base, k, v)
    disp = [(a, {"mass_descent_kg": 0.01, "ignition_delay_s": 0.05}.get(a, w)) for a, w in sweep.DISPERSION_FIELDS]
    jobs = [(dataclasses.asdict(vc), dll, s) for vc, _, s, _ in mc_batch.build_jobs(base, disp, SEED, N, [])]
    with multiprocessing.Pool(19, initializer=_init, initargs=(sim,)) as pool:
        out = pool.map(_one, jobs, chunksize=1)
    os.chdir(HERE)
    return out


def report(label, rows):
    rows = [s for s in rows if "error" not in s]
    q = lambda a, p: sorted(a)[int(p * (len(a) - 1))] if a else float("nan")
    fire_h = [s["fire_h_true"] for s in rows if s.get("fire_h_true") is not None]
    opened = [s for s in rows if s.get("chute_detected_t") is not None and s.get("touchdown_t") is not None
              and s["chute_detected_t"] < s["touchdown_t"]]
    vimp = [s.get("contact_vspeed_mps") or s.get("touchdown_speed_mps") or 0.0 for s in rows]
    dud = sum(1 for s in rows if s.get("abort_reason") == "DUD_IGNITER")
    print("| %s | %d | %.1f m | %d %% | %d %% | %.1f / %.1f m/s |" % (
        label, len(rows), q(fire_h, 0.5), 100 * dud // max(len(rows), 1), 100 * len(opened) // max(len(rows), 1),
        q(vimp, 0.5), q(vimp, 0.9)), flush=True)


if __name__ == "__main__":
    c = json.load(open(os.path.join(HERE, "combos.json")))["c1_e12_e12"]
    P = {"ascent_eng_path": c["ascent_eng"], "landing_eng_path": c["landing_eng"], "legs_cut_s": 0.75,
         "ascent_casing_eject_kg": 0.0391}
    FW = {"LEGS_DELAY_MS": "0.0f", "BURN_TABLE_MARGIN_M": "1.0f"}
    runs = [("fix (19.5 m apogee), dud window 700 ms", "c1_ej_b80", P, FW),
            ("fix, dud window 300 ms", "c1_ej_b80", P, dict(FW, DUD_WINDOW_MS="300.0f")),
            ("today's F10 + F15 (73 m apogee), 700 ms", "base", {}, {})]
    if len(sys.argv) > 1:   # one run per process: each workspace has its own sim modules
        label, ws, plant, fw = runs[int(sys.argv[1])]
        report(label, batch(ws, plant, fw))
    else:
        import subprocess
        print("| Dud landing igniter | flights | fire height (median) | dud detected | canopy open before ground | impact median / p90 |")
        print("|---|---|---|---|---|---|", flush=True)
        for i in range(len(runs)):
            subprocess.run([sys.executable, os.path.abspath(__file__), str(i)], check=True)
