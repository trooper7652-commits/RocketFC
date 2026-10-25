#!/usr/bin/env python3
"""Make a HIL trace from one closed-loop sim flight, plus its expected answer.

Writes, into tools/hil/traces/ (or --out):
    <name>.bin / .json   the trace the Teensy plays (see hil_trace.py)
    <name>_sim.csv       the sim's own flight log (logger.h format), for context
    <name>_ref.csv       the PC replay of the trace through the same flight
                         core -- what the Teensy's FLIGHTS/flight_NNN.csv
                         should look like (tools/hil/compare.py checks it)

Then it runs the PC loopback check: the replayed events must match the sim's
own events. If they don't, the trace is incomplete and must not be used.

Presets (both on today's src/config.h, calm-wind vehicle defaults otherwise):
    nominal  powered landing: LAUNCH, APOGEE, FIRE_LANDING, LEGS_BURN_ON, ...
    abort    4 m/s wind, tilt abort in DESCENT: the landing motor must NOT
             fire and the chute latch must release

Usage:
    python tools/hil/make_trace.py                 # nominal
    python tools/hil/make_trace.py --preset abort
    python tools/hil/make_trace.py --seed 3 --wind 2.0 --name windy
"""
import argparse
import datetime
import os
import sys

import hil_trace  # puts tools/sim on sys.path

import core  # noqa: E402
import logfile  # noqa: E402
import run  # noqa: E402
import vehicle  # noqa: E402

PRESETS = {
    "nominal": {"wind": 0.0, "seed": 1},
    "abort": {"wind": 4.0, "seed": 1},
}
TRACES_DIR = os.path.join(hil_trace.HIL_DIR, "traces")
LOOPBACK_TOL_S = 0.02


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--preset", choices=sorted(PRESETS), default="nominal")
    ap.add_argument("--name", help="output base name (default: the preset)")
    ap.add_argument("--seed", type=int)
    ap.add_argument("--wind", type=float, help="steady wind, m/s")
    ap.add_argument("--mode", choices=["land", "chute"], default="land")
    ap.add_argument("--countdown", type=float, default=3.0,
                    help="seconds of armed pad time in the trace before ignition")
    ap.add_argument("--out", default=TRACES_DIR)
    args = ap.parse_args()

    p = dict(PRESETS[args.preset])
    if args.seed is not None:
        p["seed"] = args.seed
    if args.wind is not None:
        p["wind"] = args.wind
    name = args.name or args.preset
    os.makedirs(args.out, exist_ok=True)
    base = os.path.join(args.out, name)

    dll = core.default_dll()
    vcfg = vehicle.default_vehicle_config(dll)
    vcfg.wind_mps = p["wind"]
    print("simulating: mode={} seed={} wind={} m/s".format(args.mode, p["seed"], p["wind"]))
    sim = run.simulate(vcfg, mode=args.mode, dll_path=dll, seed=p["seed"],
                       countdown_s=args.countdown, record=True, record_inputs=True)
    s, arm = sim["summary"], sim["arm_info"]
    if arm is None:
        raise SystemExit("the sim never armed -- no trace")
    blob = hil_trace.frames_from_inputs(sim["inputs"])
    n = len(blob) // hil_trace.FRAME.size
    if n > hil_trace.MAX_FRAMES:
        raise SystemExit("trace is {} frames; the Teensy buffer holds {}".format(
            n, hil_trace.MAX_FRAMES))

    sim_events = hil_trace.event_list(
        [dict(e, ms=e["t"] * 1000.0) for e in sim["events"]], arm["arm_ms"])
    fire = [t for nm, t in sim_events if nm == "FIRE_LANDING"]
    meta = {
        "name": name,
        "created": datetime.datetime.now().isoformat(timespec="seconds"),
        "source": "tools/sim run.simulate, seed {}, wind {} m/s".format(p["seed"], p["wind"]),
        "mode": int(core.FlightMode.FULL_LANDING if args.mode == "land"
                    else core.FlightMode.CHUTE_TEST),
        "mode_name": "FULL_LANDING" if args.mode == "land" else "CHUTE_TEST",
        "pad_accel": list(arm["pad_accel"]),
        "baro_var": arm["baro_var"],
        "dt": hil_trace.FAST_DT,
        "launch_s": (arm["launch_ms"] - arm["arm_ms"]) / 1000.0,
        "fire_s": fire[0] if fire else None,
        "sim_summary": {k: s.get(k) for k in (
            "apogee_h", "fire_t", "touchdown_speed_mps", "abort_reason",
            "abort_state", "legs_on_t")},
        "sim_events": sim_events,
    }
    meta = hil_trace.save(base, blob, meta)
    logfile.write_log(base + "_sim.csv", sim)

    # ---- PC reference replay + loopback check ----
    ref = hil_trace.replay(dll, blob, meta)
    logfile.write_log(base + "_ref.csv", ref)
    ref_events = hil_trace.event_list(ref["events"], ref["go_ms"])
    meta["ref_events"] = ref_events
    meta["ref_go_ms"] = ref["go_ms"]
    hil_trace.save(base, blob, meta)

    a = [(nm, t) for nm, t in sim_events if nm != "ARM"]
    b = [(nm, t) for nm, t in ref_events if nm != "ARM" and t >= 0.0]
    ok = [nm for nm, _ in a] == [nm for nm, _ in b[:len(a)]]
    print("\n{:<22}{:>10}{:>10}{:>9}".format("event", "sim s", "replay s", "diff ms"))
    for i in range(max(len(a), len(b))):
        sa = a[i] if i < len(a) else ("-", None)
        sb = b[i] if i < len(b) else ("-", None)
        d = (sb[1] - sa[1]) * 1000.0 if sa[1] is not None and sb[1] is not None else None
        if d is not None and abs(d) > LOOPBACK_TOL_S * 1000.0:
            ok = False
        print("{:<22}{:>10}{:>10}{:>9}  {}".format(
            sa[0], "" if sa[1] is None else "{:.3f}".format(sa[1]),
            "" if sb[1] is None else "{:.3f}".format(sb[1]),
            "" if d is None else "{:+.0f}".format(d),
            "" if sa[0] == sb[0] else "<- replay: " + sb[0]))
    print("\nframes {} ({:.1f} s), crc {}, launch at +{:.2f} s, fire at {}".format(
        n, n * hil_trace.FAST_DT, meta["crc32"], meta["launch_s"],
        "+{:.2f} s".format(meta["fire_s"]) if meta["fire_s"] is not None else "never"))
    print("wrote", base + ".bin/.json, _sim.csv, _ref.csv")
    if not ok:
        print("\nLOOPBACK FAILED: the replay does not reproduce the sim's events "
              "(tolerance {:.0f} ms). Don't use this trace.".format(LOOPBACK_TOL_S * 1000))
        return 1
    print("loopback OK: the replay reproduces every sim event.")
    return 0


if __name__ == "__main__":
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    sys.exit(main())
