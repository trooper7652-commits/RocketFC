#!/usr/bin/env python3
"""Closed-loop bench flight from the command line: the Teensy (HIL build,
LIVE mode) flies the simulated rocket; afterwards the pure-sim twin flies the
same scenario and the two are compared. The dashboard's "HIL bench" window
(tools/sim/dashboard.py) does the same with live plots and the 3D view.

    python tools/hil/make_hil.py --profile e18w_1400g --upload COM7   # once
    python tools/hil/live_run.py --mode chute
    python tools/hil/live_run.py --mode land --set dud_landing_igniter=true
    python tools/hil/live_run.py --mode land --set imu_frozen_at_s=3 imu_frozen_s=2

Ports are found automatically (or --port/--console; socket://127.0.0.1:N for
the PC build, tools/hil/test/host_live.cpp). Sequence: stream a still pad for
--pad-s, Prepare (pad calibration, `mode`, `chute lock`), `arm` (~4 s), wait
--countdown-s, Launch, fly until landed. Ctrl-C sends the operator abort.

--live-legs really powers the leg nichrome when the flight computer commands
it (you must type LEGS); otherwise both pyro channels are simulated and no
pyro gate pin is ever driven. Each run is saved to tools/hil/sessions/.
Exit code: 0 if the twin agrees, 1 if it differs, 2 on a link/sequence error.
"""
import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "live"))

import runner as runner_mod  # noqa: E402
from scenario import Scenario  # noqa: E402


def parse_set(items, scen):
    for it in items or []:
        k, v = it.split("=", 1)
        if not hasattr(scen, k):
            raise SystemExit("unknown scenario field: {} (see tools/hil/live/scenario.py)".format(k))
        cur = getattr(scen, k)
        if isinstance(cur, bool):
            v = v.lower() in ("1", "true", "yes", "on")
        else:
            v = type(cur)(json.loads(v))
        setattr(scen, k, v)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="HIL channel (COMx or socket://host:port)")
    ap.add_argument("--console", help="RocketFC console (COMx or socket://host:port)")
    ap.add_argument("--mode", choices=["chute", "land"], default=None)
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--scenario", help="scenario JSON (tools/hil/live/scenario.py fields)")
    ap.add_argument("--set", nargs="*", default=[], metavar="FIELD=VALUE")
    ap.add_argument("--live-legs", action="store_true",
                    help="really power the leg nichrome (asks you to type LEGS)")
    ap.add_argument("--pad-s", type=float, default=3.0)
    ap.add_argument("--countdown-s", type=float, default=1.0)
    ap.add_argument("--abort-at", type=float, default=None,
                    help="send the operator abort this many s after launch")
    ap.add_argument("--reboot-after", action="store_true",
                    help="reboot the Teensy when done (ready for the next run)")
    ap.add_argument("--no-twin", action="store_true")
    ap.add_argument("--sketch", default=None, help="HIL build tree (default tools/hil/build/RocketFC_HIL)")
    args = ap.parse_args()

    scen = Scenario.load(args.scenario) if args.scenario else Scenario()
    if args.mode:
        scen.mode = args.mode
    if args.seed is not None:
        scen.seed = args.seed
    parse_set(args.set, scen)
    if args.live_legs:
        scen.live_legs = True
    if scen.live_legs and not (args.port or "").startswith("socket://"):
        print("LIVE LEGS: the leg nichrome WILL be powered when the flight computer "
              "commands it (pyro LiPo + band on the bench, clear of anything).")
        if input("Type LEGS to continue: ").strip() != "LEGS":
            raise SystemExit("not confirmed")

    kw = {"sketch_dir": args.sketch} if args.sketch else {}
    r = runner_mod.Runner(args.port, args.console, scen, **kw)
    print("profile:", r.profile.get("name"), "| scenario:", ", ".join(scen.faults()),
          "| mode", scen.mode, "| seed", scen.seed, flush=True)
    try:
        r.start()
        step = "pad"
        t_step = time.time()
        while not r.flight_over:
            r.poll()
            now = time.time()
            if r.error:
                raise runner_mod.RunnerError(r.error)
            if step == "pad" and now - t_step >= args.pad_s:
                r.prepare()
                step = "preparing"
            elif step == "preparing" and r.phase == "prepared" and r.task is None:
                r.arm()
                step = "arming"
            elif step == "arming" and r.phase == "armed" and r.state == "ARMED":
                step = "countdown"
                t_step = now
            elif step == "countdown" and now - t_step >= args.countdown_s:
                r.launch()
                step = "flight"
                t_step = now
            elif (step == "flight" and args.abort_at is not None and r.fl.launch_t is not None
                  and r.t - r.fl.launch_t >= args.abort_at):
                r.abort()
                step = "aborted"
    except KeyboardInterrupt:
        print("\nCtrl-C: operator abort", flush=True)
        try:
            r.abort()
            end = time.time() + 1.0
            while time.time() < end:
                r.poll()
        except Exception:
            pass
        r.finish(save=True)
        return 2
    except runner_mod.RunnerError as e:
        print("ERROR:", e, flush=True)
        try:
            r.finish(save=bool(r.ticks))
        except Exception:
            r.close()
        return 2

    res, path = r.finish(save=True, close=not args.reboot_after)
    if args.reboot_after:
        r.reboot()
        time.sleep(0.2)
        r.close()
    s = res["summary"]
    print("link: {telemetry} packets, max gap {max_gap_ms:.1f} ms, max stale {max_stale} ticks, "
          "max batch {max_batch}, crc errors {crc_errors}".format(**r.stats))
    o = s.get("landing_outcome") or {}
    print("apogee {:.2f} m | touchdown {} | abort {} | chute releases {} | {}".format(
        s.get("apogee_h") or 0.0,
        "{:.2f} m/s".format(s["touchdown_speed_mps"]) if s.get("touchdown_speed_mps") is not None
        else "none", s.get("abort_reason") or "none", s.get("chute_releases"),
        o.get("label", "")))
    if args.no_twin:
        return 0
    _, rows, text = runner_mod.run_twin(path)
    print("\ntwin comparison (" + path + "):\n" + text)
    return 0 if all(r_[5] for r_ in rows) else 1


if __name__ == "__main__":
    sys.exit(main())
