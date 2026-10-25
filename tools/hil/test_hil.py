#!/usr/bin/env python3
"""PC test of the whole HIL chain -- run this before any hardware session.

Compiles the GENERATED HIL sketch (make_hil.py's copy of RocketFC.ino + src/
with the trace-playback sensors.h) for the PC against tools/hil/test/mock,
then drives it like an operator would (load, mode land, chute lock, arm, go)
and checks the SD files it writes with compare.py:

  nominal    events/gimbal/loop match the reference; the landing gate fires;
             the known injected ignition delay is measured back (+-15 ms)
  abort      the tilt-abort trace: chute released, landing gate never on
  op-abort   `abort` sent on the HIL port mid-COAST: IMU_FAIL abort, chute
             released, landing gate never on

    python tools/hil/test_hil.py
    python tools/hil/test_hil.py --sketch <dir>   # test another sketch copy
"""
import argparse
import os
import re
import shutil
import subprocess
import sys

import hil_trace
import make_hil

HIL = hil_trace.HIL_DIR
PY = sys.executable
GXX = r"C:\msys64\ucrt64\bin\g++.exe"
HOST_DIR = os.path.join(make_hil.BUILD_ROOT, "host")
IGN_MS = 437


def ensure_trace(name):
    base = os.path.join(HIL, "traces", name)
    if not os.path.isfile(base + ".json"):
        subprocess.run([PY, os.path.join(HIL, "make_trace.py"), "--preset", name],
                       check=True, stdout=subprocess.DEVNULL)
    return base


def build(sketch):
    exe = os.path.join(HOST_DIR, "host_hil.exe")
    os.makedirs(HOST_DIR, exist_ok=True)
    env = dict(os.environ, PATH=os.path.dirname(GXX) + os.pathsep + os.environ["PATH"])
    subprocess.run([GXX, "-std=gnu++17", "-O2", "-Wall", "-Wno-unused-function",
                    "-I", os.path.join(HIL, "test", "mock"), "-I", sketch,
                    os.path.join(HIL, "test", "host_main.cpp"), "-o", exe],
                   check=True, env=env)
    return exe, env


def run_case(exe, env, name, trace, abort_at=None):
    blob, meta = hil_trace.load(trace)
    sd = os.path.join(HOST_DIR, "sd_" + name)
    if os.path.isdir(sd):
        shutil.rmtree(sd)
    os.makedirs(sd)
    p = meta["pad_accel"]
    load = "load {} {} {} {:.6f} {:.6f} {:.6f} {:.6g}".format(
        meta["frames"], meta["crc32"], meta["mode"], p[0], p[1], p[2], meta["baro_var"])
    cmd = [exe, sd, trace + ".bin", load, str(meta["mode"]), str(IGN_MS)]
    if abort_at is not None:
        cmd.append(str(abort_at))
    r = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=900)
    host_log = os.path.join(sd, "host_console.txt")
    with open(host_log, "w") as f:
        f.write(r.stdout)
    if r.returncode != 0:
        tail = "\n".join(l for l in r.stdout.splitlines() if " P " not in l)[-1500:]
        return False, "host run failed (exit {}):\n{}\n(full log: {})".format(
            r.returncode, tail, host_log)
    flight = os.path.join(sd, "FLIGHTS", "flight_001.csv")
    imu = os.path.join(sd, "HIL", "IMU_000.BIN")
    return True, (flight, imu)


def compare(trace, flight, imu):
    r = subprocess.run([PY, os.path.join(HIL, "compare.py"), "--trace", trace,
                        "--flight", flight, "--imu", imu],
                       capture_output=True, text=True, env=dict(os.environ, PYTHONIOENCODING="utf-8"))
    return r.returncode == 0, r.stdout


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--sketch", help="sketch dir to test (default: regenerate make_hil's copy)")
    args = ap.parse_args()
    if args.sketch:
        sketch = os.path.abspath(args.sketch)
    else:
        make_hil.generate()
        sketch = make_hil.SKETCH_DIR
    nominal, abort = ensure_trace("nominal"), ensure_trace("abort")
    print("compiling the HIL sketch for the PC:", sketch)
    exe, env = build(sketch)

    results = []

    ok, out = run_case(exe, env, "nominal", nominal)
    if ok:
        ok, rep = compare(nominal, *out)
        m = re.search(r"onset (-?\d+) ms after FIRE_LANDING", rep)
        got = int(m.group(1)) if m else None
        ign_ok = got is not None and abs(got - IGN_MS) <= 15
        print(rep)
        out = "compare {}; ignition delay measured {} ms vs {} injected".format(
            "PASS" if ok else "FAIL", got, IGN_MS)
        ok = ok and ign_ok
    results.append(("nominal", ok, out))

    ok, out = run_case(exe, env, "abort", abort)
    if ok:
        ok, rep = compare(abort, *out)
        print(rep)
        ok = ok and "chute latch:  release" in rep and "landing gate: never on" in rep
        out = "compare + chute released + landing gate never on: {}".format("PASS" if ok else "FAIL")
    results.append(("abort", ok, out))

    ok, out = run_case(exe, env, "op_abort", nominal, abort_at=7.5)
    if ok:
        import logfile  # tools/sim, on sys.path via hil_trace
        log = logfile.read_log(out[0])
        names = [e["name"] for e in log["events"]]
        fired = any(r["pyro"] & 2 for r in log["rows"])
        released = any(r["pyro"] & 1 for r in log["rows"])
        ok = "ABORT" in names and "FIRE_LANDING" not in names and not fired and released
        out = "events {} -> {}".format(" ".join(names), "PASS" if ok else "FAIL")
    results.append(("op-abort", ok, out))

    print("\n==== HIL PC test ====")
    for name, ok, msg in results:
        print("{:<10}{}  {}".format(name, "PASS" if ok else "FAIL", msg))
    return 0 if all(ok for _, ok, _ in results) else 1


if __name__ == "__main__":
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    sys.exit(main())
