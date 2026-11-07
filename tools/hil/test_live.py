#!/usr/bin/env python3
"""PC test of the closed-loop bench sim -- run this before a bench session.

Builds the GENERATED HIL sketch (make_hil.py --profile e18w_1400g, no Teensy
compile) into tools/hil/test/host_live.cpp: the real firmware running on the
PC in real time, with its two USB serial ports on TCP sockets. Each case
starts a fresh host_live on its own ports and flies it with live_run.py, the
same way the bench would, then checks:

  every case   the twin agrees (live_run.py exit 0) and the host never
               printed `landing gate ON` (LIVE never drives the e-match)
  chute        CHUTE_TEST flight, chute out at apogee, soft touchdown
  land         FULL_LANDING, landing motor fired, legs simulated: the leg
               gate is never driven
  dud          the landing motor never lights. FIRE_LANDING is issued and no
               thrust appears. NOTE: at this profile's fire point (~15.5 m,
               -7.4 m/s) the ground comes ~1.2 s after the fire command, before
               the DUD_IGNITER window closes (IGNITION_DELAY_MS + DUD_WINDOW_MS
               = 1.32 s), so the flight computer never aborts. The impact spike
               then reads as IGNITION_CONFIRMED. The table shows what happened.
  stuck        the first 2 latch releases don't free the canopy: 3 releases
               (1.9 s apart). NOTE: from an 18 m apogee the rocket is on the
               ground ~2 s after the first release, so the canopy never opens.
  imu-dropout  frames stop for 1 s in flight: IMU_FAIL abort, chute
  imu-frozen   the same IMU sample for 2 s: IMU_FAIL abort, chute
  op-abort     operator abort 1.5 s after launch: IMU_FAIL abort, chute
  live-legs    land mode with live legs: `legs gate ON` is printed

    python tools/hil/test_live.py
    python tools/hil/test_live.py --only land dud   # some cases
    python tools/hil/test_live.py --keep            # leave the profile build

At the end the plain HIL build is regenerated (as test_hil.py does), unless
--keep. Each case takes ~25 s of wall time (it runs in real time).
"""
import argparse
import json
import os
import re
import shutil
import socket
import subprocess
import sys
import time

HIL = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HIL)
import make_hil  # noqa: E402

PY = sys.executable
MSYS_GXX = r"C:\msys64\ucrt64\bin\g++.exe"
HOST_DIR = os.path.join(make_hil.BUILD_ROOT, "host")
PROFILE = "e18w_1400g"
BASE_PORT = 7740

# name, live_run.py args, checks on the saved summary
CASES = [
    ("chute", ["--mode", "chute"],
     {"abort": None, "final": "TOUCHDOWN", "releases": 1, "touchdown_max": 3.5}),
    ("land", ["--mode", "land"],
     {"abort": None, "final": "TOUCHDOWN", "releases": 0, "fired": True, "legs_gate": False}),
    # dud / stuck: with this profile the flight code can't recover in time (see
    # the module docstring), so these check what it does do, and the safety rules.
    ("dud", ["--mode", "land", "--set", "dud_landing_igniter=true"],
     {"fired": True, "thrust": False}),
    ("stuck", ["--mode", "chute", "--set", "chute_stuck_releases=2"],
     {"abort": None, "releases": 3}),
    ("imu-dropout", ["--mode", "land", "--set", "imu_dropout_at_s=1.5", "imu_dropout_s=1"],
     {"abort": "IMU_FAIL", "releases_min": 1, "fired": False}),
    ("imu-frozen", ["--mode", "land", "--set", "imu_frozen_at_s=1.5", "imu_frozen_s=2"],
     {"abort": "IMU_FAIL", "releases_min": 1, "fired": False}),
    ("op-abort", ["--mode", "land", "--abort-at", "1.5"],
     {"abort": "IMU_FAIL", "releases_min": 1, "fired": False}),
    ("live-legs", ["--mode", "land", "--live-legs"],
     {"abort": None, "final": "TOUCHDOWN", "fired": True, "legs_gate": True}),
]


def find_gxx():
    if os.path.isfile(MSYS_GXX):
        return MSYS_GXX
    g = shutil.which("g++")
    if not g:
        raise SystemExit("no g++: install MSYS2 ucrt64 (C:\\msys64\\ucrt64\\bin) or put g++ on PATH")
    return g


def build(sketch):
    gxx = find_gxx()
    exe = os.path.join(HOST_DIR, "host_live" + (".exe" if os.name == "nt" else ""))
    os.makedirs(HOST_DIR, exist_ok=True)
    env = dict(os.environ, PATH=os.path.dirname(gxx) + os.pathsep + os.environ["PATH"])
    cmd = [gxx, "-std=gnu++17", "-O2", "-Wall", "-Wno-unused-function",
           "-I", os.path.join(HIL, "test", "mock"), "-I", sketch,
           os.path.join(HIL, "test", "host_live.cpp"), os.path.join(HIL, "test", "net.cpp"),
           "-o", exe]
    if os.name == "nt":
        cmd.append("-lws2_32")
    subprocess.run(cmd, check=True, env=env)
    return exe, env


def port_free(p):
    for q in (p, p + 1):
        s = socket.socket()
        try:
            s.bind(("127.0.0.1", q))
        except OSError:
            return False
        finally:
            s.close()
    return True


def run_case(exe, env, idx, name, args, sketch):
    sd = os.path.join(HOST_DIR, "sd_live_" + name)
    if os.path.isdir(sd):
        shutil.rmtree(sd)
    os.makedirs(sd)
    port = BASE_PORT + 2 * idx
    while not port_free(port):
        port += 20
    host_out = os.path.join(sd, "host_stdout.txt")
    with open(host_out, "w") as hf:
        host = subprocess.Popen([exe, sd, str(port), "180"], stdout=hf, stderr=subprocess.STDOUT,
                                env=env)
        # wait for LISTENING
        end = time.time() + 10
        while time.time() < end:
            with open(host_out) as f:
                if "LISTENING" in f.read():
                    break
            time.sleep(0.05)
        cmd = [PY, os.path.join(HIL, "live_run.py"),
               "--port", "socket://127.0.0.1:{}".format(port + 1),
               "--console", "socket://127.0.0.1:{}".format(port), "--sketch", sketch] + args
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=240,
                               env=dict(os.environ, PYTHONIOENCODING="utf-8"))
            out, code = r.stdout + r.stderr, r.returncode
        except subprocess.TimeoutExpired as e:
            out, code = (e.stdout or "") if isinstance(e.stdout, str) else "", -1
        try:
            host.wait(timeout=10)
        except subprocess.TimeoutExpired:
            host.kill()
            host.wait()
    with open(os.path.join(sd, "live_run.txt"), "w", encoding="utf-8") as f:
        f.write(out)
    with open(host_out) as f:
        host_txt = f.read()
    return code, out, host_txt, sd


def check(name, want, code, out, host_txt):
    probs = []
    if code != 0:
        probs.append("live_run exit {} ({})".format(
            code, "twin differs" if code == 1 else "link/sequence error"))
    if "landing gate ON" in host_txt:
        probs.append("LANDING GATE DRIVEN")
    legs_on = "legs gate ON" in host_txt
    if "legs_gate" in want and legs_on != want["legs_gate"]:
        probs.append("legs gate {}".format("driven" if legs_on else "never driven"))
    m = re.search(r"^saved (.+)$", out, re.M)
    s = {}
    if m:
        with open(os.path.join(m.group(1).strip(), "result.json"), encoding="utf-8") as f:
            res = json.load(f)
        s = res["summary"]
        names = [e["name"] for e in res["events"]]
    else:
        probs.append("no session saved")
        names = []
    if s:
        if "abort" in want and s.get("abort_reason") != want["abort"]:
            probs.append("abort {} (want {})".format(s.get("abort_reason"), want["abort"]))
        if "final" in want and s.get("final_state") != want["final"]:
            probs.append("final state {}".format(s.get("final_state")))
        if "releases" in want and s.get("chute_releases") != want["releases"]:
            probs.append("chute releases {} (want {})".format(s.get("chute_releases"), want["releases"]))
        if "releases_min" in want and (s.get("chute_releases") or 0) < want["releases_min"]:
            probs.append("chute never released")
        if "fired" in want and ("FIRE_LANDING" in names) != want["fired"]:
            probs.append("FIRE_LANDING {}".format("issued" if "FIRE_LANDING" in names else "missing"))
        if "thrust" in want:
            lit = s.get("land_ignite_t") is not None
            if lit != want["thrust"]:
                probs.append("landing motor {}".format("lit" if lit else "never lit"))
        for ev in want.get("events", []):
            if ev not in names:
                probs.append(ev + " missing")
        td = s.get("touchdown_speed_mps")
        if "touchdown_max" in want and (td is None or td > want["touchdown_max"]):
            probs.append("touchdown {}".format(td))
    link = s.get("link", {})
    twin = re.search(r"(\d+) of (\d+) checks agree", out)
    info = {
        "abort": s.get("abort_reason") or "-",
        "releases": s.get("chute_releases"),
        "td": s.get("touchdown_speed_mps"),
        "apogee": s.get("apogee_h"),
        "twin": "{}/{}".format(*twin.groups()) if twin else "-",
        "link": link,
    }
    return probs, info


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--only", nargs="*", help="run only these cases")
    ap.add_argument("--keep", action="store_true", help="don't regenerate the plain HIL build")
    args = ap.parse_args()
    cases = [c for c in CASES if not args.only or c[0] in args.only]

    make_hil.generate(make_hil.load_profile(PROFILE))
    sketch = make_hil.SKETCH_DIR
    print("compiling host_live for the PC:", sketch, flush=True)
    results = []
    try:
        exe, env = build(sketch)
        for i, (name, cargs, want) in enumerate(cases):
            print("---- {} : live_run.py {}".format(name, " ".join(cargs)), flush=True)
            t0 = time.time()
            code, out, host_txt, sd = run_case(exe, env, i, name, cargs, sketch)
            probs, info = check(name, want, code, out, host_txt)
            if probs:
                print(out[-3000:])
            print("     {} in {:.0f} s{}".format("PASS" if not probs else "FAIL",
                                                time.time() - t0,
                                                "" if not probs else ": " + "; ".join(probs)),
                  flush=True)
            results.append((name, not probs, probs, info))
    finally:
        if not args.keep:
            make_hil.generate()

    print("\n==== closed-loop HIL PC test ({}) ====".format(PROFILE))
    print("{:<12}{:<6}{:<13}{:>4}{:>8}{:>9}{:>7}{:>8}{:>7}{:>7}{:>6}".format(
        "case", "", "abort", "rel", "apogee", "td m/s", "twin", "pkts", "gap ms", "stale",
        "batch"))
    for name, ok, probs, i in results:
        L = i["link"]
        print("{:<12}{:<6}{:<13}{:>4}{:>8}{:>9}{:>7}{:>8}{:>7.1f}{:>7}{:>6}".format(
            name, "PASS" if ok else "FAIL", i["abort"], str(i["releases"]),
            "{:.2f}".format(i["apogee"]) if i["apogee"] is not None else "-",
            "{:.2f}".format(i["td"]) if i["td"] is not None else "-", i["twin"],
            L.get("telemetry", 0), L.get("max_gap_ms", 0.0), L.get("max_stale", 0),
            L.get("max_batch", 0)))
        if probs:
            print("            " + "; ".join(probs))
    return 0 if results and all(ok for _, ok, _, _ in results) else 1


if __name__ == "__main__":
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    sys.exit(main())
