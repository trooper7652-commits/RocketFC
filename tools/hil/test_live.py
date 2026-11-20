#!/usr/bin/env python3
"""PC test of the closed-loop bench sim -- run this before a bench session.

Builds the GENERATED HIL sketch (make_hil.py --profile e18w_1400g, no Teensy
compile) into tools/hil/test/host_live.cpp: the real firmware running on the
PC in real time, with its two USB serial ports on TCP sockets. Each case
starts a fresh host_live on its own ports and flies it with live_run.py, the
same way the bench would, then checks:

  every case   the twin agrees (live_run.py exit 0); the landing e-match
               gate is driven ONLY in the live-landing cases and the leg
               gate only in the live-legs cases (simulated channels never
               drive their pin). Every run uses the Run sequence (Prepare ->
               Arm -> countdown -> Launch), and the simulated nozzle follows
               the 50 Hz servo writes (the twin replays the same hold).
  chute        CHUTE_TEST flight, chute out at apogee, soft touchdown
  land         FULL_LANDING, landing motor fired, legs simulated: the leg
               gate is never driven
  dud          the landing motor never lights. FIRE_LANDING is issued and no
               thrust appears. The fire point is ~15.5 m at -7.4 m/s and the
               ground comes ~1.2 s after the command; the DUD_IGNITER window
               closes at IGNITION_DELAY_MS + DUD_WINDOW_MS = 0.82 s (~6 m up):
               DUD_IGNITER abort, chute latch released before impact.
  stuck        the first 2 latch releases don't free the canopy: the fast
               wiggle's 3rd release (~0.7 s after the first) does, so the
               canopy is out and confirmed (3 releases).
  imu-dropout  frames stop for 1 s in flight: IMU_FAIL abort, chute
  imu-frozen   the same IMU sample for 2 s: IMU_FAIL abort, chute
  op-abort     operator abort 1.5 s after launch: IMU_FAIL abort, chute
  live-legs    land mode with live legs: `legs gate ON` is printed
  live-landing land mode with the live landing e-match: `landing gate ON`
               is printed at FIRE_LANDING, the leg gate never
  live-both    both pyro channels live: both gates
  randomize    the sim's Monte Carlo spreads (seed 3): the twin agrees
  reboot       chute mode, then `reboot` after the flight log closes; the
               restarted host boots the chute latch where it was (SET),
               not OPEN

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
    # chute / reboot: a healthy latch is confirmed during the fast wiggle, so
    # its next stroke may already count as a release (1 .. 1 + CHUTE_WIG_CYCLES)
    ("chute", ["--mode", "chute"],
     {"abort": None, "final": "TOUCHDOWN", "releases_min": 1, "releases_max": 3,
      "events": ["CHUTE_DETECTED"], "touchdown_max": 3.5}),
    ("land", ["--mode", "land"],
     {"abort": None, "final": "TOUCHDOWN", "releases": 0, "fired": True, "legs_gate": False}),
    ("dud", ["--mode", "land", "--set", "dud_landing_igniter=true"],
     {"abort": "DUD_IGNITER", "releases_min": 1, "fired": True, "thrust": False}),
    ("stuck", ["--mode", "chute", "--set", "chute_stuck_releases=2"],
     {"abort": None, "releases": 3, "events": ["CHUTE_DETECTED"]}),
    ("imu-dropout", ["--mode", "land", "--set", "imu_dropout_at_s=1.5", "imu_dropout_s=1"],
     {"abort": "IMU_FAIL", "releases_min": 1, "fired": False}),
    ("imu-frozen", ["--mode", "land", "--set", "imu_frozen_at_s=1.5", "imu_frozen_s=2"],
     {"abort": "IMU_FAIL", "releases_min": 1, "fired": False}),
    ("op-abort", ["--mode", "land", "--abort-at", "1.5"],
     {"abort": "IMU_FAIL", "releases_min": 1, "fired": False}),
    ("live-legs", ["--mode", "land", "--live-legs"],
     {"abort": None, "final": "TOUCHDOWN", "fired": True, "legs_gate": True}),
    ("live-landing", ["--mode", "land", "--live-landing"],
     {"abort": None, "final": "TOUCHDOWN", "fired": True, "landing_gate": True,
      "legs_gate": False}),
    ("live-both", ["--mode", "land", "--live-legs", "--live-landing"],
     {"abort": None, "final": "TOUCHDOWN", "fired": True, "landing_gate": True,
      "legs_gate": True}),
    ("randomize", ["--mode", "land", "--randomize", "--seed", "3"], {}),
    ("reboot", ["--mode", "chute", "--reboot-after"],
     {"abort": None, "releases_min": 1, "releases_max": 3, "chute_kept": "SET (loading)"}),
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
    if "--reboot-after" in args:
        host_txt += "\n" + reboot_boot_console(exe, env, sd, port)
    return code, out, host_txt, sd


def reboot_boot_console(exe, env, sd, port):
    """After a HIL `reboot` the PC host has exited (the Teensy would reset).
    Start it again on the same SD/EEPROM folder, let it boot, and return its
    boot console -- the chute latch line says where the latch came up."""
    out = os.path.join(sd, "host_stdout_after_reboot.txt")
    with open(out, "w") as hf:
        host = subprocess.Popen([exe, sd, str(port), "15"], stdout=hf, stderr=subprocess.STDOUT,
                                env=env)
        time.sleep(0.5)
        socks = []
        try:
            for p in (port, port + 1):
                socks.append(socket.create_connection(("127.0.0.1", p), timeout=5))
            time.sleep(2.5)            # boot report
        finally:
            for s in socks:
                s.close()
        try:
            host.wait(timeout=10)
        except subprocess.TimeoutExpired:
            host.kill()
            host.wait()
    with open(os.path.join(sd, "host_console.txt"), encoding="utf-8", errors="replace") as f:
        boot = [ln for ln in f.read().splitlines() if "chute latch:" in ln]
    return "AFTER REBOOT: " + (boot[0] if boot else "(no chute latch line)")


def check(name, want, code, out, host_txt):
    probs = []
    if code != 0:
        probs.append("live_run exit {} ({})".format(
            code, "twin differs" if code == 1 else "link/sequence error"))
    land_on = "landing gate ON" in host_txt
    if land_on != want.get("landing_gate", False):
        probs.append("LANDING GATE DRIVEN" if land_on else "landing gate never driven")
    legs_on = "legs gate ON" in host_txt
    if legs_on != want.get("legs_gate", False):
        probs.append("legs gate {}".format("driven" if legs_on else "never driven"))
    if "chute_kept" in want:
        m = re.search(r"AFTER REBOOT: (.*)", host_txt)
        line = m.group(1) if m else ""
        if want["chute_kept"] not in line or "restored last position" not in line:
            probs.append("after reboot: " + (line or "no boot report"))
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
        if "releases_max" in want and (s.get("chute_releases") or 0) > want["releases_max"]:
            probs.append("chute releases {} (max {})".format(s.get("chute_releases"), want["releases_max"]))
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
    print("{:<14}{:<6}{:<13}{:>4}{:>8}{:>9}{:>7}{:>8}{:>7}{:>7}{:>6}".format(
        "case", "", "abort", "rel", "apogee", "td m/s", "twin", "pkts", "gap ms", "stale",
        "batch"))
    for name, ok, probs, i in results:
        L = i["link"]
        print("{:<14}{:<6}{:<13}{:>4}{:>8}{:>9}{:>7}{:>8}{:>7.1f}{:>7}{:>6}".format(
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
