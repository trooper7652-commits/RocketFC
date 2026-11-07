#!/usr/bin/env python3
"""Generate, compile and (optionally) upload the HIL ground-test firmware.

*** The result is NOT flight firmware. Reflash RocketFC before any flight. ***

Never edits the real flight code. Each run rebuilds a throwaway copy:

    tools/hil/build/RocketFC_HIL/
        RocketFC_HIL.ino        <- RocketFC.ino, unchanged
        src/...                 <- src/, unchanged, except:
        src/hw/sensors_real.h   <- the real sensors.h, `class Sensors` renamed
                                   to `class RealSensors` (nothing else)
        src/hw/sensors.h        <- tools/hil/firmware/sensors_hil.h, the
                                   trace-playback / LIVE wrapper around
                                   RealSensors
        src/hw/hil_hooks.h      <- tools/hil/firmware/hil_hooks.h
        src/hw/actuators.h      <- the real one + three hook calls (landing
                                   gate, leg gate, pyro sense volts) that only
                                   act in LIVE mode
        hil_profile.json        <- the motor/mass profile this build carries

Because the copy is regenerated from the working tree every time, the HIL
build can't drift from the flight code: whatever RocketFC is today is what
gets tested.

A profile (tools/hil/profiles/<name>.json) changes ONLY the build copy: its
"config" constants are patched into src/config.h and src/core/burn_table.h
is regenerated for its landing motor, so the Teensy flies e.g. the E18W/E18W
1.4 kg rocket while the real config.h stays as it is. The closed-loop PC side
(tools/hil/live/) reads hil_profile.json for the matching plant.

Usage:
    python tools/hil/make_hil.py                       # generate + compile
    python tools/hil/make_hil.py --profile e18w_1400g  # ... flying a profile
    python tools/hil/make_hil.py --upload COM7         # ... and upload
    python tools/hil/make_hil.py --no-compile          # generate only
"""
import json
import argparse
import os
import re
import shutil
import subprocess
import sys

HIL_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(os.path.dirname(HIL_DIR))
SKETCH_NAME = "RocketFC_HIL"
BUILD_ROOT = os.path.join(HIL_DIR, "build")
SKETCH_DIR = os.path.join(BUILD_ROOT, SKETCH_NAME)
FQBN = "teensy:avr:teensy41"
BOARD_OPTIONS = "usb=serial2"  # Dual Serial: CLI on port 1, HIL control on port 2
ARDUINO_CLI = os.path.join(os.environ.get("LOCALAPPDATA", ""), "Programs",
                           "arduino-cli", "arduino-cli.exe")


def source_desc():
    """'RocketFC <short hash>[ +uncommitted]' for the boot banner."""
    def git(*args):
        return subprocess.run(["git", "-C", REPO_ROOT, *args], capture_output=True,
                              text=True, check=True).stdout.strip()
    try:
        h = git("rev-parse", "--short", "HEAD")
        dirty = git("status", "--porcelain", "--", "src", "RocketFC.ino")
    except (OSError, subprocess.CalledProcessError):
        return "RocketFC (no git info)"
    return "RocketFC {}{}".format(h, " +uncommitted src changes" if dirty else "")


def mirror(src, dst, extra=()):
    """Make dst an exact copy of src, in place. (Deleting and recreating the
    tree fails under OneDrive, which briefly locks folders it is syncing.)
    `extra`: generated files under dst to keep."""
    shutil.copytree(src, dst, dirs_exist_ok=True)
    for root, _, files in os.walk(dst):
        for f in files:
            p = os.path.join(root, f)
            rel = os.path.relpath(p, dst)
            if rel not in extra and not os.path.exists(os.path.join(src, rel)):
                os.remove(p)


PROFILES_DIR = os.path.join(HIL_DIR, "profiles")
PY = sys.executable


def _sub_once(text, old, new, what):
    if text.count(old) != 1:
        raise SystemExit("make_hil: can't patch {} ({} matches) -- update make_hil.py "
                         "to match src/hw/actuators.h".format(what, text.count(old)))
    return text.replace(old, new)


def patch_actuators(path):
    """Hook the landing gate, the flight leg gate and the pyro sense volts
    (tools/hil/firmware/hil_hooks.h). Outside LIVE mode they pass through."""
    with open(path, encoding="utf-8") as f:
        t = f.read().replace("\r\n", "\n")
    t = _sub_once(t, '#include "../config.h"\n',
                  '#include "../config.h"\n#include "hil_hooks.h"  // HIL build only\n',
                  "the config include")
    t = _sub_once(t, "pyroEnabled_ && (!cfg::REQUIRE_ARM_SWITCH || armSwitchOn())) {\n"
                     "      digitalWrite(cfg::PIN_PYRO_LEGS, HIGH);",
                  "pyroEnabled_ && (!cfg::REQUIRE_ARM_SWITCH || armSwitchOn())) {\n"
                  "      hilLegsGate(HIGH);  // HIL: real only with live legs",
                  "the flight leg gate")
    t = _sub_once(t, "    if (cfg::REQUIRE_ARM_SWITCH && !armSwitchOn()) return false;\n"
                     "    return gateOn(ms);",
                  "    if (cfg::REQUIRE_ARM_SWITCH && !armSwitchOn()) return false;\n"
                  "    return g_hilLive ? hilTimedFire(ms) : gateOn(ms);  // HIL: LIVE never drives the gate",
                  "the landing fire")
    t = _sub_once(t, "  bool gateOn(uint32_t ms) {\n",
                  "  bool hilTimedFire(uint32_t ms) {  // HIL LIVE: time the pulse, pin stays LOW\n"
                  "    fireEndMs_ = ms + (uint32_t)cfg::PYRO_FIRE_MS;\n"
                  "    if (fireEndMs_ == 0) fireEndMs_ = 1;\n"
                  "    return true;\n"
                  "  }\n"
                  "  bool gateOn(uint32_t ms) {\n",
                  "gateOn")
    for fn in ("contVolts", "legsContVolts"):
        t = _sub_once(t, "  float {}() const {{\n".format(fn),
                      "  float {}() const {{\n"
                      "    if (hilSimPyros()) return kHilSimPyroVolts;  // HIL: simulated pyros\n".format(fn),
                      fn)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(t)


def load_profile(name):
    path = os.path.join(PROFILES_DIR, name + ".json")
    if not os.path.isfile(path):
        raise SystemExit("make_hil: no profile {} (have: {})".format(
            path, ", ".join(sorted(p[:-5] for p in os.listdir(PROFILES_DIR)
                                   if p.endswith(".json")))))
    with open(path, encoding="utf-8") as f:
        prof = json.load(f)
    prof["name"] = name
    return prof


def apply_profile(prof):
    """Patch the build copy's config.h and regenerate its burn table."""
    cfg_path = os.path.join(SKETCH_DIR, "src", "config.h")
    with open(cfg_path, encoding="utf-8") as f:
        text = f.read()
    for name, val in prof.get("config", {}).items():
        pat = re.compile(r"(constexpr\s+[\w:]+\s+" + re.escape(name) + r"\s*=\s*)([^;]+)(;)")
        text, n = pat.subn(lambda m: m.group(1) + val + m.group(3), text)
        if n != 1:
            raise SystemExit("make_hil: profile constant {} matched {} times in config.h".format(name, n))
    with open(cfg_path, "w", encoding="utf-8", newline="") as f:
        f.write(text)
    bt = prof.get("burn_table")
    if bt:
        cmd = [PY, os.path.join(REPO_ROOT, "tools", "burn_table.py"),
               "--eng", os.path.join(REPO_ROOT, bt["eng"]), "--mass", str(bt["mass"]),
               "--eject-kg", str(bt.get("eject_kg", 0.0)),
               "--out", os.path.join(SKETCH_DIR, "src", "core", "burn_table.h")]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode:
            raise SystemExit("make_hil: burn_table.py failed:\n" + r.stderr)
        window = [l for l in r.stdout.splitlines() if "Target apogee" in l or "NO apogee" in l]
        print("burn table:", bt["eng"], "@", bt["mass"], "kg |", window[0].strip() if window else "")


def generate(profile=None):
    os.makedirs(SKETCH_DIR, exist_ok=True)
    keep = {SKETCH_NAME + ".ino", "HIL_BUILD_NOT_FOR_FLIGHT.txt", "hil_profile.json"}
    for f in os.listdir(SKETCH_DIR):
        p = os.path.join(SKETCH_DIR, f)
        if os.path.isfile(p) and f not in keep:
            os.remove(p)
    with open(os.path.join(REPO_ROOT, "RocketFC.ino"), encoding="utf-8") as f:
        ino = f.read().replace("\r\n", "\n")
    ino = _sub_once(ino, "    const uint32_t period = usFast;\n",
                    "    const uint32_t period = usFast;\n"
                    "    g_hilPeriodUs = period;  // HIL: LIVE telemetry reports the core's dt\n",
                    "the fast-tick period in RocketFC.ino")
    with open(os.path.join(SKETCH_DIR, SKETCH_NAME + ".ino"), "w", encoding="utf-8",
              newline="\n") as f:
        f.write(ino)
    mirror(os.path.join(REPO_ROOT, "src"), os.path.join(SKETCH_DIR, "src"),
           extra={os.path.join("hw", "sensors_real.h"), os.path.join("hw", "hil_hooks.h")})

    hw = os.path.join(SKETCH_DIR, "src", "hw")
    with open(os.path.join(hw, "sensors.h"), encoding="utf-8") as f:
        real = f.read()
    real, n = re.subn(r"\bclass Sensors\b", "class RealSensors", real)
    if n != 1:
        raise SystemExit("make_hil: expected exactly one `class Sensors` in "
                         "src/hw/sensors.h, found {} -- update make_hil.py / "
                         "sensors_hil.h to match".format(n))
    with open(os.path.join(hw, "sensors_real.h"), "w", encoding="utf-8", newline="") as f:
        f.write(real)

    with open(os.path.join(HIL_DIR, "firmware", "sensors_hil.h"), encoding="utf-8") as f:
        shim = f.read()
    desc = source_desc().replace("\\", "/").replace('"', "'")
    if profile:
        desc += " profile " + profile["name"]
    shim = shim.replace("#pragma once\n",
                        '#pragma once\n#define HIL_SOURCE_DESC "{}"\n'.format(desc), 1)
    with open(os.path.join(hw, "sensors.h"), "w", encoding="utf-8", newline="") as f:
        f.write(shim)
    shutil.copy2(os.path.join(HIL_DIR, "firmware", "hil_hooks.h"), os.path.join(hw, "hil_hooks.h"))
    patch_actuators(os.path.join(hw, "actuators.h"))
    if profile:
        apply_profile(profile)
    with open(os.path.join(SKETCH_DIR, "hil_profile.json"), "w", encoding="utf-8") as f:
        json.dump(profile or {"name": "none"}, f, indent=1)

    with open(os.path.join(SKETCH_DIR, "HIL_BUILD_NOT_FOR_FLIGHT.txt"), "w") as f:
        f.write("Generated by tools/hil/make_hil.py from {}.\n"
                "Sensors are replaced by a trace (tools/hil/make_trace.py) or by the\n"
                "live closed-loop sim (tools/hil/live/).\n"
                "Never fly this build.\n".format(desc))
    print("generated", SKETCH_DIR)
    print("source:   ", desc)


def arduino(*args):
    if not os.path.isfile(ARDUINO_CLI):
        raise SystemExit("arduino-cli not found at " + ARDUINO_CLI)
    cmd = [ARDUINO_CLI, *args]
    print(">", " ".join(cmd))
    subprocess.run(cmd, check=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--no-compile", action="store_true", help="generate the copy only")
    ap.add_argument("--upload", metavar="PORT",
                    help="upload after compiling, e.g. COM7 (the Teensy's port)")
    ap.add_argument("--profile", metavar="NAME",
                    help="motor/mass profile from tools/hil/profiles/ (e.g. e18w_1400g)")
    args = ap.parse_args()

    generate(load_profile(args.profile) if args.profile else None)
    if args.no_compile:
        return 0
    out_dir = os.path.join(BUILD_ROOT, "out")
    arduino("compile", "--fqbn", FQBN, "--board-options", BOARD_OPTIONS,
            "--build-path", out_dir, SKETCH_DIR)
    if args.upload:
        arduino("upload", "--fqbn", FQBN, "--board-options", BOARD_OPTIONS,
                "--input-dir", out_dir, "-p", args.upload, SKETCH_DIR)
        print("\nUploaded the HIL build. It enumerates as TWO serial ports:\n"
              "  the first is the normal RocketFC CLI, the second is HIL control\n"
              "  (tools/hil/hil_ctl.py). Reflash RocketFC before any flight.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
