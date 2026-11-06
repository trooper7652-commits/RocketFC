#!/usr/bin/env python3
"""
Vendor the shared flight headers from RocketFC into this project's src/.

This bench rig exists to validate the code that actually flies, so it must use
byte-identical math: the same quaternion library, the same AHRS, the same TVC
PID, the same sensor axis mapping, and the same config constants (IMU_R_SB,
servo signs, gains). Rather than reaching across into ../RocketFC at compile
time -- which the Arduino IDE does not reliably support, since it copies the
sketch folder to a temp build directory -- we keep local copies and re-sync them
with this script.

The copies carry a provenance banner naming the RocketFC commit they came from.
Never edit them here: edit in RocketFC, then re-run this script.

Pure standard library, matching the style of RocketFC's own tools/*.py.

Usage:
    python sync_core.py                 # copy, overwriting local vendored files
    python sync_core.py --check         # exit 1 if any copy is stale/missing
    python sync_core.py --rocketfc PATH # non-default RocketFC location
"""

import argparse
import datetime
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_ROCKETFC = os.path.normpath(os.path.join(HERE, "..", "RocketFC"))

# Files copied verbatim from RocketFC, as paths relative to each project root.
#
# config.h is vendored on purpose rather than rewritten: the bench must use the
# same IMU_R_SB, servo signs/trims and PID gains as flight, because validating
# exactly those is the point of the exercise. Bench-only knobs live separately
# in src/bench_config.h so this stays a clean copy.
#
# actuators.h is vendored unstripped so Actuators::toUs() -- the gimbal-radians
# to servo-microseconds mapping -- is byte-identical to flight. Its begin()
# drives both pyro gates LOW as outputs (pin 6 landing e-match, pin 7
# leg-release nichrome), which is the safe state, and parks the parachute latch
# servo on pin 23 at LOCK. The bench firmware never calls enablePyro(),
# firePyro(), testFire(), applyLegs(), testLegs() or applyChute(), and the
# bench CLI has no pyrotest or chute command, so no code path here can
# energize a pyro channel or open the chute latch.
VENDORED = [
    "src/config.h",
    "src/core/quat.h",
    "src/core/ahrs.h",
    "src/core/control.h",
    "src/hw/sensors.h",
    "src/hw/actuators.h",
]

BANNER_MARK = "// AUTO-COPIED from RocketFC"


def rocketfc_rev(root):
    """Short git SHA of the RocketFC checkout, or 'unknown' if unavailable."""
    try:
        out = subprocess.run(
            ["git", "-C", root, "rev-parse", "--short", "HEAD"],
            capture_output=True, text=True, timeout=10,
        )
        if out.returncode == 0:
            rev = out.stdout.strip()
            dirty = subprocess.run(
                ["git", "-C", root, "status", "--porcelain", "--"] + VENDORED,
                capture_output=True, text=True, timeout=10,
            )
            if dirty.returncode == 0 and dirty.stdout.strip():
                rev += "+dirty"
            return rev
    except (OSError, subprocess.SubprocessError):
        pass
    return "unknown"


def banner(rel, rev, date):
    return (
        "{mark} ({rel}) @ {rev} on {date}.\n"
        "// DO NOT EDIT HERE. Edit the original in RocketFC and re-run\n"
        "// sync_core.py, or your fix will be silently overwritten and the\n"
        "// bench will stop testing what actually flies.\n"
    ).format(mark=BANNER_MARK, rel=rel, rev=rev, date=date)


def strip_banner(text):
    """Remove a previously prepended banner so comparisons see real content."""
    if not text.startswith(BANNER_MARK):
        return text
    lines = text.split("\n")
    i = 0
    while i < len(lines) and lines[i].startswith("//"):
        i += 1
    return "\n".join(lines[i:])


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rocketfc", default=DEFAULT_ROCKETFC,
                    help="path to the RocketFC project (default: %(default)s)")
    ap.add_argument("--check", action="store_true",
                    help="do not write; exit 1 if any vendored copy is stale")
    args = ap.parse_args()

    src_root = os.path.abspath(args.rocketfc)
    if not os.path.isdir(src_root):
        sys.exit("RocketFC not found at {}\n"
                 "Pass --rocketfc PATH to point at it.".format(src_root))

    rev = rocketfc_rev(src_root)
    date = datetime.date.today().isoformat()

    stale, copied = [], []
    for rel in VENDORED:
        src = os.path.join(src_root, rel.replace("/", os.sep))
        dst = os.path.join(HERE, rel.replace("/", os.sep))

        if not os.path.isfile(src):
            sys.exit("missing in RocketFC: {}".format(src))

        with open(src, "r", encoding="utf-8", newline="") as f:
            body = f.read()

        current = None
        if os.path.isfile(dst):
            with open(dst, "r", encoding="utf-8", newline="") as f:
                current = strip_banner(f.read())

        if current == body:
            continue

        if args.check:
            stale.append(rel)
            continue

        os.makedirs(os.path.dirname(dst), exist_ok=True)
        with open(dst, "w", encoding="utf-8", newline="") as f:
            f.write(banner(rel, rev, date))
            f.write(body)
        copied.append(rel)

    if args.check:
        if stale:
            print("STALE vendored copies ({}):".format(len(stale)))
            for rel in stale:
                print("  " + rel)
            print("\nRun:  python sync_core.py")
            return 1
        print("all {} vendored files match RocketFC @ {}".format(len(VENDORED), rev))
        return 0

    if copied:
        print("synced {} file(s) from RocketFC @ {}:".format(len(copied), rev))
        for rel in copied:
            print("  " + rel)
    else:
        print("already up to date with RocketFC @ {}".format(rev))
    return 0


if __name__ == "__main__":
    sys.exit(main())
