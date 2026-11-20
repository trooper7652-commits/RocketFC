#!/usr/bin/env python3
"""PC test of the bench `chute wiggle` command.

Compiles the generated HIL sketch against tools/hil/test/mock with
test/host_wiggle.cpp, which sends `chute wiggle` while disarmed and again
while armed, and logs every chute-servo pulse (ms, us). Checks:

  disarmed  the pulses are the in-flight wiggle: CHUTE_WIG_HIGH_DEG, _LOW_DEG,
            HIGH, LOW, then the saved OPEN angle, at 0/190/340/490/640 ms
            (+-4 ms), through the actuator's deg->us map; and the console
            printed each stroke
  armed     the command is refused ("only allowed while DISARMED") and the
            chute servo doesn't move

    python tools/hil/test_wiggle.py
"""
import os
import re
import subprocess
import sys

import make_hil
import test_hil

HIL = test_hil.HIL
TOL_MS = 4


def cfg_const(name):
    with open(os.path.join(make_hil.REPO_ROOT, "src", "config.h"), encoding="utf-8") as f:
        m = re.search(r"constexpr\s+\w+\s+" + name + r"\s*=\s*([-\d.]+)f?\s*;", f.read())
    return float(m.group(1))


def deg_to_us(deg):  # Actuators::chuteDegToUs
    lo, hi = cfg_const("CHUTE_SERVO_US_AT_0_DEG"), cfg_const("CHUTE_SERVO_US_AT_180_DEG")
    return min(2500.0, max(500.0, lo + min(180.0, max(0.0, deg)) * (hi - lo) / 180.0))


def main():
    make_hil.generate()
    sketch = make_hil.SKETCH_DIR
    gxx = test_hil.find_gxx()
    exe = os.path.join(test_hil.HOST_DIR, "host_wiggle" + (".exe" if os.name == "nt" else ""))
    os.makedirs(test_hil.HOST_DIR, exist_ok=True)
    env = dict(os.environ, PATH=os.path.dirname(gxx) + os.pathsep + os.environ["PATH"])
    subprocess.run([gxx, "-std=gnu++17", "-O2", "-Wall", "-Wno-unused-function",
                    "-I", os.path.join(HIL, "test", "mock"), "-I", sketch,
                    os.path.join(HIL, "test", "host_wiggle.cpp"), "-o", exe],
                   check=True, env=env)
    sd = os.path.join(test_hil.HOST_DIR, "sd_wiggle")
    os.makedirs(sd, exist_ok=True)
    r = subprocess.run([exe, sd], env=env, capture_output=True, text=True, timeout=300)
    print(r.stdout[-3000:] if r.returncode else "", end="")
    probs = []
    if r.returncode:
        print("host exit", r.returncode)
        return 1

    lines = r.stdout.splitlines()
    rows = [tuple(int(x) for x in l.split()[1:]) for l in lines if l.startswith("SERVO ")]
    # markers: pin -1 start, -2 end, -3 armed-before, -4 armed-after
    def at(pin):
        return next(i for i, (_, p, _) in enumerate(rows) if p == pin)
    i0, i1, i3, i4 = at(-1), at(-2), at(-3), at(-4)
    wig = [(ms, us) for ms, p, us in rows[i0 + 1:i1] if p >= 0]
    armed = [(ms, us) for ms, p, us in rows[i3 + 1:i4] if p >= 0]
    t0 = rows[i0][0]

    # ---- printed timeline (console lines) ----
    print("printed by the CLI:")
    for l in lines:
        if re.search(r"\] (wiggle|done|\s+\+\d+ ms|'chute')", l):
            print("  " + l)

    low, high = cfg_const("CHUTE_WIG_LOW_DEG"), cfg_const("CHUTE_WIG_HIGH_DEG")
    open_us = deg_to_us(cfg_const("CHUTE_RELEASE_DEG_DEFAULT"))  # EEPROM is fresh: default
    n = int(cfg_const("CHUTE_WIG_CYCLES"))
    want_us = [deg_to_us(high)]
    for _ in range(n):
        want_us += [deg_to_us(low), deg_to_us(high) if _ < n - 1 else open_us]
    # servo changes: HIGH@0, LOW@FIRST, HIGH@+DOWN, LOW@+UP, ... last OPEN
    first, down, up = (cfg_const("CHUTE_WIG_FIRST_MS"), cfg_const("CHUTE_WIG_DOWN_MS"),
                       cfg_const("CHUTE_WIG_UP_MS"))
    want_t = [0.0]
    for k in range(n):
        want_t.append(want_t[-1] + (first if k == 0 else up))
        want_t.append(want_t[-1] + down)
    want_t = want_t[:len(want_us)]

    got = [(ms - t0, us) for ms, us in wig]
    print("chute servo writes (ms from start, us):", got)
    print("expected:", list(zip(want_t, [round(u) for u in want_us])))
    if len(got) != len(want_us):
        probs.append("{} servo writes, expected {}".format(len(got), len(want_us)))
    else:
        for (ms, us), wt, wu in zip(got, want_t, want_us):
            if abs(ms - wt) > TOL_MS:
                probs.append("write at {} ms, expected {} ms".format(ms, wt))
            if abs(us - wu) > 1.0:
                probs.append("pulse {} us, expected {:.0f} us".format(us, wu))
    if not any("only allowed while DISARMED" in l and "chute" in l for l in lines):
        probs.append("armed `chute wiggle` was not refused with the DISARMED message")
    if armed:
        probs.append("chute servo moved while armed: {}".format(armed))

    print("\n==== chute wiggle PC test ====")
    print("PASS" if not probs else "FAIL: " + "; ".join(probs))
    return 0 if not probs else 1


if __name__ == "__main__":
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    sys.exit(main())
