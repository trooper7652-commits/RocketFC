#!/usr/bin/env python3
"""Validate tools/fit_cda.py against the closed-loop sim: write sim logs with
plant cda_m2 in {0.003, 0.004, 0.006}, modes land and chute, 5 seeds each, and
check the fitted CdA recovers the plant value within +/-15 % (median over seeds).

Two sensor settings are run:
  * noiseless IMU (accel noise and bias zeroed): validates the physics and the
    phase logic; ASSERTED.
  * default IMU (0.12 m/s^2/axis noise, ~0.05 m/s^2 bias): informational only.
    This rocket's flight is low (about 6 m/s at burnout, <= 6 m/s before the
    landing burn), so drag is ~0.05 m/s^2 -- below the accel noise -- and the
    3-axis norm is dominated by noise. NOT asserted; printed so the limit is
    visible.

Excluded, with explicit notes: chute mode DESCENT (the chute is released at
apogee, so there is no bare-airframe descent). The wind is 0 (the default).

Run:  python tools/test_fit_cda.py
"""
import dataclasses
import os
import statistics
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "sim"))

import fit_cda  # noqa: E402
import logfile  # noqa: E402
import run as run_mod  # noqa: E402
import vehicle  # noqa: E402

CDAS = (0.003, 0.004, 0.006)
MODES = ("land", "chute")
SEEDS = (1, 2, 3, 4, 5)
TOL = 0.15
failures = 0


def check(cond, msg):
    global failures
    print("  %s: %s" % ("PASS" if cond else "FAIL", msg))
    if not cond:
        failures += 1


def run_case(cda, mode, seed, noiseless, tmp):
    vc = vehicle.VehicleConfig(cda_m2=cda, wind_mps=0.0)
    if noiseless:
        vc = dataclasses.replace(vc, accel_noise_quiet=0.0, accel_noise_thrust=0.0,
                                 accel_bias_mps2=(0.0, 0.0, 0.0))
    res = run_mod.simulate(vc, mode=mode, seed=seed, record=True)
    path = os.path.join(tmp, "%s_%g_%d.csv" % (mode, cda, seed))
    logfile.write_log(path, res)
    return fit_fit(path, vc)


def fit_fit(path, vc):
    return fit_cda.fit_log(path, vc.mass_descent_kg, 1.225)


def main():
    with tempfile.TemporaryDirectory() as tmp:
        for noiseless in (True, False):
            print("== %s IMU ==" % ("noiseless" if noiseless else "default-noise (informational)"))
            for mode in MODES:
                for cda in CDAS:
                    fits = [run_case(cda, mode, s, noiseless, tmp) for s in SEEDS]
                    for phase in ("COAST", "DESCENT"):
                        vals = [f[phase]["cda"] for f in fits if f[phase]["n"] > 0]
                        ns = [f[phase]["n"] for f in fits]
                        tag = "cda=%.3f %-5s %-7s" % (cda, mode, phase)
                        if not vals:
                            print("  NOTE: %s excluded: no samples with |kf_vel|>=%.0f m/s "
                                  "(chute mode releases the chute at apogee)" % (tag, fit_cda.V_MIN))
                            continue
                        med = statistics.median(vals)
                        err = med / cda - 1.0
                        line = "%s fit median %.5f (%+.1f %%) n/seed=%s spread %.5f" % (
                            tag, med, 100 * err, ns, max(vals) - min(vals))
                        if noiseless:
                            check(abs(err) <= TOL, line)
                        else:
                            print("  info: " + line)
    print("FAILED: %d" % failures if failures else "ALL PASSED")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
