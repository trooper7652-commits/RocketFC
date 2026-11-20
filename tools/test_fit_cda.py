#!/usr/bin/env python3
"""Validate tools/fit_cda.py against the closed-loop sim: write sim logs with
plant cda_m2 in {0.003, 0.004, 0.006}, modes land and chute, and compare the
fitted CdA with the plant value (wind 0, the default).

  * DEFAULT sensor noise and bias (the real case, 15 seeds): the JOINT fit
    (coast + descent, shared accel bias) is ASSERTED within +/-15 % on the
    median over seeds. The per-phase fits are printed with achieved errors and
    the median sigma; they are NOT asserted here because one flight of this low
    rocket (<= 6 m/s, drag ~0.05 m/s^2 vs 0.12 m/s^2/axis accel noise) cannot
    pin a single phase down (COAST sigma ~ +/-300-560 %, DESCENT ~ +/-40-80 %).
    The tool warns whenever its own sigma exceeds 25 %.
    Also asserted: the fit POOLED over the 15 land flights at plant 0.004 is
    within +/-15 % and its sigma is below the single-flight median sigma.
  * NOISELESS IMU (noise and bias zeroed, 5 seeds): every phase and the joint
    fit are ASSERTED within +/-15 %, validating the physics and phase logic.
  * Chute mode: the COAST phase is identical to land mode (same flight up to
    apogee). The DESCENT phase is excluded (chute released at apogee: no
    bare-airframe samples) and so is the JOINT fit; explicit NOTEs print.

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
    path = os.path.join(tmp, "case.csv")
    logfile.write_log(path, res)
    f = fit_cda.fit_log(path, vc.mass_descent_kg, 1.225)
    out = {p: (f[p]["cda"], f[p]["cda_sigma"], f[p]["n"]) if f[p]["n"] >= 3 else None
           for p in f}
    j = fit_cda.fit_joint(f, vc.mass_descent_kg, 1.225)
    out["JOINT"] = (j[0], j[1], 0) if j else None
    out["_phases"] = f
    out["_mass"] = vc.mass_descent_kg
    return out


def summarize(fits, key):
    vals = [f[key] for f in fits if f[key] is not None]
    if not vals:
        return None
    return (statistics.median(v[0] for v in vals), statistics.median(v[1] for v in vals),
            len(vals))


def main():
    with tempfile.TemporaryDirectory() as tmp:
        for noiseless, seeds in ((False, range(1, 16)), (True, range(1, 6))):
            print("== %s IMU, %d seeds ==" % (
                "noiseless" if noiseless else "DEFAULT-noise", len(seeds)))
            for mode in MODES:
                for cda in CDAS:
                    fits = [run_case(cda, mode, s, noiseless, tmp) for s in seeds]
                    if mode == "land" and cda == 0.004 and not noiseless:
                        pooled = fit_cda.fit_pooled([f["_phases"] for f in fits],
                                                    fits[0]["_mass"], 1.225)
                        single_sig = summarize(fits, "JOINT")[1]
                        err = pooled[0] / cda - 1.0
                        check(abs(err) <= TOL and pooled[1] < single_sig,
                              "POOLED %d flights cda=0.004: %.5f (%+.1f %%) +/- %.5f "
                              "(+/-%.0f %%) vs single-flight median sigma %.5f" % (
                                  len(fits), pooled[0], 100 * err, pooled[1],
                                  100 * pooled[1] / cda, single_sig))
                    for key in ("COAST", "DESCENT", "JOINT"):
                        tag = "cda=%.3f %-5s %-7s" % (cda, mode, key)
                        sm = summarize(fits, key)
                        if sm is None:
                            print("  NOTE: %s excluded: no usable samples (chute mode "
                                  "releases the chute at apogee: no bare descent)" % tag)
                            continue
                        med, sig, nv = sm
                        err = med / cda - 1.0
                        line = ("%s median %.5f (%+.1f %%) median 1-sigma %.5f "
                                "(+/-%.0f %%) over %d seeds" % (tag, med, 100 * err, sig,
                                                              100 * sig / cda, nv))
                        if noiseless or key == "JOINT":
                            check(abs(err) <= TOL, line)
                        else:
                            print("  NOTE: %s -- not asserted (single-phase sigma too wide "
                                  "under default noise)" % line)
    print("FAILED: %d" % failures if failures else "ALL PASSED")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
