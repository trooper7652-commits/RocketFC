#!/usr/bin/env python3
"""Estimate the drag area CdA (m^2) from a RocketFC flight log.

Usage:
    python tools/fit_cda.py LOG.csv [--mass KG] [--rho R | --alt-m M --temp-c C] [--plot]

Physics: in unpowered flight the accelerometer measures specific force, which
is aerodynamic drag only (gravity is not sensed), so per sample
    CdA = 2 * m * |a_spec| / (rho * v^2)
with |a_spec| the 3-axis accel norm in m/s^2 and v the vertical Kalman-filter
velocity kf_vel. Horizontal speed and wind are IGNORED (v is vertical-only): a
real flight with wind or a drifting rocket sees a larger airspeed than kf_vel
and this tool then over-estimates CdA.

Log units (checked against tools/sim/logfile.py, src/hw/logger.h and
tools/sim/sensors.py): ax/ay/az are body specific force in m/s^2 (NOT g).

Phases (state names from src/core/flight_state.h):
    COAST    state COAST, kf_vel > 0 (still ascending), skipping the first
             COAST_SKIP_S seconds (motor tail-off / burnout-detection lag still
             shows up as thrust, not drag; in the sim the E18W tail-off
             runs ~0.22 s into COAST, so 0.25 s; on this low flight that
             leaves ~4-5 m/s, ~50 samples at 500 Hz but only ~10 at 100 Hz).
    DESCENT  state DESCENT, kf_vel < 0, before the first FIRE_LANDING (pyro
             bit 1) and before the first chute release (pyro bit 0). In
             CHUTE_TEST mode the chute is released at apogee, so there is no
             bare-airframe descent phase to fit (it is reported as 0 samples).
Only samples with |kf_vel| >= 4 m/s are used.

Per phase: least-squares CdA through the origin of |a|*2m/rho against v^2,
the median per-sample CdA, sample count, residual-based standard error of the
LS slope, the IQR of per-sample CdA, and the speed range. Warnings: < 50
samples, vmax/vmin < 1.3, or a median |a| < 0.5 m/s^2 (drag signal comparable
to accel noise/bias; the 3-axis norm is then biased HIGH by the noise, since
E|a| > |E a| -- on this rocket's low flight, ~6 m/s at burnout, drag is only
~0.05 m/s^2 and the default sim IMU noise (0.12 m/s^2/axis) swamps it).

Plant drag model (tools/sim/plant.py, checked): drag is -0.5*RHO*cda*v*|v|
along the body Z axis with v the body-frame AIRSPEED (velocity minus wind),
the SAME cda_m2 for nose-first ascent and tail-first descent, RHO fixed at
1.225 (no altitude dependence). Besides it there is only an aero normal force
from crossflow (small when flying straight, adds to the accel norm), the
canopy cda_chute_m2 after chute deploy, and legs/ground drag on the ground.
So with no wind and no tilt a fit recovers cda_m2 for both phases.
"""
import argparse
import math
import os
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "sim"))

import burn_table  # noqa: E402
import logfile  # noqa: E402

V_MIN = 2.0
SIGMA_WARN = 0.25
# Sign of the body-axial specific force az (+Z = nose) for each phase: drag is
# the only sensed force. Coast is nose-first, drag decelerates -> az < 0.
# Descent is tail-first, drag points up the nose axis -> az > 0.
AXIAL_SIGN = {"COAST": -1.0, "DESCENT": 1.0}
COAST_SKIP_S = 0.25
MIN_SAMPLES = 50
MIN_SPEED_RATIO = 1.3
MIN_SIGNAL_MPS2 = 0.5
PLANT_FINDING = ("plant.py drag model: one cda_m2 for ascent and descent, "
                 "along body Z on airspeed (wind incl.), RHO fixed 1.225; no "
                 "leg/crossflow body drag in flight except the small normal force")


def isa_rho(alt_m, temp_c):
    """Air density from the ISA pressure at alt_m (troposphere) and the given
    temperature (ideal gas, R = 287.058 J/kg/K)."""
    p = 101325.0 * (1.0 - 2.25577e-5 * alt_m) ** 5.25588
    return p / (287.058 * (temp_c + 273.15))


def split_phases(rows):
    """{"COAST": [rows], "DESCENT": [rows]}, before the speed filter."""
    coast, descent = [], []
    coast_t0 = None
    for r in rows:
        if r["pyro"] & 3:           # chute release or fire_landing: descent over
            if r["state"] not in ("COAST",):
                break
        if r["state"] == "COAST":
            if coast_t0 is None:
                coast_t0 = r["t"]
            if r["kf_vel"] > 0 and r["t"] - coast_t0 >= COAST_SKIP_S:
                coast.append(r)
        elif r["state"] == "DESCENT" and r["kf_vel"] < 0:
            descent.append(r)
    return {"COAST": coast, "DESCENT": descent}


def _ols(xs, ys):
    """Slope, intercept and slope standard error of y = c*x + b."""
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    c = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    b = my - c * mx
    s2 = sum((y - c * x - b) ** 2 for x, y in zip(xs, ys)) / max(n - 2, 1)
    return c, b, math.sqrt(s2 / sxx)


def fit_phase(rows, mass, rho, phase):
    """Fit one phase. Returns a dict (n == 0 when nothing qualifies)."""
    use = [r for r in rows if abs(r["kf_vel"]) >= V_MIN]
    out = {"n": len(use)}
    if len(use) < 3:
        out["n"] = len(use)
        return out
    k = 2.0 * mass / rho
    sgn = AXIAL_SIGN[phase]
    v2 = [r["kf_vel"] ** 2 for r in use]
    ad = [sgn * r["az"] for r in use]            # drag deceleration, m/s^2
    c, b, sc = _ols(v2, ad)
    c0 = sum(x * y for x, y in zip(v2, ad)) / sum(x * x for x in v2)
    n = len(use)
    speeds = [abs(r["kf_vel"]) for r in use]
    out.update(cda=k * c, cda_sigma=k * sc, bias=b, cda_origin=k * c0,
               median=statistics.median(k * y / x for x, y in zip(v2, ad)),
               vmin=min(speeds), vmax=max(speeds), v2=v2, ad=ad, az=[r["az"] for r in use], k=k, warnings=[])
    if n < MIN_SAMPLES:
        out["warnings"].append("only %d samples (< %d)" % (n, MIN_SAMPLES))
    if out["vmax"] / out["vmin"] < MIN_SPEED_RATIO:
        out["warnings"].append("narrow speed range (vmax/vmin = %.2f < %.1f)"
                               % (out["vmax"] / out["vmin"], MIN_SPEED_RATIO))
    if out["cda"] <= 0:
        out["warnings"].append("fitted CdA <= 0: drag below the noise, or the "
                               "accel Z sign convention differs from the sim's")
    elif out["cda_sigma"] > SIGMA_WARN * out["cda"]:
        out["warnings"].append("+/-1 sigma (%.0f %%) wider than +/-%.0f %%: "
                               "estimate not trustworthy"
                               % (100 * out["cda_sigma"] / out["cda"], 100 * SIGMA_WARN))
    return out


def fit_joint(phases, mass, rho):
    """Both phases at once with ONE CdA and ONE shared accel Z bias:
    az = s*c*v^2 + b with s = -1 (coast) / +1 (descent). The opposite signs
    separate the bias from the drag, so it is better conditioned than either
    phase alone -- but it assumes nose-first and tail-first CdA are equal.
    Returns (cda, sigma) or None."""
    xs, ys = [], []
    for name, f in phases.items():
        if f["n"] >= 3:
            xs += [AXIAL_SIGN[name] * x for x in f["v2"]]
            ys += f["az"]
    if len(xs) < 3 or not any(x > 0 for x in xs) or not any(x < 0 for x in xs):
        return None
    k = 2.0 * mass / rho
    c, _, sc = _ols(xs, ys)
    return k * c, k * sc


def fit_log(path, mass, rho):
    """{"COAST": fit, "DESCENT": fit} for a log file."""
    rows = logfile.read_log(path)["rows"]
    ph = split_phases(rows)
    return {name: fit_phase(rs, mass, rho, name) for name, rs in ph.items()}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("log")
    ap.add_argument("--mass", type=float, default=None,
                    help="kg (default MASS_DESCENT_KG from src/config.h)")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--rho", type=float, default=None, help="air density kg/m^3 (default 1.225)")
    g.add_argument("--alt-m", type=float, default=None, help="altitude for ISA density")
    ap.add_argument("--temp-c", type=float, default=15.0, help="with --alt-m (default 15)")
    ap.add_argument("--plot", action="store_true")
    a = ap.parse_args(argv)

    mass = a.mass if a.mass is not None else burn_table.config_value("MASS_DESCENT_KG")
    rho = a.rho if a.rho is not None else (
        isa_rho(a.alt_m, a.temp_c) if a.alt_m is not None else 1.225)
    print("mass %.4f kg, rho %.4f kg/m^3, |kf_vel| >= %.1f m/s, accel in m/s^2" %
          (mass, rho, V_MIN))
    print(PLANT_FINDING)
    res = fit_log(a.log, mass, rho)
    for name, f in res.items():
        if f["n"] == 0:
            print("%-7s no samples (|kf_vel| >= %.1f m/s)" % (name, V_MIN))
            continue
        print("%-7s CdA fit %.5f m^2  median %.5f  n=%d  stderr %.5f  IQR %.5f  "
              "v %.1f..%.1f m/s" % (name, f["cda"], f["median"], f["n"], f["stderr"],
                                   f["iqr"], f["vmin"], f["vmax"]))
        for w in f["warnings"]:
            print("        WARNING: %s" % w)
    if a.plot:
        import matplotlib.pyplot as plt
        for name, f in res.items():
            if f["n"]:
                plt.scatter(f["v2"], f["y"], s=4, label=name)
                xs = [0, max(f["v2"])]
                plt.plot(xs, [f["cda"] * x for x in xs])
        plt.xlabel("v^2 (m^2/s^2)")
        plt.ylabel("2 m |a| / rho (m^2 * m^2/s^2 / ...)")
        plt.legend()
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
