#!/usr/bin/env python3
"""Estimate the drag area CdA (m^2) from a RocketFC flight log.

Usage:
    python tools/fit_cda.py LOG.csv [LOG2.csv ...] [--mass KG] [--rho R | --alt-m M --temp-c C] [--plot]

Physics: in unpowered flight the accelerometer measures specific force, which
is aerodynamic drag only (gravity is not sensed). Drag acts along body Z (the
nose axis), so the SIGNED axial az is used, not the 3-axis norm: the norm
rectifies sensor noise into a positive bias (2.4-7x too high on this rocket's
low flight, where drag is only ~0.05 m/s^2 against 0.12 m/s^2/axis noise).
Sign: COAST is nose-first, drag decelerates, az < 0; DESCENT is tail-first,
az > 0 (plant.py/sensors.py convention; assumes the logged accel is already in
the body frame with +Z toward the nose, as logger.h says). Per phase, with
v = vertical Kalman-filter velocity kf_vel (wind and horizontal speed are
IGNORED; they would bias CdA high):
    a_drag = sign * az = c * v^2 + b        (least squares WITH intercept b,
                                             which absorbs the accel Z bias)
    CdA = 2 * m * c / rho, +/- 1 sigma from the slope's standard error.
The through-origin fit (biased by any accel bias) and the median per-sample CdA
are printed for reference. A JOINT line fits both phases with one CdA and one
shared bias (opposite signs separate bias from drag; assumes equal
nose-first/tail-first CdA, true in the sim): by far the best conditioned.
Speed estimate: kf_vel (it reads ~0.1-0.2 m/s toward +Z vs truth, i.e. ~5 %
low speed in descent -> CdA ~ +7 % high). A centred finite difference of
kf_alt was tried and was no better, so kf_vel is kept.

Log units (checked against tools/sim/logfile.py, src/hw/logger.h and
tools/sim/sensors.py): ax/ay/az are body specific force in m/s^2 (NOT g).

Phases (state names from src/core/flight_state.h):
    COAST    state COAST, kf_vel > 0 (still ascending), skipping the first
             COAST_SKIP_S = 0.25 s (motor tail-off still shows as thrust; the
             sim E18W tail-off runs ~0.22 s into COAST).
    DESCENT  state DESCENT, kf_vel < 0, before the first FIRE_LANDING (pyro
             bit 1) and before the first chute release (pyro bit 0). In
             CHUTE_TEST mode the chute is released at apogee, so there is no
             bare-airframe descent phase (reported as no samples).
Only samples with |kf_vel| >= 2 m/s are used.

Accuracy on this rocket (apogee ~15 m, <= 6 m/s before the landing burn): ONE
flight with default sensor noise gives COAST sigma ~ +/-300-560 % (useless), DESCENT
~ +/-40-80 %, JOINT ~ +/-25-45 %; the tool warns when sigma > 25 % of CdA. A higher
or faster flight, or POOLING several logs (pass several; the POOLED line
fits one CdA with a per-log accel bias), is needed for a good number.
Other warnings: < 50 samples, vmax/vmin < 1.3, CdA <= 0.

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


def fit_pooled(phase_sets, mass, rho):
    """Pool any number of logs (each a {"COAST": fit, "DESCENT": fit} dict):
    ONE CdA, one accel bias PER LOG, az = s*c*v^2 + b_log (b_log removed by
    centring each log's samples). With a single log this is the JOINT fit:
    opposite signs for coast/descent separate bias from drag. Assumes equal
    nose-first/tail-first CdA. Returns (cda, sigma), or None unless both a
    coast and a descent phase are present somewhere."""
    sxx = sxy = 0.0
    n = nlogs = 0
    groups = []
    signs = set()
    for phases in phase_sets:
        xs, ys = [], []
        for name, f in phases.items():
            if f["n"] >= 3:
                xs += [AXIAL_SIGN[name] * x for x in f["v2"]]
                ys += f["az"]
                signs.add(AXIAL_SIGN[name])
        if len(xs) < 3:
            continue
        mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
        gx = [x - mx for x in xs]
        gy = [y - my for y in ys]
        groups.append((gx, gy))
        sxx += sum(x * x for x in gx)
        sxy += sum(x * y for x, y in zip(gx, gy))
        n += len(xs)
        nlogs += 1
    if nlogs == 0 or sxx <= 0 or len(signs) < 2:
        return None
    c = sxy / sxx
    rss = sum((y - c * x) ** 2 for gx, gy in groups for x, y in zip(gx, gy))
    k = 2.0 * mass / rho
    return k * c, k * math.sqrt(rss / max(n - nlogs - 1, 1) / sxx)


def fit_joint(phases, mass, rho):
    """Both phases of ONE log, one CdA and one shared accel bias (see
    fit_pooled). Returns (cda, sigma) or None."""
    return fit_pooled([phases], mass, rho)


def fit_log(path, mass, rho):
    """{"COAST": fit, "DESCENT": fit} for a log file."""
    rows = logfile.read_log(path)["rows"]
    ph = split_phases(rows)
    return {name: fit_phase(rs, mass, rho, name) for name, rs in ph.items()}


def _warn_sigma(j):
    if j[0] <= 0 or j[1] > SIGMA_WARN * j[0]:
        print("        WARNING: +/-1 sigma wider than +/-%.0f %%: "
              "estimate not trustworthy" % (100 * SIGMA_WARN))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("log", nargs="+", help="one or more flight logs")
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
    allres = []
    for path in a.log:
        res = fit_log(path, mass, rho)
        allres.append(res)
        if len(a.log) > 1:
            print("--- %s" % path)
        for name, f in res.items():
            if f["n"] < 3:
                print("%-7s no samples (|kf_vel| >= %.1f m/s)" % (name, V_MIN))
                continue
            print("%-7s CdA %.5f +/- %.5f m^2 (intercept fit, 1 sigma)  origin fit %.5f  "
                  "median per-sample %.5f  n=%d  accel bias %+.3f m/s^2  v %.1f..%.1f m/s"
                  % (name, f["cda"], f["cda_sigma"], f["cda_origin"], f["median"], f["n"],
                     f["bias"], f["vmin"], f["vmax"]))
            for w in f["warnings"]:
                print("        WARNING: %s" % w)
        j = fit_joint(res, mass, rho)
        if j:
            print("JOINT   CdA %.5f +/- %.5f m^2 (both phases, shared accel bias; "
                  "assumes equal nose-first/tail-first CdA)" % j)
            _warn_sigma(j)
    if len(allres) > 1:
        p = fit_pooled(allres, mass, rho)
        if p:
            print("=== POOLED over %d logs: CdA %.5f +/- %.5f m^2 (one CdA, one accel "
                  "bias per log)" % ((len(allres),) + p))
            _warn_sigma(p)
    res = allres[0]
    if a.plot:
        import matplotlib.pyplot as plt
        for name, f in res.items():
            if f["n"] >= 3:
                plt.scatter(f["v2"], [f["k"] * d for d in f["ad"]], s=4, label=name)
                xs = [0, max(f["v2"])]
                plt.plot(xs, [f["cda"] * x + f["k"] * f["bias"] for x in xs])
        plt.xlabel("v^2 (m^2/s^2)")
        plt.ylabel("2 m a_drag / rho (m^2 * m^2/s^2)")
        plt.legend()
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
