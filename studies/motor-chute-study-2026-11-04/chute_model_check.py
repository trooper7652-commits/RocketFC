"""Part B, step 1 check: the sized-canopy model in the study plant.

1. Instant fill (fill_n ~ 0, no line stretch): the steady descent rate must
   match sqrt(2 m g / (rho * (CdA_canopy + CdA_body))) within 2 %.
2. Normal fill (n = 8, 0.3 s line stretch): from a 12 m/s fall, the fill
   distance must equal n * D and the fill time must grow with D.
Drives tools/sim/plant.py directly (no flight software), from free fall.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mclib as L  # noqa: E402

sys.path.insert(0, os.path.join(L.WS, "base", "tools", "sim"))
import plant as plant_mod  # noqa: E402
import vehicle  # noqa: E402
from plant import V3  # noqa: E402


def drop(D, cd, n, ls, v0=12.0, h0=200.0, wind=0.0, dt=0.002):
    vc = vehicle.VehicleConfig(chute_diameter_m=D, chute_cd=cd, chute_fill_n=n,
                               chute_line_stretch_s=ls, ground_model="arrest",
                               wind_mps=wind, normal_force_translates=False)
    pl = plant_mod.Plant(vc)
    s = pl.state
    s.ascent_ignite_t = -100.0            # motors long gone (no thrust, pad window over)
    s.apogee_t = -50.0
    s.pos = V3(0.0, 0.0, h0)
    s.vel = V3(0.0, 0.0, -v0)
    m = pl.mass_and_thrust(0.0)[0]
    pl.deploy_chute(0.0)
    t_full = d_full = None
    d = 0.0
    while s.t < 60.0 and s.pos.z > 1.0:
        pl.step(dt, 0.0, 0.0)
        if s.t >= ls:
            d += abs(s.vel.z) * dt
        if t_full is None and pl.chute_frac >= 0.999:
            t_full, d_full = s.t, d
    return m, -s.vel.z, t_full, d_full, vc


if __name__ == "__main__":
    print("1. instant fill: steady descent vs formula")
    for D, cd in ((1.0, 1.5), (1.4, 1.5), (1.8, 0.8)):
        m, v, _, _, vc = drop(D, cd, 1e-9, 0.0)
        cda = cd * math.pi * D * D / 4 + vc.cda_m2
        vf = math.sqrt(2 * m * 9.80665 / (1.225 * cda))
        print("   D %.1f m Cd %.1f, %.3f kg: sim %.3f m/s, formula %.3f m/s, error %+.2f %%" % (
            D, cd, m, v, vf, 100 * (v / vf - 1)))
    print("2. n = 8, 0.3 s line stretch, from a 12 m/s fall: fill time / distance")
    for D in (0.8, 1.2, 1.6, 2.0):
        m, v, tf, df, _ = drop(D, 1.5, 8.0, 0.3)
        print("   D %.1f m: full %.2f s after release, %.2f m of fall after line stretch (n*D = %.1f m)" % (
            D, tf, df, 8 * D))
