"""Value of information (1-D physics, analysis only): the best expected touchdown
speed any TIMING-ONLY solver could reach, for different levels of uncertainty
about the things it cannot observe before ignition (delay, mass, thrust, CdA).

For each scenario and descent speed: draw J vehicles, pick the single fire
altitude that minimizes the mean touchdown speed (hops included), report the
resulting mean / median / p90 and the chosen margin over the nominal table.
Perfect state knowledge is assumed (no estimator error) -> a lower bound.
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
L = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import risk_table as R  # noqa: E402

SCEN = [
    # name, delay half-width (frac, uniform), mass half-width, CdA half-width, thrust SD (frac, normal)
    ("sim MC as is (delay+-15%, mass+-5%, CdA+-20%)", 0.15, 0.05, 0.20, 0.0),
    ("+ thrust spread SD 1.7% (NAR cert, unverified)", 0.15, 0.05, 0.20, 0.017),
    ("+ thrust spread SD 6.7% (cert limit)", 0.15, 0.05, 0.20, 0.067),
    ("delay measured to +-5%", 0.05, 0.05, 0.20, 0.0),
    ("mass weighed to +-1%", 0.15, 0.01, 0.20, 0.0),
    ("delay +-5% and mass +-1%", 0.05, 0.01, 0.20, 0.0),
    ("delay +-2% (+-10 ms), mass +-1%, CdA +-10%", 0.02, 0.01, 0.10, 0.0),
    ("delay +-2%, mass +-1%, CdA +-10%, thrust SD 1.7%", 0.02, 0.01, 0.10, 0.017),
    ("everything known exactly", 0.0, 0.0, 0.0, 0.0),
]


def main():
    curve = R.Curve(os.path.join(L, "ws_base", "RocketFC", "tools", "motors", "Estes_F15.eng"))
    J = 1500
    speeds = [10.0, 15.0, 18.0]
    out = {}
    lines = []
    for name, dhw, mhw, chw, tsd in SCEN:
        rng = np.random.default_rng(12345)
        d = {"delay": 0.5 * (1 + rng.uniform(-dhw, dhw, J)),
             "mass": 0.95 * (1 + rng.uniform(-mhw, mhw, J)),
             "cda": 0.004 * (1 + rng.uniform(-chw, chw, J)),
             "tscale": 1 + tsd * rng.standard_normal(J),
             "eh": np.zeros(J), "ev_down": np.zeros(J)}
        row = {}
        for v in speeds:
            hn = R.nominal_altitude(curve, v, 0.95, 0.004, 0.5)
            levels = np.arange(max(1.0, hn - 8.0), hn + 8.0, 0.01)
            H, st = R.best_altitude(curve, v, d, levels)
            row[v] = {"margin_m": hn - H, "mean": st["mean"], "median": st["median"], "p90": st["p90"]}
        out[name] = row
        lines.append(f"{name:52s} " + "  ".join(
            f"v={v:4.1f}: late {row[v]['margin_m']:+.2f} m -> mean {row[v]['mean']:4.2f} "
            f"med {row[v]['median']:4.2f} p90 {row[v]['p90']:4.2f}" for v in speeds))
        print(lines[-1], flush=True)
    with open(os.path.join(L, "data", "value_of_info.json"), "w", encoding="utf-8") as f:
        json.dump(out, f, indent=1)
    with open(os.path.join(L, "data", "value_of_info.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
