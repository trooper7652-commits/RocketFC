"""Clustered chutes vs one big chute (E18W/E18W, study copy).

A cluster of N canopies of diameter d (cluster drag efficiency 0.9) is the
plant's sized canopy with D = d and Cd = N * 0.9 * Cd_single: each canopy fills
over n * d, all at once. n = 8 = ideal (all open together, same fill constant
as the single); n = 12 = sluggish / out-of-step cluster inflation. Every
configuration has the same full drag area as one 60 in (1.524 m) Cd 2.2
annular, so the descent rates match once open.

Scenarios as chute_study.py: S1 apogee release, S3 gust aborts at four phases
(only aborted flights counted).
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mclib as L  # noqa: E402
from chute_study import gust_times  # noqa: E402

C = "c_AeroTech_E18W__AeroTech_E18W"
CD1, D1, ETA = 2.2, 1.524, 0.9
CONFIGS = [("1 x 60 in", 1, 8), ("2 x %.0f in" % (D1 / math.sqrt(2 * ETA) / 0.0254), 2, 8),
           ("2 x (out of step)", 2, 12), ("3 x %.0f in" % (D1 / math.sqrt(3 * ETA) / 0.0254), 3, 8),
           ("3 x (out of step)", 3, 12)]


def plant_for(N, n):
    if N == 1:
        return {"chute_diameter_m": D1, "chute_cd": CD1, "chute_fill_n": n}
    d = D1 / math.sqrt(N * ETA)
    return {"chute_diameter_m": round(d, 4), "chute_cd": round(N * ETA * CD1, 4), "chute_fill_n": n}


if __name__ == "__main__":
    print("| Chutes | Each | Apogee release: impact med / p90 | Coast abort (~13 m): <= 3 m/s, impact med | "
          "Apogee abort (~11 m): <= 3 m/s, impact med | All gust aborts: <= 3 m/s |")
    print("|---|---|---|---|---|---|")
    for lab, N, n in CONFIGS:
        pl = plant_for(N, n)
        tag = "cl%d_n%g" % (N, n)
        s1 = L.chute_metrics(L.run("cl_S1_%s_%s" % (tag, C), ws=C, mode="chute", plant=pl))
        ph = {}
        allab = []
        for p, tg in gust_times(C).items():
            rows = L.run("cl_S3%s_%s_%s" % (p, tag, C), ws=C, n=50,
                         plant=dict(pl, gust_mps=12.0, gust_time_s=round(tg, 2), gust_dur_s=0.5))
            ab = [r for r in rows if r["summary"].get("abort_reason")]
            ph[p] = L.chute_metrics(ab)
            allab += ab
        a = L.chute_metrics(allab)
        print("| %s | %.2f m | %.2f / %.2f m/s | %.0f %%, %.2f m/s | %.0f %%, %.2f m/s | %.0f %% |" % (
            lab, pl["chute_diameter_m"], s1["vz_med"], s1["vz_p90"],
            100 * ph["coast"]["le3"], ph["coast"]["vz_med"], 100 * ph["apogee"]["le3"], ph["apogee"]["vz_med"],
            100 * a["le3"]), flush=True)
