"""Part B: chute size on the chosen motor pair (sized-canopy model, study copy).

Scenarios (dev seeds 101-103, measured spreads, wind -4..+4 m/s):
  S1  chute-test mode: the chute is released at apogee;
  S2  landing mode, dud landing igniter: the chute opens after DUD_WINDOW_MS
      (today 700 ms; S2b = 300 ms, the earlier not-ported fix);
  S3  landing mode, a 12 m/s, 0.5 s gust at four flight phases (mid-boost,
      mid-coast, apogee, early descent) to force tilt aborts at different
      heights; only flights that aborted count.
Canopies: CdA = Cd * pi * D^2 / 4; fill over n * D of fall after 0.3 s line stretch.

usage: python chute_study.py COMBO [n=8] [types=dome,flat,annular] [scen=S1,S2,S2b,S3]
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mclib as L  # noqa: E402

TYPES = {
    "dome": (1.5, [0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8]),
    "flat": (0.8, [1.2, 1.4, 1.6, 1.8, 2.0, 2.2, 2.4]),
    "annular": (2.2, [0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5]),
}
LAUNCH_T = 3.1   # run.simulate: arm_t 2.6 + countdown 0.5


def gust_times(combo):
    """Mid-boost, mid-coast, apogee, apogee + 1 s, from the combo's dev run."""
    rows = L.run("dev_" + combo, ws=combo)
    ap_t = L.q([r["summary"]["apogee_t"] for r in rows], 0.5)
    burn = float(open(os.path.join(L.WS, combo, "combo.json")).read().split('"ascent_burn_s":')[1].split(",")[0])
    t_bo = LAUNCH_T + burn
    return {"boost": LAUNCH_T + 0.5 * burn, "coast": 0.5 * (t_bo + ap_t), "apogee": ap_t, "descent": ap_t + 1.0}


def runs_for(combo, typ, D, n, scen):
    cd = TYPES[typ][0]
    plant = {"chute_diameter_m": D, "chute_cd": cd, "chute_fill_n": n}
    tag = "%s_%s_D%.2f_n%g" % (combo, typ, D, n)
    if scen == "S1":
        return L.run("ch_S1_" + tag, ws=combo, mode="chute", plant=plant)
    if scen == "S2":
        return L.run("ch_S2_" + tag, ws=combo, plant=dict(plant, dud_landing=True))
    if scen == "S2b":
        return L.run("ch_S2b_" + tag, ws=combo, plant=dict(plant, dud_landing=True),
                     fw={"DUD_WINDOW_MS": "300.0f"})
    if scen == "S3":
        rows = []
        for ph, tg in gust_times(combo).items():
            rows += L.run("ch_S3%s_%s" % (ph, tag), ws=combo, n=50,
                          plant=dict(plant, gust_mps=12.0, gust_time_s=round(tg, 2), gust_dur_s=0.5))
        return [r for r in rows if r["summary"].get("abort_reason")]
    raise ValueError(scen)


def report(combo, typ, n, scen, res):
    cd, sizes = TYPES[typ]
    print("\n== %s | %s canopy (Cd %.1f), fill n = %g | %s" % (scen, typ, cd, n, combo))
    print("| D | CdA | Vertical impact med / p90 / p99 | <= 3 m/s | Opened before impact | Sideways med / p90 | "
          "Deploy height med / p10 | Fill at impact (med, full) | Peak load med / p99 | Unconfirmed | Flights |")
    print("|---|---|---|---|---|---|---|---|---|---|---|")
    for D in sizes:
        m = L.chute_metrics(res[D])
        cda = cd * 3.14159265 * D * D / 4
        print("| %.1f m (%.0f in) | %.2f m² | %.2f / %.2f / %.2f m/s | %.0f %% | %.0f %% | %.1f / %.1f m/s | %.1f / %.1f m | %.0f %%, %.0f %% | %.1f / %.1f g | %.0f %% | %d of %d |" % (
            D, D / 0.0254, cda, m["vz_med"], m["vz_p90"], m["vz_p99"], 100 * m["le3"], 100 * m["opened"],
            m["vh_med"], m["vh_p90"], m["dh_med"], m["dh_p10"], 100 * m["frac_med"], 100 * m["full"],
            m["peak_g_med"], m["peak_g_p99"], 100 * m["unconfirmed"], m["n"], m["of"]), flush=True)


if __name__ == "__main__":
    combo = sys.argv[1]
    opts = dict(a.split("=", 1) for a in sys.argv[2:])
    n = float(opts.get("n", 8))
    types = opts.get("types", "dome,flat,annular").split(",")
    scens = opts.get("scen", "S1,S2,S2b,S3").split(",")
    for scen in scens:
        for typ in types:
            res = {D: runs_for(combo, typ, D, n, scen) for D in TYPES[typ][1]}
            report(combo, typ, n, scen, res)
