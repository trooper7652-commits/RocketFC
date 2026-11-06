"""Trace plot for one flight with and without artificial throttling (ATH):
height, falling speed vs the braking curve, commanded lean vs actual tilt,
and sideways speed, from ignition to ground contact.

usage: py ath_trace.py OUT.png TH MARGIN K [x1.10] [SEED:I]"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(os.path.dirname(HERE), "sw", "cand", "tools", "sim")
sys.path.insert(0, SIM)
os.chdir(SIM)
import core  # noqa: E402
import mc_batch  # noqa: E402
import run  # noqa: E402
import sweep  # noqa: E402
import vehicle  # noqa: E402

out, th, mg, k = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
apo = "x1.10" in sys.argv
spec = next((a for a in sys.argv[5:] if ":" in a), None)
BEST = {"DRIFT_GAIN": "2.0f", "TILT_PREDICT_INTERLOCK": "true", "GPS_VEL_GAIN": "0.3f"}
ATH = dict(BEST, THR_TILT_MAX_DEG="{:.1f}f".format(th),
           BURN_TABLE_MARGIN_M="{:.1f}f".format(mg), THR_K="{:.1f}f".format(k))


def fly(fw):
    base = vehicle.default_vehicle_config(core.default_dll())
    base.gps_hz = 10.0
    if apo:
        base.ascent_eng_path = os.path.join(HERE, "motors", "F15_x1.10.eng")
    if spec:
        seed, i = (int(x) for x in spec.split(":"))
        vc, _, fseed, _ = mc_batch.build_jobs(base, list(sweep.DISPERSION_FIELDS), seed, i + 1, [])[i]
    else:
        vc, fseed = base, 7
    return run.simulate(vc, mode="land", dll_path=core.build_with_overrides(fw), seed=fseed)


import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

SURF, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
BLUE, ORANGE = "#2a78d6", "#eb6834"
fig, axs = plt.subplots(4, 1, figsize=(8.5, 9.5), sharex=True, facecolor=SURF)
for lab, fw, col in (("without tilt-throttling", BEST, BLUE),
                     ("with tilt-throttling", ATH, ORANGE)):
    r = fly(fw)
    s = r["summary"]
    ig = s["land_ignite_t"]
    c = s["ground"]["contact"]
    end = (s["ground"].get("fall_t") or c["t"]) + 0.3
    rows = [x for x in r["rows"] if ig - 0.3 <= x["t"] <= min(end, c["t"] + 1.5)]
    t = [x["t"] - ig for x in rows]
    vh, prev = [], None
    for x in rows:
        if prev is None:
            vh.append(0.0)
        else:
            dt = x["t"] - prev["t"]
            vh.append(math.hypot(x["x_true"] - prev["x_true"], x["y_true"] - prev["y_true"]) / dt)
        prev = x
    name = "{} -- {}".format(lab, s["landing_outcome"]["label"])
    axs[0].plot(t, [x["h_true"] for x in rows], color=col, lw=2, label=name)
    axs[1].plot(t, [-x["v_true"] for x in rows], color=col, lw=2)
    if fw is ATH:
        # The braking curve the firmware steers to (truth height, capped braking).
        vdes = [math.sqrt(max(0.0, 2 * 0.85 * min(5.4, max(0.0, x["thrust_n"] / 0.92 - 9.81)) *
                              max(x["h_true"], 0.0))) if x["thrust_n"] > 3 else float("nan")
                for x in rows]
        axs[1].plot(t, vdes, color=col, lw=1, ls=":", label="braking curve (lean if slower)")
        axs[2].plot(t, [x.get("thr_lean_deg", 0.0) for x in rows], color=col, lw=1, ls=":",
                    label="commanded throttling lean")
    axs[2].plot(t, [x["tilt_deg"] for x in rows], color=col, lw=2)
    axs[3].plot(t, vh, color=col, lw=2)
for ax, yl in zip(axs, ("height (m)", "falling speed (m/s)", "tilt (deg)", "sideways speed (m/s)")):
    ax.set_facecolor(SURF)
    ax.set_ylabel(yl, color=INK2, fontsize=9)
    ax.grid(color=GRID)
    ax.tick_params(colors=INK2, labelsize=8)
    for sd in ("top", "right"):
        ax.spines[sd].set_visible(False)
    for sd in ("left", "bottom"):
        ax.spines[sd].set_color(GRID)
axs[3].axhline(0.85, color=INK2, lw=1, ls="--")
axs[3].text(0.02, 0.9, "tip-over limit ~0.85 m/s", color=INK2, fontsize=8,
            transform=axs[3].get_yaxis_transform())
axs[-1].set_xlabel("time since the landing motor lit (s)", color=INK2, fontsize=9)
axs[0].legend(frameon=False, fontsize=8, loc="upper right")
axs[1].legend(frameon=False, fontsize=8, loc="upper right")
axs[2].legend(frameon=False, fontsize=8, loc="upper right")
axs[0].set_title("Tilt-throttling trace: max lean {:.0f}°, fire {:+.0f} m, gain {:.0f}°/(m/s){}".format(
    th, mg, k, ", apogee x1.10" if apo else ""), color=INK, fontsize=11, loc="left")
fig.tight_layout()
fig.savefig(out, dpi=120, facecolor=SURF)
print("saved", out)
