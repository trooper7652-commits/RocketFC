"""Success and aborts by wind speed: today's best setup vs. the wind-matched ascent."""
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(HERE, "results")
runs = [(a, b) for a, b in (x.split("=", 1) for x in sys.argv[2:])]
out = sys.argv[1]
bands = [(0, 1), (1, 2), (2, 3), (3, 4.01)]
fig, axes = plt.subplots(1, 2, figsize=(10, 4.2))
w = 0.8 / len(runs)
for j, (label, name) in enumerate(runs):
    rows = [r for r in json.load(open(os.path.join(RES, name + ".json")))["rows"] if "error" not in r["summary"]]
    succ, ab = [], []
    for lo, hi in bands:
        sub = [r for r in rows if lo <= abs(r["dispersions"]["wind_mps"]) < hi]
        succ.append(100.0 * sum(r["summary"]["landing_outcome"]["success"] for r in sub) / len(sub))
        ab.append(100.0 * sum(1 for r in sub if r["summary"].get("abort_reason")) / len(sub))
    xs = [i + (j - (len(runs) - 1) / 2) * w for i in range(len(bands))]
    axes[0].bar(xs, succ, w, label=label)
    axes[1].bar(xs, ab, w, label=label)
for ax, t in zip(axes, ["Successful landings (%)", "Aborted to the parachute (%)"]):
    ax.set_xticks(range(len(bands)))
    ax.set_xticklabels(["0-1", "1-2", "2-3", "3-4"])
    ax.set_xlabel("wind (m/s)")
    ax.set_title(t)
    ax.grid(axis="y", alpha=0.3)
handles, labels = axes[0].get_legend_handles_labels()
fig.legend(handles, labels, loc="lower center", ncol=len(runs), fontsize=9, frameon=False)
fig.tight_layout(rect=(0, 0.08, 1, 1))
fig.savefig(out, dpi=110)
print("wrote", out)
