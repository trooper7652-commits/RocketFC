#!/usr/bin/env python3
"""Plot a RocketFC flight log (or a synthetic replay case).

    python tools/plot_flight.py FLIGHTS/flight_001.csv

Produces <name>_plots.png next to the input file: altitude/velocity, tilt +
gimbal, PID terms, and raw IMU, with state transitions and events marked.

Requires matplotlib:  pip install matplotlib
"""
import csv
import os
import sys

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    sys.exit("matplotlib is required:  pip install matplotlib")


def load(path):
    cols, events, states = {}, [], []
    with open(path) as f:
        header = None
        for row in csv.reader(f):
            if not row or row[0].startswith("#"):
                continue
            if header is None:
                header = row
                cols = {name: [] for name in header}
                continue
            if len(row) < len(header):
                row += [""] * (len(header) - len(row))
            evt = row[-1]
            if evt.startswith("EVT:"):
                events.append((float(row[0]) / 1000.0, evt[4:]))
                continue
            for name, val in zip(header, row):
                cols[name].append(val)
    t = [float(x) / 1000.0 for x in cols["t_ms"]]
    # state transition markers
    prev = None
    for ti, st in zip(t, cols["state"]):
        if st != prev:
            states.append((ti, st))
            prev = st
    def f(name):
        return [float(x) if x not in ("", None) else 0.0 for x in cols[name]]
    return t, f, events, states


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    path = sys.argv[1]
    t, f, events, states = load(path)

    fig, axes = plt.subplots(4, 1, figsize=(14, 16), sharex=True)

    ax = axes[0]
    ax.plot(t, f("baro_alt"), label="baro raw", alpha=0.5)
    ax.plot(t, f("kf_alt"), label="KF altitude")
    ax2 = ax.twinx()
    ax2.plot(t, f("kf_vel"), "g", label="KF velocity", alpha=0.7)
    ax2.set_ylabel("m/s")
    ax.set_ylabel("m")
    ax.legend(loc="upper left")
    ax2.legend(loc="upper right")
    ax.set_title(os.path.basename(path))

    ax = axes[1]
    ax.plot(t, f("tilt_deg"), label="tilt")
    ax.plot(t, f("gimx_deg"), label="gimbal X", alpha=0.7)
    ax.plot(t, f("gimy_deg"), label="gimbal Y", alpha=0.7)
    ax.set_ylabel("deg")
    ax.legend(loc="upper left")

    ax = axes[2]
    for k in ("pX", "iX", "dX", "pY", "iY", "dY"):
        ax.plot(t, f(k), label=k, alpha=0.7)
    ax.set_ylabel("PID terms (rad)")
    ax.legend(loc="upper left", ncol=6, fontsize=8)

    ax = axes[3]
    ax.plot(t, f("ax"), label="ax", alpha=0.6)
    ax.plot(t, f("ay"), label="ay", alpha=0.6)
    ax.plot(t, f("az"), label="az", alpha=0.6)
    ax.set_ylabel("m/s^2")
    ax.set_xlabel("t (s)")
    ax.legend(loc="upper left")

    for ax in axes:
        for ti, st in states:
            ax.axvline(ti, color="k", alpha=0.15)
        for ti, name in events:
            ax.axvline(ti, color="r", alpha=0.4, linestyle="--")
    for ti, st in states:
        axes[0].text(ti, axes[0].get_ylim()[1], st, rotation=90, fontsize=7,
                     va="top")
    for ti, name in events:
        axes[1].text(ti, axes[1].get_ylim()[1], name, rotation=90, fontsize=6,
                     va="top", color="r")

    out = os.path.splitext(path)[0] + "_plots.png"
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print("wrote", out)


if __name__ == "__main__":
    main()
