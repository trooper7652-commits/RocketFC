#!/usr/bin/env python3
"""Plot and analyze a ThrustStand run.

    python tools/plot_thrust.py runs/20260823_193000_C6-5.csv

Produces <name>_plots.png next to the input file (thrust vs time, raw and
filtered, plus cumulative impulse) with the FIRE/burnout events marked, and
prints the computed stats -- including a ready-to-paste config.h line for
RocketFC's measured e-match ignition delay.

Requires matplotlib:  pip install matplotlib

Mirrors the loader conventions of ../../tools/plot_flight.py (stdlib
csv.reader, '#'/blank-row skipping, short-row padding, EVT: extraction) so the
two plotters stay easy to cross-reference.
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

THRUST_TRIGGER_N = 0.50   # must match stand::THRUST_TRIGGER_N in config.h
BURN_END_FRAC = 0.05      # must match stand::BURN_END_FRAC


def load(path):
    """Returns (t_s, raw, thrust_n, thrust_filt_n, state, events, meta)."""
    t, raw, n, filt, state = [], [], [], [], []
    events = []
    meta = {}
    with open(path) as f:
        header = None
        for row in csv.reader(f):
            if not row:
                continue
            if row[0].startswith("#"):
                if "=" in row[0]:
                    key, _, val = row[0][1:].partition("=")
                    meta[key.strip()] = val.strip()
                continue
            if header is None:
                header = row
                continue
            if len(row) < len(header):
                row += [""] * (len(header) - len(row))
            evt = row[-1]
            if evt.startswith("EVT:"):
                parts = evt[4:].split(":")
                name = parts[0]
                val = float(parts[1]) if len(parts) > 1 and parts[1] else 0.0
                events.append((float(row[0]) / 1000.0, name, val))
                continue
            if row[1] == "":
                continue  # a data row with no state is a malformed/blank line
            t.append(float(row[0]) / 1000.0)
            raw.append(int(row[2]) if row[2] else 0)
            n.append(float(row[3]) if row[3] else 0.0)
            filt.append(float(row[4]) if row[4] else 0.0)
            state.append(row[1])
    return t, raw, n, filt, state, events, meta


def analyze(t, n, events):
    """Recomputes peak/impulse/burn-time/ignition-delay from raw samples,
    independent of what the firmware itself displayed on the LCD."""
    fire_t = next((ti for ti, name, _ in events if name == "FIRE"), None)

    peak = max(n) if n else 0.0
    onset_i = next((i for i, v in enumerate(n) if v >= THRUST_TRIGGER_N), None)
    if onset_i is None:
        return dict(peak=peak, impulse=0.0, avg=0.0, burn_s=0.0,
                    ignition_delay_ms=None, fire_t=fire_t, onset_t=None,
                    burnout_t=None)

    onset_t = t[onset_i]
    end_thresh = max(peak * BURN_END_FRAC, THRUST_TRIGGER_N)
    end_i = len(n) - 1
    for i in range(onset_i, len(n)):
        if n[i] < end_thresh:
            end_i = i
            break
    burnout_t = t[end_i]

    impulse = 0.0
    for i in range(onset_i, end_i):
        f0, f1 = max(n[i], 0.0), max(n[i + 1], 0.0)
        impulse += 0.5 * (f0 + f1) * (t[i + 1] - t[i])

    burn_s = max(burnout_t - onset_t, 1e-6)
    avg = impulse / burn_s
    delay_ms = (onset_t - fire_t) * 1000.0 if fire_t is not None else None

    return dict(peak=peak, impulse=impulse, avg=avg, burn_s=burn_s,
                ignition_delay_ms=delay_ms, fire_t=fire_t, onset_t=onset_t,
                burnout_t=burnout_t)


def nar_class(impulse_ns):
    if impulse_ns < 1.26:
        return "-"
    top = 2.5
    for c in "ABCDEFGHIJKLMNO":
        if impulse_ns <= top:
            return c
        top *= 2.0
    return "?"


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    path = sys.argv[1]
    t, raw, n, filt, state, events, meta = load(path)
    if not t:
        sys.exit(f"no data rows found in {path}")

    stats = analyze(t, n, events)
    cls = nar_class(stats["impulse"])

    print(f"peak thrust      : {stats['peak']:.2f} N")
    print(f"average thrust   : {stats['avg']:.2f} N")
    print(f"burn time        : {stats['burn_s']:.3f} s")
    print(f"total impulse    : {stats['impulse']:.2f} N*s  (class {cls})")
    if stats["ignition_delay_ms"] is not None:
        print(f"ignition delay   : {stats['ignition_delay_ms']:.0f} ms"
              "  (command -> thrust onset)")
        print(f"  -> paste into src/config.h:")
        print(f"     constexpr float IGNITION_DELAY_MS = "
              f"{stats['ignition_delay_ms']:.0f}.0f;")
    else:
        print("ignition delay   : n/a (no FIRE event and/or motor never lit)")
    if meta.get("calibrated") == "0":
        print("WARNING: run was UNCALIBRATED -- these are not trustworthy newtons")
    overload_events = [e for e in events if e[1] == "OVERLOAD"]
    if overload_events:
        print(f"WARNING: {len(overload_events)} overload event(s) -- peak may be "
              "clipped or the cell damaged")

    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    ax = axes[0]
    ax.plot(t, n, label="raw", alpha=0.4, color="tab:gray")
    ax.plot(t, filt, label="filtered", color="tab:blue")
    capacity_n = float(meta.get("cell_capacity_n", 0) or 0)
    if capacity_n > 0:
        ax.axhline(capacity_n, color="tab:red", linestyle=":", alpha=0.5,
                   label="cell capacity")
    ax.set_ylabel("thrust (N)")
    ax.set_title(os.path.basename(path))
    ax.legend(loc="upper right")

    ax = axes[1]
    cum = [0.0]
    for i in range(1, len(t)):
        f0, f1 = max(n[i - 1], 0.0), max(n[i], 0.0)
        cum.append(cum[-1] + 0.5 * (f0 + f1) * (t[i] - t[i - 1]))
    ax.plot(t, cum, color="tab:green")
    ax.set_ylabel("cumulative impulse (N*s)")
    ax.set_xlabel("t (s)")

    for ax in axes:
        for ti, name, _ in events:
            ax.axvline(ti, color="r", alpha=0.4, linestyle="--")
    for ti, name, _ in events:
        axes[0].text(ti, axes[0].get_ylim()[1], name, rotation=90, fontsize=7,
                     va="top", color="r")

    stat_text = (f"peak {stats['peak']:.1f} N\n"
                 f"avg  {stats['avg']:.1f} N\n"
                 f"impulse {stats['impulse']:.1f} N*s ({cls})\n"
                 f"burn {stats['burn_s']:.2f} s")
    if stats["ignition_delay_ms"] is not None:
        stat_text += f"\ndelay {stats['ignition_delay_ms']:.0f} ms"
    axes[0].text(0.98, 0.05, stat_text, transform=axes[0].transAxes,
                 ha="right", va="bottom", fontsize=9,
                 bbox=dict(boxstyle="round", fc="white", alpha=0.8))

    out = os.path.splitext(path)[0] + "_plots.png"
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f"\nwrote {out}")


if __name__ == "__main__":
    main()
