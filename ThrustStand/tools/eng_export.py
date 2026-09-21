#!/usr/bin/env python3
"""Export a ThrustStand run as a RASP .eng thrust-curve file.

    python tools/eng_export.py runs/20260823_193000_C6-5.csv \\
        --name MyC6 --diam 18 --len 70 --delay 5 \\
        --prop-g 12.5 --total-g 23.0 --mfr Estes

prop-g/total-g come from weighing the motor on a kitchen scale before and
after the burn (propellant mass = before - after; total mass = before, or the
motor's printed spec if you'd rather trust that). If you skip them, rough
defaults are written and flagged with a comment -- the .eng will still parse
and simulate, but the delta-v numbers downstream will be off.

The output matches exactly what ../../tools/burn_table.py's Motor class
parses (header: name diam len delays prop_kg total_kg mfr; then time/thrust
pairs; ';' comments) so it drops straight into the existing pipeline:

    python tools/burn_table.py --motor tools/motors/MyC6.eng

The curve is decimated to ~30 points (peak-preserving) because RASP files are
meant to be hand-scannable and burn_table.py's linear interpolation does not
need every 12.5 ms sample to reproduce the shape well.
"""
import argparse
import csv
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from plot_thrust import load, analyze  # noqa: E402


def decimate(t, n, onset_t, burnout_t, max_points=30):
    """Keep every sample above threshold from onset to burnout, always
    including the peak, then thin evenly down to max_points."""
    pts = [(ti - onset_t, max(v, 0.0)) for ti, v in zip(t, n)
           if onset_t <= ti <= burnout_t]
    if len(pts) <= max_points:
        return pts
    peak_i = max(range(len(pts)), key=lambda i: pts[i][1])
    step = len(pts) / float(max_points)
    keep_i = sorted({int(round(i * step)) for i in range(max_points)}
                    | {0, len(pts) - 1, peak_i})
    return [pts[i] for i in keep_i]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", help="ThrustStand run CSV")
    ap.add_argument("--name", help="motor designation, e.g. C6-5 "
                    "(default: derived from the CSV filename)")
    ap.add_argument("--diam", type=float, default=18.0, help="mm")
    ap.add_argument("--len", type=float, default=70.0, help="mm")
    ap.add_argument("--delay", default="0", help="ejection delay(s), e.g. '5' "
                    "or '3-5-7' for multiple; use '0' if not applicable")
    ap.add_argument("--prop-g", type=float, default=None,
                    help="propellant mass in grams (before - after burn)")
    ap.add_argument("--total-g", type=float, default=None,
                    help="loaded motor mass in grams")
    ap.add_argument("--mfr", default="Unknown")
    ap.add_argument("--out", default=None,
                    help="default: ../../tools/motors/<name>.eng")
    args = ap.parse_args()

    t, raw, n, filt, state, events, meta = load(args.csv)
    if not t:
        sys.exit(f"no data rows found in {args.csv}")
    stats = analyze(t, n, events)
    if stats["onset_t"] is None:
        sys.exit("motor never lit in this run (no sample crossed the thrust "
                  "trigger) -- nothing to export")

    name = args.name or os.path.splitext(os.path.basename(args.csv))[0]
    prop_kg = (args.prop_g / 1000.0) if args.prop_g else 0.001
    total_kg = (args.total_g / 1000.0) if args.total_g else prop_kg * 2.0
    if args.prop_g is None or args.total_g is None:
        print("NOTE: --prop-g/--total-g not given; writing placeholder masses. "
              "Weigh the motor before and after the burn for real numbers.")

    pts = decimate(t, n, stats["onset_t"], stats["burnout_t"])
    if pts[-1][1] != 0.0:
        pts.append((pts[-1][0] + 0.01, 0.0))  # RASP files end at zero thrust

    here = os.path.dirname(__file__)
    out = args.out or os.path.join(here, "..", "..", "tools", "motors",
                                    f"{name}.eng")
    out = os.path.abspath(out)
    os.makedirs(os.path.dirname(out), exist_ok=True)

    with open(out, "w", newline="\n") as f:
        f.write(f"; {name} -- measured on ThrustStand from {os.path.basename(args.csv)}\n")
        f.write(f"; peak {stats['peak']:.2f} N, impulse {stats['impulse']:.2f} N*s, "
                f"burn {stats['burn_s']:.2f} s\n")
        if args.prop_g is None or args.total_g is None:
            f.write("; WARNING: prop/total mass are placeholders, not measured\n")
        f.write("; name diam(mm) len(mm) delays prop_mass(kg) total_mass(kg) mfr\n")
        f.write(f"{name} {args.diam:g} {args.len:g} {args.delay} "
                f"{prop_kg:.4f} {total_kg:.4f} {args.mfr}\n")
        for ti, fi in pts:
            f.write(f"   {ti:.3f} {fi:.3f}\n")

    print(f"wrote {out}  ({len(pts)} points)")
    print(f"verify with:  python tools/burn_table.py --eng {out}")


if __name__ == "__main__":
    main()
