#!/usr/bin/env python3
"""Summarise mc_batch.py result files as a markdown table, optionally paired.

    python tools/sim/mc_summary.py run_a.json run_b.json ...
    python tools/sim/mc_summary.py --base run_a.json run_b.json run_c.json

Per file: landing success (with a 95 % Wilson range), calm-wind success
(|wind| < 1 m/s), powered landings, aborts, and among powered landings the
share that hit too hard, got lifted back off, hopped, had the legs late or
tipped over, plus median contact speeds, apogee and fall speed at fire.

--base pairs every other file with it flight by flight (same seed and index):
landings rescued / lost and a two-sided sign test. Files must come from the
same --seeds / --n for the pairing to mean anything.
"""
import argparse
import collections
import json
import math
import os


def wilson(k, n, z=1.96):
    if n == 0:
        return 0.0, 0.0, 0.0
    p = k / n
    den = 1 + z * z / n
    mid = (p + z * z / (2 * n)) / den
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return p, max(0.0, mid - half), min(1.0, mid + half)


def median(a):
    a = sorted(x for x in a if x is not None)
    return a[len(a) // 2] if a else float("nan")


def load(path):
    return [r for r in json.load(open(path, encoding="utf-8"))["rows"] if "error" not in r["summary"]]


def metrics(rows):
    n = len(rows)
    outs = [r["summary"]["landing_outcome"] for r in rows]
    k = sum(o["success"] for o in outs)
    calm = [o for r, o in zip(rows, outs) if abs(r["dispersions"].get("wind_mps", 0.0)) < 1.0]
    powered = [(r, o) for r, o in zip(rows, outs) if "NO_POWERED_LANDING" not in o["factors"]]
    fac = collections.Counter(f for _, o in powered for f in set(o["factors"]))
    npw = max(len(powered), 1)
    contacts = [(r["summary"].get("ground") or {}).get("contact") for r, _ in powered]
    contacts = [c for c in contacts if c]
    return {
        "n": n, "k": k, "rate": wilson(k, n),
        "calm": (sum(o["success"] for o in calm), len(calm)),
        "powered": len(powered) / max(n, 1),
        "abort": sum(1 for r in rows if r["summary"].get("abort_reason")) / max(n, 1),
        "crash": fac["LEGS_BROKEN"] / npw, "lifted": fac["LIFTED_OFF"] / npw,
        "hopped": fac["HOPPED"] / npw, "late": fac["LEGS_NOT_DOWN"] / npw,
        "tipped": fac["TIPPED_OVER"] / npw,
        "vz": median([abs(c["vel"][2]) for c in contacts]),
        "vh": median([math.hypot(c["vel"][0], c["vel"][1]) for c in contacts]),
        "apogee": median([r["summary"]["apogee_h"] for r in rows]),
        "vfire": median([-r["summary"]["fire_v_true"] for r, _ in powered
                         if r["summary"].get("fire_v_true") is not None]),
    }


def paired(base, rows):
    a = {(r["seed"], r["i"]): r["summary"]["landing_outcome"]["success"] for r in base}
    b = {(r["seed"], r["i"]): r["summary"]["landing_outcome"]["success"] for r in rows}
    common = a.keys() & b.keys()
    rescued = sum(1 for key in common if b[key] and not a[key])
    lost = sum(1 for key in common if a[key] and not b[key])
    m = rescued + lost
    p = 1.0
    if m:
        p = min(1.0, 2 * sum(math.comb(m, i) for i in range(min(rescued, lost) + 1)) / 2 ** m)
    return rescued, lost, p, len(common)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("files", nargs="+", help="mc_batch.py --out files")
    ap.add_argument("--base", help="pair every file against this one")
    args = ap.parse_args()
    files = ([args.base] if args.base else []) + [f for f in args.files if f != args.base]
    base = load(args.base) if args.base else None
    pct = lambda x: "{:.1f} %".format(100.0 * x)
    head = ("| Run | Success [95 % range] | Calm wind | Powered | Aborts | Hit too hard | Lifted off | "
            "Hopped | Legs late | Tipped | Contact vz / vh (m/s) | Apogee (m) | Fall at fire (m/s) |")
    if base is not None:
        head += " Paired vs base |"
    print(head)
    print("|" + "---|" * (head.count("|") - 1))
    for f in files:
        rows = load(f)
        m = metrics(rows)
        p, lo, hi = m["rate"]
        line = ("| {} | **{}** [{:.1f}-{:.1f}] ({}/{}) | {}/{} | {} | {} | {} | {} | {} | {} | {} | "
                "{:.1f} / {:.2f} | {:.1f} | {:.1f} |").format(
            os.path.splitext(os.path.basename(f))[0], pct(p), 100 * lo, 100 * hi, m["k"], m["n"],
            m["calm"][0], m["calm"][1], pct(m["powered"]), pct(m["abort"]), pct(m["crash"]),
            pct(m["lifted"]), pct(m["hopped"]), pct(m["late"]), pct(m["tipped"]),
            m["vz"], m["vh"], m["apogee"], m["vfire"])
        if base is not None:
            if f == args.base:
                line += " (base) |"
            else:
                r, l, pv, nc = paired(base, rows)
                line += " {} rescued / {} lost, p = {:.3g} ({} flights) |".format(r, l, pv, nc)
        print(line)


if __name__ == "__main__":
    main()
