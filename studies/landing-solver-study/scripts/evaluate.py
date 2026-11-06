"""Apply the pre-registered PASS criteria (notes/03_metrics.md) to the holdout
runs, clean sweeps and robustness gates; write notes/04_results.md + plots."""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
L = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import metrics as M  # noqa: E402

import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

D = os.path.join(L, "data")
P = os.path.join(L, "notes", "plots")


def clean_points(path):
    return {c["j"]: c for c in M.clean_table(M.load(path))}


def track(name, hb, hc, cb, cc, rob):
    b, c = M.load(os.path.join(D, hb)), M.load(os.path.join(D, hc))
    sb, sc, p = M.stats(b), M.stats(c), M.paired(b, c)
    crit = {}
    crit["1 median -0.5 & CI<0"] = p["d_median"] <= -0.5 and p["d_median_ci"][1] < 0
    crit["2 p90 lower & CI<0"] = p["d_p90"] < 0 and p["d_p90_ci"][1] < 0
    crit["3 hops <= baseline"] = sc["hops"] <= sb["hops"]
    reasons = set(sb["aborts"]) | set(sc["aborts"])
    crit["4 aborts <= baseline (each reason)"] = all(sc["aborts"].get(r, 0) <= sb["aborts"].get(r, 0) for r in reasons)
    pb, pc = clean_points(os.path.join(D, cb)), clean_points(os.path.join(D, cc))
    common = [j for j in pb if j in pc and pb[j]["n_powered"] and pc[j]["n_powered"]]
    mb = np.mean([pb[j]["td_mean"] for j in common])
    mcn = np.mean([pc[j]["td_mean"] for j in common])
    worst = max(pc[j]["td_mean"] - pb[j]["td_mean"] for j in common)
    new_ab = sum(len(pc[j]["aborts"]) for j in pc) > sum(len(pb[j]["aborts"]) for j in pb)
    crit["5 clean sweep better, no pt >0.3 worse"] = mcn < mb and worst <= 0.3 and not new_ab
    rp = os.path.join(D, f"robustness_{rob}.txt")
    rtxt = open(rp, encoding="utf-8").read() if os.path.exists(rp) else "(not run)"
    crit["6 robustness gate"] = "GATE: PASS" in rtxt
    passed = all(crit.values())
    return {"name": name, "sb": sb, "sc": sc, "p": p, "crit": crit, "passed": passed,
            "clean": (pb, pc, mb, mcn, worst), "rob": rtxt, "b": b, "c": c}


def md_track(t):
    sb, sc, p = t["sb"], t["sc"], t["p"]
    ab = lambda s: ", ".join(f"{k} {v}" for k, v in sorted(s["aborts"].items()))
    out = [f"### {t['name']}", "",
           "| | powered | median | mean | p90 | soft <2.5 | <1 | hops (severe) | aborts |",
           "|---|---|---|---|---|---|---|---|---|",
           f"| baseline | {sb['powered']}/{sb['flights']} | {sb['median']:.2f} | {sb['mean']:.2f} | {sb['p90']:.2f} | {sb['soft']} | {sb['very_soft']} | {sb['hops']} ({sb['severe_hops']}) | {ab(sb)} |",
           f"| candidate | {sc['powered']}/{sc['flights']} | {sc['median']:.2f} | {sc['mean']:.2f} | {sc['p90']:.2f} | {sc['soft']} | {sc['very_soft']} | {sc['hops']} ({sc['severe_hops']}) | {ab(sc)} |",
           "",
           f"Paired over {p['pairs']} flights that landed under power in both: "
           f"Δmedian **{p['d_median']:+.2f}** [95 % CI {p['d_median_ci'][0]:+.2f}, {p['d_median_ci'][1]:+.2f}], "
           f"Δp90 **{p['d_p90']:+.2f}** [{p['d_p90_ci'][0]:+.2f}, {p['d_p90_ci'][1]:+.2f}], "
           f"Δmean {p['d_mean']:+.2f} [{p['d_mean_ci'][0]:+.2f}, {p['d_mean_ci'][1]:+.2f}]; "
           f"better/worse/same {p['better']}/{p['worse']}/{p['same']}; "
           f"powered only in baseline {p['powered_only_base']}, only in candidate {p['powered_only_cand']}.",
           "", "| criterion | result |", "|---|---|"]
    out += [f"| {k} | {'PASS' if v else 'FAIL'} |" for k, v in t["crit"].items()]
    pb, pc, mb, mcn, worst = t["clean"]
    out += ["", f"Clean apogee sweep: mean touchdown baseline {mb:.2f} vs candidate {mcn:.2f} m/s, "
            f"worst point change {worst:+.2f} m/s.", "",
            f"**Verdict: {'PASS' if t['passed'] else 'FAIL'}**", ""]
    out += ["Robustness (seeds 101–103):", "```", t["rob"].strip(), "```", ""]
    return "\n".join(out)


def plots(tracks):
    fig, axes = plt.subplots(1, len(tracks), figsize=(6 * len(tracks), 4.5))
    for ax, t in zip(np.atleast_1d(axes), tracks):
        for rows, lab in ((t["b"], "baseline (current table)"), (t["c"], "candidate")):
            v = np.sort([r["summary"]["touchdown_speed_mps"] for r in rows if M.is_powered(r["summary"])])
            ax.plot(v, np.arange(1, len(v) + 1) / len(v), label=f"{lab} (n={len(v)})")
        ax.axvline(2.5, color="g", ls=":", label="soft (2.5 m/s)")
        ax.set_xlabel("touchdown speed (m/s)"); ax.set_ylabel("fraction of powered landings at or below")
        ax.set_title(t["name"]); ax.grid(alpha=0.3); ax.legend(fontsize=8); ax.set_xlim(0, 35)
    fig.tight_layout(); fig.savefig(os.path.join(P, "holdout_cdf.png"), dpi=110); plt.close(fig)

    fig, axes = plt.subplots(1, len(tracks), figsize=(6 * len(tracks), 5))
    for ax, t in zip(np.atleast_1d(axes), tracks):
        b = {M.key(r): r["summary"] for r in t["b"]}
        c = {M.key(r): r["summary"] for r in t["c"]}
        ks = [k for k in b if M.is_powered(b[k]) and M.is_powered(c[k])]
        x = np.array([b[k]["touchdown_speed_mps"] for k in ks]); y = np.array([c[k]["touchdown_speed_mps"] for k in ks])
        tl = np.array([b[k]["fire_tilt_prev_deg"] >= 15 for k in ks])
        ax.scatter(x[~tl], y[~tl], s=8, alpha=0.6, label="baseline fire not tilt-limited")
        ax.scatter(x[tl], y[tl], s=8, alpha=0.6, color="r", label="baseline fire tilt-limited")
        m = max(x.max(), y.max()); ax.plot([0, m], [0, m], "k--", lw=0.8)
        ax.set_xlabel("baseline touchdown (m/s)"); ax.set_ylabel("candidate touchdown (m/s)")
        ax.set_title(t["name"] + " (same flights)"); ax.legend(fontsize=8); ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(os.path.join(P, "holdout_paired_scatter.png"), dpi=110); plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 4.5))
    pb = tracks[0]["clean"][0]
    ap = [pb[j]["apogee"] for j in sorted(pb)]
    ax.plot(ap, [pb[j]["td_mean"] for j in sorted(pb)], "ko-", label="baseline")
    for t in tracks:
        pc = t["clean"][1]
        ax.plot([pc[j]["apogee"] for j in sorted(pc)], [pc[j]["td_mean"] for j in sorted(pc)], "o-", label=t["name"])
    ax.set_xlabel("apogee (m)"); ax.set_ylabel("mean touchdown speed, 3 noise seeds (m/s)")
    ax.set_title("Clean flights (no dispersions) across apogee"); ax.grid(alpha=0.3); ax.legend()
    fig.tight_layout(); fig.savefig(os.path.join(P, "clean_apogee_sweep.png"), dpi=110); plt.close(fig)


def main():
    tracks = [track("Track 1: lag fix + risk-aware aim (B+C)", "hold1_base.json", "hold1_cand.json",
                    "base_clean_sweep.json", "clean_track1.json", "track1"),
              track("Track 2: B+C + predicted-tilt interlock (D2)", "hold2_base.json", "hold2_cand.json",
                    "base_clean_sweep.json", "clean_track2.json", "track2")]
    plots(tracks)
    txt = "# 04 — Results (holdout, clean sweep, robustness)\n\n" + "\n".join(md_track(t) for t in tracks)
    with open(os.path.join(L, "notes", "04_results.md"), "w", encoding="utf-8") as f:
        f.write(txt)
    for t in tracks:
        print(t["name"], "->", "PASS" if t["passed"] else "FAIL", {k: v for k, v in t["crit"].items()})
        print("   ", M.fmt_stats("baseline", t["sb"]))
        print("   ", M.fmt_stats("candidate", t["sc"]))
        print("   ", M.fmt_paired("paired", t["p"]))


if __name__ == "__main__":
    main()
