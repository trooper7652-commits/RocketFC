"""Part A, steps 3-5: closed-loop motor screening at 1.4 kg liftoff.

  stage 1: every built pair, seed 101, 200 flights, measured spreads;
  stage 2: the best pairs that pass the apogee gate, dev seeds 101-103 (600);
  stage 3: the top 3, holdout seeds 701-710 (2,000 flights).

Apogee gate: p99 <= 30.48 m (100 ft) over the Monte Carlo spreads.
usage: python screen.py 1 | 2 [N] | 3 PAIR PAIR PAIR
"""
import glob
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mclib as L  # noqa: E402


def combos():
    out = []
    for p in sorted(glob.glob(os.path.join(L.WS, "c_*", "combo.json"))):
        out.append(json.load(open(p)))
    return out


def short(name):
    return name[2:].replace("AeroTech_", "").replace("Cesaroni_", "CTI ").replace("Roadrunner_", "RR ") \
        .replace("Piotr_", "PT ").replace("Ellis_", "EM ").replace("Estes_", "Estes ").replace("__", " / ")


def table(results, info, sort=True):
    print("| Ascent / landing | Success (95 % CI) | Apogee med / p99 / max | Contact vz med / p90 | "
          "Lifted off | Legs late | Hops | Aborts | Casing |")
    print("|---|---|---|---|---|---|---|---|---|")
    items = list(results.items())
    if sort:
        items.sort(key=lambda kv: -L.metrics(kv[1])["k"])
    for name, rows in items:
        m = L.metrics(rows)
        lo, hi = L.wilson(m["k"], m["n"])
        gate = "" if m["ap_p99"] <= L.APOGEE_LIMIT else " **over 100 ft**"
        print("| %s | %.1f %% (%.0f-%.0f) | %.1f / %.1f / %.1f m%s | %.2f / %.2f | %.0f %% | %.0f %% | %.0f %% | %.1f %% | %.0f g |" % (
            short(name), 100.0 * m["k"] / m["n"], lo, hi, m["ap_med"], m["ap_p99"], m["ap_max"], gate,
            m["vz_med"], m["vz_p90"], 100 * m["lifted"], 100 * m["legs_late"], 100 * m["hop"],
            100 * m["abort"], 1000 * info[name]["casing_kg"]), flush=True)


if __name__ == "__main__":
    stage = sys.argv[1]
    info = {c["name"]: c for c in combos()}
    if stage == "1":
        R = {}
        for name in info:
            R[name] = L.run("s1_" + name, ws=name, seeds=(101,), n=200)
        print("\n== Stage 1: seed 101, 200 flights each, measured spreads, 1.4 kg liftoff")
        table(R, info)
    elif stage == "2":
        top = int(sys.argv[2]) if len(sys.argv) > 2 else 12
        s1 = {name: L.run("s1_" + name, ws=name, seeds=(101,), n=200) for name in info}
        ok = [n for n in s1 if L.metrics(s1[n])["ap_p99"] <= L.APOGEE_LIMIT + 1.0]
        ok.sort(key=lambda n: -L.metrics(s1[n])["k"])
        R = {name: L.run("dev_" + name, ws=name, seeds=L.DEV, n=200) for name in ok[:top]}
        print("\n== Stage 2: dev seeds 101-103, 600 flights each, measured spreads")
        table(R, info)
    elif stage == "3":
        R = {name: L.run("hold_" + name, ws=name, seeds=L.HOLD, n=200) for name in sys.argv[2:]}
        print("\n== Stage 3: holdout seeds 701-710, 2,000 flights each, measured spreads")
        table(R, info, sort=False)
        names = sys.argv[2:]
        for b in names[1:]:
            print("  %s vs %s: %d rescued / %d lost, p=%.3g" % ((short(b), short(names[0])) + L.paired(R[names[0]], R[b])))
