"""pair.py BASE.csv.gz VAR.csv.gz [...]: paired comparison by (seed, i); sign test on rescued vs lost.
Only flights present in both (so a screen job pairs against the first 100 of each dev seed)."""
import csv, gzip, sys
from math import comb
def load(p): return {(r["seed"], r["i"]): r for r in csv.DictReader(gzip.open(p, "rt"))}
def sign_p(a, b):
    n = a + b
    if n == 0: return 1.0
    k = min(a, b); return min(1.0, 2 * sum(comb(n, j) for j in range(k + 1)) / 2 ** n)
def num(v):
    try: return float(v)
    except (TypeError, ValueError): return None
def pct(xs, q):
    xs = sorted(xs); return xs[min(len(xs) - 1, int(q * len(xs)))] if xs else float("nan")
def compare(bp, vp):
    B, V = load(bp), load(vp); keys = sorted(set(B) & set(V))
    sb = sum(int(B[k]["success"]) for k in keys); sv = sum(int(V[k]["success"]) for k in keys)
    resc = sum(1 for k in keys if V[k]["success"] == "1" and B[k]["success"] == "0")
    lost = sum(1 for k in keys if V[k]["success"] == "0" and B[k]["success"] == "1")
    n = len(keys); ap = [num(V[k]["apogee_h"]) for k in keys if num(V[k]["apogee_h"]) is not None]
    codes = {}
    for k in keys: codes[V[k]["code"]] = codes.get(V[k]["code"], 0) + 1
    return dict(n=n, base=100 * sb / n, var=100 * sv / n, resc=resc, lost=lost, p=sign_p(resc, lost),
                ap=(pct(ap, .5), pct(ap, .99), max(ap)), codes=codes)
if __name__ == "__main__":
    base = sys.argv[1]
    for v in sys.argv[2:]:
        c = compare(base, v)
        mix = " ".join(f"{k}:{n}" for k, n in sorted(c["codes"].items(), key=lambda kv: -kv[1]))
        print(f"{v.split('/')[-1]:28s} n={c['n']} {c['base']:.1f}->{c['var']:.1f} % +{c['resc']}/-{c['lost']} p={c['p']:.3g} "
              f"ap {c['ap'][0]:.1f}/{c['ap'][1]:.1f}/{c['ap'][2]:.1f} | {mix}")
