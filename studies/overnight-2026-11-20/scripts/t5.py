"""task 5: missed-window flights, chute release height, paired landings. usage: t5.py base.csv.gz var.csv.gz ..."""
import csv, gzip, sys, statistics as st
sys.path.insert(0, __file__.rsplit("/", 1)[0]); from pair import compare
def L(p): return {(r["seed"], r["i"]): r for r in csv.DictReader(gzip.open(p, "rt"))}
def f(v):
    try: return float(v)
    except: return None
base = sys.argv[1]; B = L(base)
def mw(R): return [k for k, r in R.items() if r["abort_reason"] == "MISSED_WINDOW"]
def hs(R, ks): return sorted(h for h in (f(R[k]["chute_release_h"]) for k in ks) if h is not None)
def desc(h):
    if not h: return "none"
    return f"n={len(h)} med {st.median(h):.2f} p10 {h[int(.1*len(h))]:.2f} >6m {100*sum(x>6 for x in h)/len(h):.0f} % >8m {100*sum(x>8 for x in h)/len(h):.0f} %"
print(f"{base.split('/')[-1]}: missed-window aborts {desc(hs(B, mw(B)))}")
for v in sys.argv[2:]:
    V = L(v); c = compare(base, v); kv = mw(V)
    early_lost = sum(1 for k in kv if B[k]["success"] == "1")
    print(f"{v.split('/')[-1]}: {c['base']:.1f}->{c['var']:.1f} % +{c['resc']}/-{c['lost']} p={c['p']:.3g}; MW aborts {desc(hs(V, kv))}; "
          f"aborted but base landed: {early_lost}; same flights in base: {desc(hs(B, kv))}")
