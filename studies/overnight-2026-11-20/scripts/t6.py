"""task 6 table: per DUD_WINDOW_MS x true delay: false-dud rate, outcome, after-light metrics; real duds: chute release height."""
import csv, gzip, glob, os, statistics as st
def L(p): return list(csv.DictReader(gzip.open(p, "rt")))
def f(v):
    try: return float(v)
    except: return None
def med(xs): xs=[x for x in xs if x is not None]; return st.median(xs) if xs else float("nan")
print("| window ms | true delay s | dud aborts | success | med contact v | med max h after light | med max tilt after light | med chute release h |")
print("|---|---|---|---|---|---|---|---|")
for W in (200, 300, 400):
    for X in ("0.62","0.70","0.76","0.80","0.84","0.90","1.00","dud"):
        p = f"results/T6_d{X}_w{W}.csv.gz" if X!="dud" else f"results/T6_dud_w{W}.csv.gz"
        if not os.path.exists(p): continue
        R = L(p); n=len(R)
        dud = sum(r["abort_reason"]=="DUD_IGNITER" for r in R); ok=sum(r["success"]=="1" for r in R)
        print(f"| {W} | {X} | {100*dud/n:.0f} % | {100*ok/n:.1f} % | {med(f(r['contact_vspeed_mps']) for r in R):.2f} | "
              f"{med(f(r['post_ign_max_h']) for r in R):.2f} | {med(f(r['post_ign_max_tilt']) for r in R):.1f} | {med(f(r['chute_release_h']) for r in R):.2f} |")
