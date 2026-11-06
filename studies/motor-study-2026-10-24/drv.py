"""Motor / leg / software-fix study driver (2026-10-24). Runs mc_batch.py in a
workspace copy (cached by label) and computes the pre-registered metrics."""
import collections
import json
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(HERE, "results")
PY = r"C:\Users\troop\AppData\Local\Programs\Python\Python312\python.exe"
os.makedirs(RES, exist_ok=True)
MEAS = {"mass_descent_kg": 0.01, "ignition_delay_s": 0.05}
DEV = (101, 102, 103)
HOLD = tuple(range(701, 711))
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")


def ws_sim(name):
    return os.path.join(HERE, "ws", name, "tools", "sim")


def run(label, ws="fix", seeds=DEV, n=200, fw=None, plant=None, disp=MEAS, ext=None,
        plant_list=None, force=False):
    out = os.path.join(RES, label + ".json")
    if os.path.exists(out) and not force:
        return json.load(open(out))["rows"]
    cmd = [PY, "mc_batch.py", "--seeds", *map(str, seeds), "--n", str(n), "--out", out]
    if fw:
        cmd += ["--fw-overrides", json.dumps(fw)]
    if plant:
        cmd += ["--plant", *["{}={}".format(k, json.dumps(v)) for k, v in plant.items()]]
    if disp:
        cmd += ["--disp", *["{}={}".format(k, v) for k, v in disp.items()]]
    if ext:
        cmd += ["--ext", *ext]
    if plant_list is not None:
        cmd += ["--plant-list", json.dumps(plant_list)]
    r = subprocess.run(cmd, cwd=ws_sim(ws), capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(label + ": " + r.stderr[-3000:])
    print("  ran", label, r.stdout.strip().splitlines()[-1], flush=True)
    return json.load(open(out))["rows"]


def wilson(k, n, z=1.96):
    if n == 0:
        return 0.0, 0.0, 0.0
    p = k / n
    den = 1 + z * z / n
    mid = (p + z * z / (2 * n)) / den
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return p, max(0.0, mid - half), min(1.0, mid + half)


def q(a, p):
    a = sorted(x for x in a if x is not None)
    return a[min(len(a) - 1, int(p * len(a)))] if a else float("nan")


def ok(rows):
    return [r for r in rows if "error" not in r["summary"]]


def contact_t(s):
    c = (s.get("ground") or {}).get("contact") or {}
    return c.get("t", s.get("touchdown_t"))


def metrics(rows, calm=False):
    rows = ok(rows)
    if calm:
        rows = [r for r in rows if abs(r["dispersions"].get("wind_mps", 0.0)) < 1.0]
    n = len(rows)
    outs = [r["summary"]["landing_outcome"] for r in rows]
    k = sum(o["success"] for o in outs)
    powered = [r for r, o in zip(rows, outs) if "NO_POWERED_LANDING" not in o["factors"]]
    npw = len(powered)
    fac = collections.Counter(f for r in powered for f in set(r["summary"]["landing_outcome"]["factors"]))
    cs = [r["summary"]["ground"]["contact"] for r in powered if (r["summary"].get("ground") or {}).get("contact")]
    vz = [abs(c["vel"][2]) for c in cs]
    vh = [math.hypot(c["vel"][0], c["vel"][1]) for c in cs]
    aborts = sum(1 for r in rows if r["summary"].get("abort_reason"))
    ap = [r["summary"]["apogee_h"] for r in rows]
    fv = [-r["summary"]["fire_v_true"] for r in powered if r["summary"].get("fire_v_true") is not None]
    return {"n": n, "k": k, "rate": wilson(k, n), "powered": npw / max(n, 1),
            "vz_med": q(vz, .5), "vh_med": q(vh, .5), "ap_med": q(ap, .5), "fv_med": q(fv, .5),
            "legs_late": fac["LEGS_NOT_DOWN"] / max(npw, 1), "lifted": fac["LIFTED_OFF"] / max(npw, 1),
            "hops": fac["HOPPED"] / max(npw, 1), "crash": fac["LEGS_BROKEN"] / max(npw, 1),
            "crash_all": fac["LEGS_BROKEN"] / max(n, 1),
            "tipped": fac["TIPPED_OVER"] / max(npw, 1), "abort": aborts / max(n, 1)}


def line(label, m):
    p, lo, hi = m["rate"]
    return ("{:40s} succ {:5.2f}% [{:4.2f}-{:4.2f}] (k={:d}) apo {:5.1f} vfire {:4.1f} pwr {:3.0f}% "
            "vz {:4.1f} vh {:4.2f} late {:3.0f}% lift {:3.0f}% hop {:3.0f}% crash {:3.0f}% tip {:3.0f}% "
            "abort {:4.1f}%").format(label, 100 * p, 100 * lo, 100 * hi, m["k"], m["ap_med"], m["fv_med"],
                                     100 * m["powered"], m["vz_med"], m["vh_med"], 100 * m["legs_late"],
                                     100 * m["lifted"], 100 * m["hops"], 100 * m["crash"], 100 * m["tipped"],
                                     100 * m["abort"])


def show(label, rows):
    m = metrics(rows)
    print(line(label, m), flush=True)
    c = metrics(rows, calm=True)
    print("        calm(<1 m/s): succ %.1f%% (k=%d/%d)" % (100 * c["rate"][0], c["k"], c["n"]), flush=True)
    return m


def paired(a, b):
    ka = {(r["seed"], r["i"]): r["summary"].get("landing_outcome", {}).get("success", False) for r in a}
    kb = {(r["seed"], r["i"]): r["summary"].get("landing_outcome", {}).get("success", False) for r in b}
    resc = sum(1 for k in ka if kb.get(k) and not ka[k])
    lost = sum(1 for k in ka if ka[k] and not kb.get(k))
    n = resc + lost
    p = 1.0
    if n:
        kk = min(resc, lost)
        p = min(1.0, 2 * sum(math.comb(n, i) for i in range(kk + 1)) / 2 ** n)
    return resc, lost, p


def legs_need(rows, swing=0.3):
    """Per powered landing: time from the nichrome turning on to ground contact, minus
    the leg swing = the latest the band may part for that flight's legs to be down."""
    out = []
    for r in ok(rows):
        s = r["summary"]
        if "NO_POWERED_LANDING" in s["landing_outcome"]["factors"] or s.get("legs_on_t") is None:
            continue
        out.append((contact_t(s) - s["legs_on_t"] - swing, s))
    return out
