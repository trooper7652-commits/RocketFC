"""Study helpers: run tools/sim/mc_batch.py in a scratch workspace (cached by
label) and compute landing / apogee / chute metrics, paired against a base."""
import json
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SCR = r"C:\Users\troop\AppData\Local\Temp\claude\3e9e20f0e564c4e3\2e110106-3a4f-48f3-9a7d-a66c508094e2\scratchpad\mc"
WS = os.path.join(SCR, "ws")
RES = os.path.join(SCR, "results")
PY = r"C:\Users\troop\AppData\Local\Programs\Python\Python312\python.exe"
DEV = (101, 102, 103)
HOLD = tuple(range(701, 711))
MEAS = {"mass_descent_kg": 0.01, "ignition_delay_s": 0.05}
APOGEE_LIMIT = 30.48
os.makedirs(RES, exist_ok=True)
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")


def run(label, ws, seeds=DEV, n=200, disp=MEAS, fw=None, plant=None, ext=None, wind=None,
        mode="land", plant_list=None):
    out = os.path.join(RES, label + ".json")
    if not os.path.exists(out):
        cmd = [PY, "mc_batch.py", "--seeds", *map(str, seeds), "--n", str(n), "--out", out,
               "--workers", "19", "--mode", mode]
        if disp:
            cmd += ["--disp", *["%s=%s" % kv for kv in disp.items()]]
        if fw:
            cmd += ["--fw-overrides", json.dumps(fw)]
        if plant:
            cmd += ["--plant", *["%s=%s" % (k, json.dumps(v)) for k, v in plant.items()]]
        if ext:
            cmd += ["--ext", *ext]
        if wind is not None:
            cmd += ["--wind", str(wind)]
        if plant_list is not None:
            cmd += ["--plant-list", json.dumps(plant_list)]
        r = subprocess.run(cmd, cwd=os.path.join(WS, ws, "tools", "sim"), capture_output=True, text=True)
        if r.returncode:
            raise RuntimeError(label + ": " + r.stderr[-3000:])
        print("  ran", label, "|", r.stdout.strip().splitlines()[-1].split(": ", 1)[-1], flush=True)
    rows = json.load(open(out))["rows"]
    bad = [r for r in rows if "error" in r["summary"]]
    if bad:
        print("  !! %s: %d errors, e.g. %s" % (label, len(bad), bad[0]["summary"]["error"][:200]))
    return [r for r in rows if "error" not in r["summary"]]


def q(a, p):
    a = sorted(x for x in a if x is not None)
    return a[min(len(a) - 1, int(p * len(a)))] if a else float("nan")


def metrics(rows):
    n = len(rows)
    outs = [r["summary"]["landing_outcome"] for r in rows]
    vz = []
    for r in rows:
        c = (r["summary"].get("ground") or {}).get("contact")
        if c and "NO_POWERED_LANDING" not in r["summary"]["landing_outcome"]["factors"]:
            vz.append(abs(c["vel"][2]))
    pw = [o for o in outs if "NO_POWERED_LANDING" not in o["factors"]]
    fac = lambda f: sum(1 for o in pw if f in o["factors"]) / max(len(pw), 1)
    ap = [r["summary"]["apogee_h"] for r in rows]
    return {"n": n, "k": sum(o["success"] for o in outs), "vz_med": q(vz, .5), "vz_p90": q(vz, .9),
            "over3": sum(1 for v in vz if v > 3.0) / max(len(vz), 1),
            "lifted": fac("LIFTED_OFF"), "legs_late": fac("LEGS_NOT_DOWN"), "hop": fac("HOPPED"),
            "abort": (n - len(pw)) / max(n, 1),
            "ap_med": q(ap, .5), "ap_p99": q(ap, .99), "ap_max": max(ap) if ap else float("nan")}


def paired(base, rows):
    a = {(r["seed"], r["i"]): r["summary"]["landing_outcome"]["success"] for r in base}
    b = {(r["seed"], r["i"]): r["summary"]["landing_outcome"]["success"] for r in rows}
    resc = sum(1 for k in a if b.get(k) and not a[k])
    lost = sum(1 for k in a if a[k] and not b.get(k))
    m = resc + lost
    p = min(1.0, 2 * sum(math.comb(m, i) for i in range(min(resc, lost) + 1)) / 2 ** m) if m else 1.0
    return resc, lost, p


def wilson(k, n, z=1.96):
    if n == 0:
        return (float("nan"), float("nan"))
    p = k / n
    d = 1 + z * z / n
    c = p + z * z / (2 * n)
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return (100 * (c - h) / d, 100 * (c + h) / d)


def chute_metrics(rows):
    """Chute landings: vertical / horizontal contact speed, deploy height,
    canopy fill at contact, peak load after deploy, unconfirmed canopies."""
    vz, vh, dh, frac, pk, unc, opened = [], [], [], [], [], 0, 0
    for r in rows:
        s = r["summary"]
        c = (s.get("ground") or {}).get("contact")
        if not c:
            continue
        vz.append(abs(c["vel"][2]))
        vh.append(math.hypot(c["vel"][0], c["vel"][1]))
        dep = s.get("chute_deploy_t")
        before = dep is not None and dep < c["t"]
        opened += 1 if before else 0
        frac.append((s.get("chute_frac_td") or 0.0) if before else 0.0)
        if before:
            dh.append(s["chute_deploy_h"])
            pk.append(s.get("chute_peak_g") or 0.0)
            unc += 1 if s.get("chute_unconfirmed") else 0
    n = len(vz)
    return {"n": n, "of": len(rows), "opened": opened / max(n, 1),
            "vz_med": q(vz, .5), "vz_p90": q(vz, .9), "vz_p99": q(vz, .99),
            "le3": sum(1 for v in vz if v <= 3.0) / max(n, 1),
            "vh_med": q(vh, .5), "vh_p90": q(vh, .9),
            "dh_med": q(dh, .5), "dh_p10": q(dh, .1),
            "frac_med": q(frac, .5), "full": sum(1 for f in frac if f >= 0.999) / max(n, 1),
            "peak_g_med": q(pk, .5), "peak_g_p99": q(pk, .99), "unconfirmed": unc / max(opened, 1)}
