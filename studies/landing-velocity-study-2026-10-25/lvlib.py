"""Landing-velocity study helpers: run mc_batch.py in a workspace (cached by label)
and compute contact-speed metrics, paired against a base run."""
import json
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(HERE, "results")
PY = r"C:\Users\troop\AppData\Local\Programs\Python\Python312\python.exe"
DEV = (101, 102, 103)
HOLD = tuple(range(701, 711))
MEAS = {"mass_descent_kg": 0.01, "ignition_delay_s": 0.05}
os.makedirs(RES, exist_ok=True)
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")


def ws_sim(name):
    return os.path.join(HERE, "ws", name, "tools", "sim")


def run(label, ws="base", seeds=DEV, n=200, disp=MEAS, fw=None, plant=None, ext=None, wind=None):
    out = os.path.join(RES, label + ".json")
    if not os.path.exists(out):
        cmd = [PY, "mc_batch.py", "--seeds", *map(str, seeds), "--n", str(n), "--out", out,
               "--workers", "19"]
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
        r = subprocess.run(cmd, cwd=ws_sim(ws), capture_output=True, text=True)
        if r.returncode:
            raise RuntimeError(label + ": " + r.stderr[-3000:])
        print("  ran", label, "|", r.stdout.strip().splitlines()[-1].split(": ", 1)[-1], flush=True)
    return [r for r in json.load(open(out))["rows"] if "error" not in r["summary"]]


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
    lifted = sum(1 for o in pw if "LIFTED_OFF" in o["factors"])
    return {"n": n, "k": sum(o["success"] for o in outs), "vz_med": q(vz, .5), "vz_p90": q(vz, .9),
            "over3": sum(1 for v in vz if v > 3.0) / max(len(vz), 1),
            "lifted": lifted / max(len(pw), 1), "abort": (n - len(pw)) / max(n, 1)}


def paired(base, rows):
    a = {(r["seed"], r["i"]): r["summary"]["landing_outcome"]["success"] for r in base}
    b = {(r["seed"], r["i"]): r["summary"]["landing_outcome"]["success"] for r in rows}
    resc = sum(1 for k in a if b.get(k) and not a[k])
    lost = sum(1 for k in a if a[k] and not b.get(k))
    m = resc + lost
    p = min(1.0, 2 * sum(math.comb(m, i) for i in range(min(resc, lost) + 1)) / 2 ** m) if m else 1.0
    return resc, lost, p


def table(rows_by_label, base_label):
    base = rows_by_label[base_label]
    print("| Run | Success | Contact vz median / p90 (m/s) | Over 3 m/s | Lifted off | Aborts | vs base |")
    print("|---|---|---|---|---|---|---|")
    for lab, rows in rows_by_label.items():
        m = metrics(rows)
        pr = "" if lab == base_label else "%d rescued / %d lost, p=%.3g" % paired(base, rows)
        print("| %s | %.1f %% (%d/%d) | %.2f / %.2f | %.0f %% | %.0f %% | %.1f %% | %s |" % (
            lab, 100.0 * m["k"] / m["n"], m["k"], m["n"], m["vz_med"], m["vz_p90"], 100 * m["over3"],
            100 * m["lifted"], 100 * m["abort"], pr), flush=True)
