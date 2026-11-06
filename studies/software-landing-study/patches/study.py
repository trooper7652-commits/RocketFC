"""Software-landing study driver: run a config from a workspace copy via
mc_batch.py (cached by label) and compute the pre-registered metrics."""
import collections
import json
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SCRATCH = os.path.dirname(HERE)
RES = os.path.join(HERE, "results")
PY = r"C:\Users\troop\AppData\Local\Programs\Python\Python312\python.exe"
os.makedirs(RES, exist_ok=True)


def ws_path(name):
    return os.path.join(SCRATCH, "sw", name, "tools", "sim") if name != "base" else \
        os.path.join(SCRATCH, "sw", "RocketFC", "tools", "sim")


def run(label, ws="base", seeds=(101, 102, 103), n=200, fw=None, plant=None,
        disp=None, force=False):
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
    r = subprocess.run(cmd, cwd=ws_path(ws), capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(label + ": " + r.stderr[-2000:])
    print("  ran", label, r.stdout.strip().splitlines()[-1])
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
    a = sorted(a)
    return a[min(len(a) - 1, int(p * len(a)))] if a else float("nan")


def metrics(rows, calm=False):
    rows = [r for r in rows if "error" not in r["summary"]]
    if calm:
        rows = [r for r in rows if abs(r["dispersions"]["wind_mps"]) < 1.0]
    n = len(rows)
    outs = [r["summary"]["landing_outcome"] for r in rows]
    k = sum(o["success"] for o in outs)
    powered = [r for r, o in zip(rows, outs) if "NO_POWERED_LANDING" not in o["factors"]]
    npw = len(powered)
    fac = collections.Counter(f for r in powered for f in r["summary"]["landing_outcome"]["factors"])
    cs = [r["summary"]["ground"]["contact"] for r in powered if r["summary"]["ground"]["contact"]]
    vz = [abs(c["vel"][2]) for c in cs]
    vh = [math.hypot(c["vel"][0], c["vel"][1]) for c in cs]
    aborts = collections.Counter(r["summary"].get("abort_state") or "?" for r in rows
                                 if r["summary"].get("abort_reason"))
    return {
        "n": n, "k": k, "rate": wilson(k, n), "powered": npw / max(n, 1),
        "vz_med": q(vz, .5), "vz_p90": q(vz, .9), "vh_med": q(vh, .5), "vh_p90": q(vh, .9),
        "legs_late": fac["LEGS_NOT_DOWN"] / max(npw, 1),
        "lifted": fac["LIFTED_OFF"] / max(npw, 1),
        "hops": fac["HOPPED"] / max(npw, 1),
        "crash": fac["LEGS_BROKEN"] / max(npw, 1),
        "tipped": fac["TIPPED_OVER"] / max(npw, 1),
        "aborts": dict(aborts), "abort_total": sum(aborts.values()) / max(n, 1),
        "vh_lt_085": sum(1 for x in vh if x < 0.85) / max(npw, 1),
        "vz_lt_3": sum(1 for x in vz if x < 3.0) / max(npw, 1),
    }


def line(label, m):
    p, lo, hi = m["rate"]
    return ("{:34s} succ {:5.1f}% [{:4.1f}-{:4.1f}] pwr {:4.0f}% vz {:4.1f}/{:4.1f} vh {:4.2f}/{:4.2f} "
            "vh<.85 {:4.1f}% vz<3 {:4.1f}% late {:3.0f}% lift {:3.0f}% hop {:3.0f}% crash {:3.0f}% "
            "aborts {:4.1f}%").format(
        label, 100 * p, 100 * lo, 100 * hi, 100 * m["powered"], m["vz_med"], m["vz_p90"],
        m["vh_med"], m["vh_p90"], 100 * m["vh_lt_085"], 100 * m["vz_lt_3"], 100 * m["legs_late"],
        100 * m["lifted"], 100 * m["hops"], 100 * m["crash"], 100 * m["abort_total"])


if __name__ == "__main__":
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
