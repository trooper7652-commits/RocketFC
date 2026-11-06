"""New-ideas study driver: same metrics as sw_study/study.py, results kept in
sw_ideas/results, flights from the sw/ideas workspace."""
import collections
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SCR = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(SCR, "sw_study"))
import study  # noqa: E402

study.RES = os.path.join(HERE, "results")
os.makedirs(study.RES, exist_ok=True)
X110 = os.path.join(SCR, "sw_study", "motors", "F15_x1.10.eng")
MEAS = {"mass_descent_kg": 0.01, "ignition_delay_s": 0.05}
IMU = {"DRIFT_GAIN": "2.0f", "TILT_PREDICT_INTERLOCK": "true", "BURN_TABLE_MARGIN_M": "1.0f"}
GPS = dict(IMU, GPS_VEL_GAIN="0.3f")
DEV = (101, 102, 103)


def run(label, fw, plant=None, disp=MEAS, seeds=DEV, n=200, ws="ideas", force=False, ext=None):
    if "GPS_VEL_GAIN" in fw:
        plant = dict({"gps_hz": 10.0}, **(plant or {}))
    p = {"ascent_eng_path": X110}
    p.update(plant or {})
    if not ext:
        return study.run(label, ws=ws, seeds=seeds, n=n, fw=fw, plant=p, disp=disp, force=force)
    import subprocess
    out = os.path.join(study.RES, label + ".json")
    if os.path.exists(out) and not force:
        return json.load(open(out))["rows"]
    cmd = [study.PY, "mc_batch.py", "--seeds", *map(str, seeds), "--n", str(n), "--out", out,
           "--fw-overrides", json.dumps(fw),
           "--plant", *["{}={}".format(k, json.dumps(v)) for k, v in p.items()],
           "--disp", *["{}={}".format(k, v) for k, v in disp.items()],
           "--ext", *ext]
    r = subprocess.run(cmd, cwd=study.ws_path(ws), capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(label + ": " + r.stderr[-2000:])
    print("  ran", label, r.stdout.strip().splitlines()[-1])
    return json.load(open(out))["rows"]


def q(a, p):
    a = sorted(a)
    return a[min(len(a) - 1, int(p * len(a)))] if a else float("nan")


def extra(rows):
    rows = [r for r in rows if "error" not in r["summary"]]
    bo = [r["summary"].get("bo_hs_air") for r in rows if r["summary"].get("bo_hs_air") is not None]
    ap = [r["summary"].get("ap_tilt_true") for r in rows if r["summary"].get("ap_tilt_true") is not None]
    pw = [r for r in rows if r["summary"].get("fire_t") is not None]
    ft = [r["summary"]["fire_tilt_deg"] for r in pw]
    return ("    burnout air-rel hs med %.2f p90 %.2f | apogee tilt med %.1f p90 %.1f | fire tilt med %.1f"
            % (q(bo, .5), q(bo, .9), q(ap, .5), q(ap, .9), q(ft, .5)))


def show(label, rows):
    print(study.line(label, study.metrics(rows)))
    m = study.metrics(rows, calm=True)
    print("      calm: succ %.1f%% (k=%d/%d)" % (100 * m["rate"][0], m["k"], m["n"]))
    print(extra(rows))
    # success by wind band
    rr = [r for r in rows if "error" not in r["summary"]]
    s = []
    for lo, hi in [(0, 1), (1, 2), (2, 3), (3, 4.01)]:
        sub = [r for r in rr if lo <= abs(r["dispersions"]["wind_mps"]) < hi]
        k = sum(r["summary"]["landing_outcome"]["success"] for r in sub)
        a = sum(1 for r in sub if r["summary"].get("abort_reason"))
        s.append("w%d-%d: succ %d/%d abort %.0f%%" % (lo, hi, k, len(sub), 100 * a / max(1, len(sub))))
    print("      " + " | ".join(s))


def paired(a, b):
    ka = {(r["seed"], r["i"]): r["summary"].get("landing_outcome", {}).get("success", False) for r in a}
    kb = {(r["seed"], r["i"]): r["summary"].get("landing_outcome", {}).get("success", False) for r in b}
    resc = sum(1 for k in ka if kb.get(k) and not ka[k])
    lost = sum(1 for k in ka if ka[k] and not kb.get(k))
    n = resc + lost
    p = 1.0
    if n:
        from math import comb
        kk = min(resc, lost)
        p = min(1.0, 2 * sum(comb(n, i) for i in range(kk + 1)) / 2 ** n)
    return resc, lost, p
