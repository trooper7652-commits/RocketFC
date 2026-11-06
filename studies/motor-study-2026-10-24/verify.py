"""Step 7: the ported RocketFC on the holdout seeds must reproduce the study copy's
S3 + margin numbers exactly and beat the previous RocketFC (today) on the same flights."""
import json, os, subprocess, sys
sys.path.insert(0, ".")
import drv
M = json.load(open("motors/motors.json"))
RFC_SIM = r"C:\Users\troop\OneDrive\Documents\Inventor\SpaceX Rocket\RocketFC\tools\sim"
NEW = {"bo_hs_air", "bo_wind_est", "bo_wind_true", "bo_hs", "bo_tilt_true", "bo_rate_dps", "ap_tilt_true"}


def run_at(label, cwd, plant=None):
    out = os.path.join(drv.RES, label + ".json")
    if not os.path.exists(out):
        cmd = [drv.PY, "mc_batch.py", "--seeds", *map(str, drv.HOLD), "--n", "200", "--out", out,
               "--disp", *["%s=%s" % kv for kv in drv.MEAS.items()]]
        if plant:
            cmd += ["--plant", *["%s=%s" % (k, json.dumps(v)) for k, v in plant.items()]]
        r = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True)
        if r.returncode:
            raise RuntimeError(r.stderr[-2000:])
        print("  ran", label, r.stdout.strip().splitlines()[-1], flush=True)
    return json.load(open(out))["rows"]


for tag, cwd, plant in (("F15", RFC_SIM, None),
                        ("F10", drv.ws_sim("rfc_F10"), {"ascent_eng_path": M["F10"]["eng"]})):
    new = run_at("verify_rfc_%s" % tag, cwd, plant)
    study = json.load(open(os.path.join(drv.RES, "ho_%s_S3M.json" % tag)))["rows"]
    prev = json.load(open(os.path.join(drv.RES, "ho_%s_today.json" % tag)))["rows"]
    diff = sum(1 for a, b in zip(study, new)
               if {k: v for k, v in a["summary"].items() if k not in NEW} != b["summary"])
    print("== %s ascent: ported RocketFC vs study copy (S3 + margin): %d of %d flights differ" % (tag, diff, len(new)))
    drv.show("RocketFC before (today)  %s" % tag, prev)
    drv.show("RocketFC after (ported)  %s" % tag, new)
    a, b = drv.metrics(prev), drv.metrics(new)
    print("        paired: rescued %d lost %d p=%.3g | aborts %+.1f pt, crashes %+.1f pt" % (
        drv.paired(prev, new) + (100 * (b["abort"] - a["abort"]), 100 * (b["crash_all"] - a["crash_all"]))), flush=True)
