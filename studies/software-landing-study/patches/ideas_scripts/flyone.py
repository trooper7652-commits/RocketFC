"""Fly one study flight (same draws as mc_batch) with recording, for diagnostics.
Usage (as a module): rows, summary, vc = fly(dll, seed, i, plant={...}, disp={...})"""
import copy
import math
import os
import sys

SCR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIM = os.path.join(SCR, "sw", "cand", "tools", "sim")
sys.path.insert(0, SIM)
os.chdir(SIM)

import core  # noqa: E402
import mc_batch  # noqa: E402
import run as run_mod  # noqa: E402
import sweep  # noqa: E402
import vehicle  # noqa: E402

X110 = os.path.join(SCR, "sw_study", "motors", "F15_x1.10.eng")
MEASURED = {"mass_descent_kg": 0.01, "ignition_delay_s": 0.05}


def make_vc(seed, i, plant=None, disp=None):
    truth = core.default_dll()
    base = vehicle.default_vehicle_config(truth)
    for k, v in (plant or {}).items():
        setattr(base, k, v)
    fields = list(sweep.DISPERSION_FIELDS)
    for a, w in (disp or {}).items():
        fields = [(n, (w if n == a else p)) for n, p in fields]
    jobs = mc_batch.build_jobs(base, fields, seed, i + 1, [])
    vc, d, s, _ = jobs[i]
    return vc, d, s


def fly(dll, seed, i, plant=None, disp=None, vc_edit=None):
    vc, d, s = make_vc(seed, i, plant, disp)
    if vc_edit:
        vc_edit(vc)
    r = run_mod.simulate(vc, mode="land", dll_path=dll, seed=s, record=True)
    return r["rows"], r["summary"], vc, d


def qrot(q, v):
    w, x, y, z = q
    vx, vy, vz = v
    # v' = q v q*
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def tilt_of(q):
    zb = qrot(q, (0.0, 0.0, 1.0))
    return math.degrees(math.acos(max(-1.0, min(1.0, zb[2]))))


def dll_for(fw):
    """Flight DLL of the current candidate copy with these config overrides."""
    return core.build_with_overrides(fw) if fw else core.default_dll()


IMU_BEST = {"DRIFT_GAIN": "2.0f", "TILT_PREDICT_INTERLOCK": "true"}
GPS_BEST = dict(IMU_BEST, GPS_VEL_GAIN="0.3f", BURN_TABLE_MARGIN_M="1.0f")
