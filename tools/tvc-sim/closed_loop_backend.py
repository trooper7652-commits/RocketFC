"""Bridges tvc_pid_tuner.py's optimizer to RocketFC's closed-loop simulator
(../RocketFC/tools/sim), so --closed-loop tunes the REAL C++ TvcControl
over a full 6-DOF trajectory instead of this file's own rotation-only Python
PID stand-in.

Path convention matches ../SensorServoBenchTest/sync_core.py's: RocketFC is
assumed to be a sibling directory, override with --rocketfc if not.

Cost is scored the same way tvc_pid_tuner.py's own _score_run() does (same
weights, same metrics: ITAE, steady-state error, residual oscillation,
gimbal thrashing, control effort, saturation), just computed over the
closed-loop sim's rows for the phase being tuned, so "lower is better" means
the same thing whether you ran --closed-loop or not. Overshoot isn't
included here: the closed-loop sim doesn't start from a deliberately offset
initial tilt the way the rotation-only simulate() does, so "excursion past
zero" isn't a meaningful signal in the same way.

Every candidate gain triple needs a fresh DLL build (~0.3-0.5s, uncached --
gains differ every call) plus a full flight simulation (~1-2s), so a
closed-loop optimization run is CPU-bound: expect single-digit minutes even
with reduced maxiter/popsize (see tvc_pid_tuner.py's --closed-loop default
override), not the seconds the rotation-only path takes.
"""
import math
import os
import sys

_BRIDGED = {}


def _bridge(rocketfc_root):
    root = os.path.abspath(rocketfc_root)
    if root in _BRIDGED:
        return _BRIDGED[root]
    sim_dir = os.path.join(root, "tools", "sim")
    if not os.path.isdir(sim_dir):
        raise FileNotFoundError(
            "RocketFC/tools/sim not found at {} -- pass --rocketfc PATH".format(sim_dir))
    if sim_dir not in sys.path:
        sys.path.insert(0, sim_dir)
    import core as rfc_core
    import run as rfc_run
    import vehicle as rfc_vehicle
    mods = (rfc_core, rfc_run, rfc_vehicle)
    _BRIDGED[root] = mods
    return mods


def make_vehicle_config(cfg, rocketfc_root):
    """Build the closed-loop sim's VehicleConfig for this tuning run, and
    sync `cfg`'s geometry fields FROM it (not the other way around).

    vehicle.py's own defaults (inertia_kgm2, gimbal arm) are the more
    grounded estimate for THIS airframe -- derived from RocketFC's actual
    CAD-informed geometry (see vehicle.py's docstrings) -- while this file's
    own Config.I_kgm2/L_arm_m are generic, standalone placeholders that have
    never been reconciled with RocketFC specifically (this file predates the
    closed-loop sim and was written to run on its own with an arbitrary
    example airframe). Overwriting vehicle.py's estimate with this file's
    unedited defaults produced a genuinely UNSTABLE closed-loop flight in
    testing (0.03 kg*m^2 vs vehicle.py's ~0.13 -- a ~4x lower inertia for the
    same torque) purely from that mismatch, not from anything wrong with the
    gains being tested. So: vehicle.py wins, and `cfg` is updated to match
    for the printed "I=... L=..." summary in main() to stay honest about
    what was actually simulated.

    Only the fields that don't already have a HIGHER-fidelity source in
    vehicle.py are pushed the other way (servo dynamics happen to share the
    same defaults already; disturbance torque and the motor file are
    tuning-run choices with no vehicle.py equivalent).
    """
    rfc_core, _rfc_run, rfc_vehicle = _bridge(rocketfc_root)
    vcfg = rfc_vehicle.default_vehicle_config(rfc_core.default_dll())
    cfg.I_kgm2 = vcfg.inertia_kgm2
    cfg.L_arm_m = vcfg.gimbal_pivot_from_nose_m - vcfg.cg_from_nose_m
    vcfg.servo_slew_dps = cfg.servo_slew_dps
    vcfg.servo_lag_tau_s = cfg.servo_lag_tau_s
    vcfg.disturbance_torque_nm = cfg.disturbance_torque_Nm
    vcfg.disturbance_ref_thrust_n = cfg.disturbance_ref_thrust_N
    if cfg.eng_file:
        vcfg.ascent_eng_path = cfg.eng_file
        vcfg.landing_eng_path = cfg.eng_file
    return vcfg


def _score_rows(rows, cfg):
    """Same formula as tvc_pid_tuner.py's _score_run(), computed from a
    closed-loop run's row dicts (radians throughout, matching that
    function's units)."""
    if not rows:
        return cfg.w_unstable
    t0 = rows[0]["t"]
    dt = (rows[1]["t"] - rows[0]["t"]) if len(rows) > 1 else 0.002
    ts = [r["t"] - t0 for r in rows]
    tilt_rad = [math.radians(r["tilt_deg"]) for r in rows]
    gx_cmd = [r["gimbal_x_cmd"] for r in rows]
    gx_act = [r["gimbal_x_act"] for r in rows]
    n = len(ts)
    abs_th = [abs(x) for x in tilt_rad]

    itae = sum(t * a for t, a in zip(ts, abs_th)) * dt
    tail = max(1, int(0.2 * n))
    sse = sum(abs_th[-tail:]) / tail
    mean_tail = sum(tilt_rad[-tail:]) / tail
    osc = (sum((x - mean_tail) ** 2 for x in tilt_rad[-tail:]) / tail) ** 0.5
    thrash = sum(abs(gx_cmd[i] - gx_cmd[i - 1]) for i in range(1, n))
    ctrl_eff = sum(a * a for a in gx_act) * dt
    sat_frac = sum(1 for x in gx_cmd if abs(x) >= cfg.delta_max_rad * 0.999) / n

    return (cfg.w_itae * itae + cfg.w_sse * sse + cfg.w_osc * osc +
            cfg.w_thrash * thrash + cfg.w_ctrl * ctrl_eff + cfg.w_sat * sat_frac)


def cost_closed_loop(gains, cfg, rocketfc_root="../RocketFC", phase="boost",
                     mode=None):
    """Drop-in replacement for tvc_pid_tuner.py's cost(gains, cfg): same
    signature and "lower is better" contract, but gains are patched into the
    REAL src/config.h PidGains struct and scored over a REAL closed-loop
    flight run through the actual C++ TvcControl."""
    rfc_core, rfc_run, _rfc_vehicle = _bridge(rocketfc_root)
    kp, ki, kd = gains
    group = "GAINS_BOOST" if phase == "boost" else "GAINS_LANDING"
    overrides = {group: {"kp": "{:.6f}f".format(kp), "ki": "{:.6f}f".format(ki),
                         "kd": "{:.6f}f".format(kd)}}
    try:
        dll = rfc_core.build_with_overrides(overrides)
    except Exception:
        return cfg.w_unstable

    vcfg = make_vehicle_config(cfg, rocketfc_root)
    sim_mode = mode or ("land" if phase == "landing" else "chute")
    try:
        result = rfc_run.simulate(vcfg, mode=sim_mode, dll_path=dll,
                                  seed=cfg.tune_seed,
                                  max_t=cfg.tune_duration_s + 20.0)
    except Exception:
        return cfg.w_unstable

    if result["summary"]["abort_reason"] == "TILT":
        return cfg.w_unstable

    target_state = "BOOST" if phase == "boost" else "LANDING_BURN"
    phase_rows = [r for r in result["rows"] if r["state"] == target_state]
    return _score_rows(phase_rows, cfg)


def run_and_summarize(gains, cfg, rocketfc_root="../RocketFC", phase="boost",
                      mode=None):
    """Run one closed-loop flight with the given gains and return
    (result, metrics) where metrics has the SAME KEYS as
    tvc_pid_tuner.py's compute_metrics() (overshoot_pct, settling_s, sse_deg,
    peak_delta_deg, saturated, diverged), so main()'s print/report block
    works unchanged regardless of which path produced it.

    "Overshoot" here means something slightly different than in the
    rotation-only path: rather than the excursion past a deliberately offset
    initial tilt (this sim starts upright), it's the peak |tilt| reached
    during the phase, relative to the steady-state tilt at the end of it --
    still "how far did it swing before settling", just without an initial
    offset to measure "past zero" against.
    """
    rfc_core, rfc_run, _rfc_vehicle = _bridge(rocketfc_root)
    kp, ki, kd = gains
    group = "GAINS_BOOST" if phase == "boost" else "GAINS_LANDING"
    overrides = {group: {"kp": "{:.6f}f".format(kp), "ki": "{:.6f}f".format(ki),
                         "kd": "{:.6f}f".format(kd)}}
    dll = rfc_core.build_with_overrides(overrides)
    vcfg = make_vehicle_config(cfg, rocketfc_root)
    sim_mode = mode or ("land" if phase == "landing" else "chute")
    result = rfc_run.simulate(vcfg, mode=sim_mode, dll_path=dll,
                              seed=cfg.eval_seed,
                              max_t=cfg.tune_duration_s + 20.0)

    target_state = "BOOST" if phase == "boost" else "LANDING_BURN"
    rows = [r for r in result["rows"] if r["state"] == target_state]
    diverged = result["summary"]["abort_reason"] == "TILT" or not rows

    if diverged:
        metrics = {"overshoot_pct": float("nan"), "settling_s": float("nan"),
                  "sse_deg": float("nan"), "peak_delta_deg": float("nan"),
                  "saturated": False, "diverged": True}
        return result, metrics

    deg = 180.0 / math.pi
    tilt_deg = [r["tilt_deg"] for r in rows]
    ts = [r["t"] - rows[0]["t"] for r in rows]
    gx_cmd_deg = [math.degrees(r["gimbal_x_cmd"]) for r in rows]
    gx_act_deg = [math.degrees(r["gimbal_x_act"]) for r in rows]
    n = len(rows)
    tail = max(1, int(0.2 * n))
    sse_deg = sum(tilt_deg[-tail:]) / tail
    peak_tilt = max(tilt_deg)
    overshoot_pct = (max(0.0, peak_tilt - abs(sse_deg)) / max(abs(sse_deg), 1e-6) * 100.0)

    band = max(0.02 * peak_tilt, 0.5)
    outside = [i for i, x in enumerate(tilt_deg) if abs(x) > band]
    settling_s = ts[outside[-1]] if outside else 0.0

    snap = rfc_core.FlightCore.config_snapshot(dll)
    delta_max_deg = math.degrees(snap["gimbalMaxRad"])
    saturated = any(abs(x) >= delta_max_deg * 0.999 for x in gx_cmd_deg)

    metrics = {
        "overshoot_pct": overshoot_pct, "settling_s": settling_s,
        "sse_deg": sse_deg, "peak_delta_deg": max(abs(x) for x in gx_act_deg),
        "saturated": saturated, "diverged": False,
    }
    return result, metrics
