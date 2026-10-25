#!/usr/bin/env python3
"""Headless closed-loop flight simulation: wires plant.py (truth) + sensors.py
(noisy readings) + servo.py (actuator dynamics) + core.py (the REAL flight
computer, via ctypes) into one step loop, mirroring exactly how
RocketFC.ino/tools/replay drive FlightCore, but with the attitude and
trajectory now genuinely CLOSED LOOP instead of prescribed.

Two things this does NOT attempt, both deliberate scope decisions (see
sensors.py's module docstring and the note below for why):

  - Sensor readings are generated directly in the flight-core's OWN body
    frame (the frame flight_core.h already assumes pre-mapped inputs are in)
    rather than modeling a physically separate sensor-mounting frame and
    applying cfg::IMU_R_SB in this loop. This matches tools/synth_flight.py
    and tools/replay's own precedent -- IMU_R_SB correctness is validated by
    the real-hardware SensorServoBenchTest rig, not by this simulator.
  - Servo SIGN errors, unlike IMU_R_SB, ARE modeled end-to-end below (see
    "servo sign round-trip"): a deliberately wrong SERVO_A_SIGN/SERVO_B_SIGN
    in a swept build genuinely inverts the actuator's physical response in
    the closed loop and destabilizes it, because the sign is the one
    uncertain quantity config.h itself calls out (center/scale are assumed
    correctly calibrated).

Run directly for a quick nominal-flight smoke test:
    python tools/sim/run.py [chute|land]
"""
import math
import random
import sys

import core
import landing_outcome
import plant as plant_mod
import sensors as sensors_mod
import servo as servo_mod


def _ramp(t, t0, duration):
    """0 before t0, rising linearly to 1 over `duration` seconds."""
    return min(1.0, max(0.0, (t - t0) / max(duration, 1e-6)))


def simulate(vcfg, mode="land", dll_path=None, seed=42, arm_t=2.6,
            countdown_s=0.5, max_t=60.0, dt=0.002, baro_hz=100.0,
            record=True, dud_landing_igniter=False, record_inputs=False):
    """Run one closed-loop flight. Returns a dict:
        rows: list of per-tick dicts (only if record=True) -- truth, sensor,
              estimator and control values, one row per FAST_HZ tick, in the
              same spirit as src/hw/logger.h's columns (see logfile.py for the
              on-disk format these get written to).
        events: list of {t, code, value} FlightEvent occurrences.
        summary: apogee/touchdown/tilt/abort/KF-error summary.

    dud_landing_igniter: if True, the FIRE_LANDING command is issued to the
        flight computer as normal but the plant never actually ignites the
        landing motor -- exercises the same DUD_IGNITER abort path as
        tools/replay/cases/dud_igniter.csv (real e-match failure to light).
    record_inputs: if True, the result also carries "inputs" -- every
        CoreInput fed to the flight core from the arm tick onward, as
        (ms, dt, accel, gyro_bias_corrected, baro_new, baro_alt) tuples -- and
        "arm_info" (the pad-level accel, baro noise variance and arm ms the
        core was armed with). tools/hil plays these back on the real Teensy.
        Read-only: the flight itself is unchanged.
    """
    dll_path = dll_path or core.default_dll()
    snap = core.FlightCore.config_snapshot(dll_path)
    baro_every = max(1, round((1.0 / baro_hz) / dt))

    pl = plant_mod.Plant(vcfg)
    if pl.ground is not None:
        # Field slope for this flight: its own RNG stream, so the sensor noise
        # and every other draw stay exactly as they were.
        _rng = random.Random(seed * 7907 + 3)
        pl.ground.set_slope(_rng.uniform(0.0, vcfg.ground_slope_max_deg),
                            _rng.uniform(0.0, 360.0))
    sm = sensors_mod.SensorModel(vcfg, seed=seed)
    sv = servo_mod.GimbalServos(math.radians(vcfg.servo_slew_dps),
                                vcfg.servo_lag_tau_s, snap["gimbalMaxRad"], dt)

    fc = core.FlightCore(dll_path)
    fc.begin()
    fc.set_mode(core.FlightMode.FULL_LANDING if mode == "land"
               else core.FlightMode.CHUTE_TEST)

    launch_t = arm_t + countdown_s
    cal_end_t = arm_t - 0.2

    gbx = gby = gbz = 0.0
    aax = aay = aaz = 0.0
    n_cal = 0
    p_mean = 0.0
    p_m2 = 0.0
    n_p = 0

    armed = False
    launched = False
    p0_ref = 101325.0
    last_baro_alt = 0.0
    last_baro_pa = 0.0

    rows = [] if record else None
    inputs = [] if record_inputs else None
    arm_info = None
    events = []
    max_tilt = 0.0
    max_kf_err = 0.0
    touchdown_speed = None
    abort_reason = None
    abort_t = None
    chute_releases = 0
    chute_detected_t = None
    chute_unconfirmed = False
    legs_on_t = None
    legs_off_t = None
    # ---- landing diagnostics (read-only; never fed back into the flight) ----
    fire_info = None
    prev_tilt = None
    prev_state = None
    abort_state = None
    hop = False
    hop_stop_h = None
    hop_apex_h = None
    touchdown_hspeed = None
    lag_shv = lag_svv = lag_sva = lag_saa = 0.0
    lag_n = 0
    lag_eh_sum = lag_ev_sum = 0.0
    prev_v_true = None

    t = 0.0
    i = 0
    try:
        while t < max_t:
            s = pl.state
            _, thrust_now = pl.mass_and_thrust(t)
            (ax, ay, az), (gx, gy, gz) = sm.imu(s.specific_force_body, s.omega,
                                                dt, thrust_now)

            baro_new_raw = (i % baro_every == 0)
            p_pa = 0.0
            if baro_new_raw:
                if getattr(vcfg, "baro_q_coeff", 0.0) != 0.0:
                    _w = pl._wind(t)
                    _rel = s.vel - _w
                    p_pa = sm.baro(s.pos.z, baro_every * dt, airspeed=_rel.norm())
                else:
                    p_pa = sm.baro(s.pos.z, baro_every * dt)

            if t < cal_end_t:
                gbx += gx; gby += gy; gbz += gz
                aax += ax; aay += ay; aaz += az
                n_cal += 1
                if baro_new_raw:
                    n_p += 1
                    d = p_pa - p_mean
                    p_mean += d / n_p
                    p_m2 += d * (p_pa - p_mean)

            if not armed and t >= arm_t and n_cal > 0:
                gbx /= n_cal; gby /= n_cal; gbz /= n_cal
                aax /= n_cal; aay /= n_cal; aaz /= n_cal
                p0_ref = p_mean if p_mean > 0 else 101325.0
                m_per_pa = 8434.6 / p0_ref
                alt_var = ((p_m2 / (n_p - 1)) * m_per_pa * m_per_pa
                          if n_p > 1 else 0.25)
                fc.pad_level_init((aax, aay, aaz))
                fc.set_baro_noise_var(max(alt_var, 0.0025))
                fc.zero_altitude()
                fc.request_arm(int(t * 1000.0 + 0.5))
                armed = True
                if record_inputs:
                    arm_info = {"pad_accel": (aax, aay, aaz),
                                "baro_var": max(alt_var, 0.0025),
                                "arm_ms": int(t * 1000.0 + 0.5),
                                "launch_ms": int(launch_t * 1000.0 + 0.5)}

            if armed and not launched and t >= launch_t:
                pl.ignite_ascent(t)
                launched = True

            baro_new = armed and baro_new_raw
            baro_alt = last_baro_alt
            if baro_new:
                baro_alt = 44330.0 * (1.0 - (p_pa / p0_ref) ** 0.190295)
                last_baro_alt = baro_alt
                last_baro_pa = p_pa

            gyro_corrected = (gx - gbx, gy - gby, gz - gbz)
            ms = int(t * 1000.0 + 0.5)
            out = fc.step(ms, dt, (ax, ay, az), gyro_corrected,
                         baro_new=baro_new, baro_alt=baro_alt)
            if inputs is not None and armed:
                inputs.append((ms, dt, (ax, ay, az), gyro_corrected,
                               baro_new, baro_alt))

            for ev in fc.pop_events():
                ev["t"] = t
                events.append(ev)
                if ev["code"] == core.FlightEvent.FIRE_LANDING and fire_info is None:
                    fire_info = {
                        "fire_t": t, "fire_h_true": s.pos.z,
                        "fire_v_true": s.vel.z,
                        "fire_kf_alt": out["kf_alt"],
                        "fire_kf_vel": out["kf_vel"],
                        "fire_tilt_deg": out["tilt_deg"],
                        "fire_tilt_prev_deg": prev_tilt,
                        "fire_hspeed_true": math.hypot(s.vel.x, s.vel.y),
                    }
                if ev["code"] == core.FlightEvent.ABORT_DET:
                    abort_state = prev_state
                if ev["code"] == core.FlightEvent.FIRE_LANDING and not dud_landing_igniter:
                    # e-match ignition delay: burn_table.h's ignition altitudes
                    # already account for this gap between the fire COMMAND and
                    # actual thrust onset (see tools/burn_table.py / IGN_DELAY
                    # in tools/synth_flight.py) -- omitting it here would fire
                    # the motor too early relative to what the table assumed.
                    pl.ignite_landing(t + vcfg.ignition_delay_s)
                elif ev["code"] == core.FlightEvent.CHUTE_RELEASE:
                    # Spring latch: the first chute_stuck_releases releases
                    # stick, so the canopy only appears once the flight
                    # computer's re-cycles get past them.
                    chute_releases += 1
                    if chute_releases > vcfg.chute_stuck_releases:
                        pl.deploy_chute(t)
                elif ev["code"] == core.FlightEvent.CHUTE_DETECTED:
                    chute_detected_t = t
                elif ev["code"] == core.FlightEvent.CHUTE_UNCONFIRMED:
                    chute_unconfirmed = True
                elif ev["code"] == core.FlightEvent.LEGS_BURN_ON:
                    legs_on_t = t
                elif ev["code"] == core.FlightEvent.LEGS_BURN_OFF:
                    legs_off_t = t
                elif ev["code"] == core.FlightEvent.ABORT_DET:
                    abort_reason = core.AbortReason(int(ev["value"]))
                    abort_t = t
                elif ev["code"] == core.FlightEvent.TOUCHDOWN_DET:
                    pass  # plant's own touchdown_t is authoritative for the summary

            # ---- servo sign round-trip (see module docstring) ----
            true_cmd_x = out["gimbal_x"] * snap["servoASign"]
            true_cmd_y = out["gimbal_y"] * snap["servoBSign"]
            act_x, act_y = sv.step(true_cmd_x, true_cmd_y)

            # Kalman error vs truth in the fall (> 5 m/s), fitted as a lag:
            # altitude error ~ |v| tau_h, velocity error ~ |a| tau_v.
            if (out["state"] == core.FlightState.DESCENT and s.vel.z < -5.0
                    and prev_v_true is not None):
                _a = (s.vel.z - prev_v_true) / dt
                _eh = out["kf_alt"] - s.pos.z
                _ev = out["kf_vel"] - s.vel.z
                lag_shv += _eh * (-s.vel.z)
                lag_svv += s.vel.z * s.vel.z
                lag_sva += _ev * (-_a)
                lag_saa += _a * _a
                lag_eh_sum += _eh
                lag_ev_sum += _ev
                lag_n += 1
            prev_v_true = s.vel.z
            prev_tilt = out["tilt_deg"]
            prev_state = out["state"].name

            # Truth: how far the legs have actually swung down (nichrome on +
            # band burn + swing). A leg that isn't fully down can't hold the
            # rocket up in the "legs" ground model.
            legs_frac = (_ramp(t, legs_on_t + vcfg.legs_cut_s, vcfg.legs_swing_s)
                         if legs_on_t is not None else 0.0)
            pl.legs_frac = legs_frac
            pl.step(dt, act_x, act_y)

            # Hop: after landing thrust starts, the rocket stopped falling
            # while still > 0.25 m up (it will climb, burn out and fall back).
            if (pl.state.landing_ignite_t is not None and t >= pl.state.landing_ignite_t
                    and pl.state.touchdown_t is None):
                if not hop and s.vel.z >= 0.0 and s.pos.z > 0.25:
                    hop = True
                    hop_stop_h = s.pos.z
                if hop:
                    hop_apex_h = max(hop_apex_h or 0.0, s.pos.z)

            if launched:
                max_tilt = max(max_tilt, out["tilt_deg"])
                if pl.state.touchdown_t is None:
                    err = abs(out["kf_alt"] - s.pos.z)
                    if err > max_kf_err:
                        max_kf_err = err

            if pl.state.touchdown_t is not None and touchdown_speed is None:
                if pl.contact is not None:
                    # legs model: the speed at the instant of first contact
                    cv = pl.contact["vel"]
                    touchdown_speed = abs(cv[2])
                    touchdown_hspeed = math.hypot(cv[0], cv[1])
                else:
                    touchdown_speed = abs(s.vel.z)
                    touchdown_hspeed = math.hypot(s.vel.x, s.vel.y)

            if record:
                rows.append({
                    "t": t, "state": out["state"].name,
                    "h_true": s.pos.z, "v_true": s.vel.z,
                    "x_true": s.pos.x, "y_true": s.pos.y,
                    "quat_true": (s.q.w, s.q.x, s.q.y, s.q.z),
                    "quat_est": out["quat"],
                    "kf_alt": out["kf_alt"], "kf_vel": out["kf_vel"],
                    "kf_bias": out["kf_bias"], "innovation": out["innovation"],
                    "tilt_deg": out["tilt_deg"],
                    "gimbal_x_cmd": out["gimbal_x"], "gimbal_y_cmd": out["gimbal_y"],
                    "gimbal_x_act": act_x, "gimbal_y_act": act_y,
                    "us_a": out["us_a"], "us_b": out["us_b"],
                    "p_x": out["p_x"], "i_x": out["i_x"], "d_x": out["d_x"],
                    "p_y": out["p_y"], "i_y": out["i_y"], "d_y": out["d_y"],
                    "ax": ax, "ay": ay, "az": az, "gx": gx, "gy": gy, "gz": gz,
                    "baro_new": baro_new,
                    "baro_alt": baro_alt, "baro_pa": last_baro_pa,
                    "chute_release": out["chute_release"],
                    "chute_detected": out["chute_detected"],
                    # Deployment TRUTH, for the 3D view: how far the canopy
                    # has actually inflated (the plant's own drag ramp -- zero
                    # while a stuck latch holds it in) and how far the legs
                    # have swung down (nichrome on + band cut + swing).
                    "chute_frac": (_ramp(t, pl.chute_open_t, vcfg.chute_inflation_s)
                                   if pl.chute_deployed else 0.0),
                    "legs_frac": legs_frac,
                    "thrust_n": thrust_now,
                    "fire_landing": out["fire_landing"],
                    "legs_burn": out["legs_burn"],
                })

            t += dt
            i += 1
            if pl.state.landed:
                break
    finally:
        fc.close()

    # Legs are out once the nichrome has cut the band and they've swung down
    # (vehicle.py's legs_cut_s + legs_swing_s); margin = how long before
    # ground contact that was. Negative = the legs were still stowed.
    legs_out_t = None
    legs_margin_s = None
    if legs_on_t is not None:
        legs_out_t = legs_on_t + vcfg.legs_cut_s + vcfg.legs_swing_s
        if pl.state.touchdown_t is not None:
            legs_margin_s = pl.state.touchdown_t - legs_out_t

    summary = {
        "apogee_t": pl.state.apogee_t, "apogee_h": pl.state.apogee_h,
        "touchdown_t": pl.state.touchdown_t,
        "touchdown_speed_mps": touchdown_speed,
        "max_tilt_deg": max_tilt, "max_kf_alt_err_m": max_kf_err,
        "abort_reason": abort_reason.name if abort_reason else None,
        "abort_t": abort_t,
        "chute_releases": chute_releases,
        "chute_detected_t": chute_detected_t,
        "chute_unconfirmed": chute_unconfirmed,
        "legs_on_t": legs_on_t, "legs_off_t": legs_off_t,
        "legs_out_t": legs_out_t, "legs_margin_s": legs_margin_s,
        "final_state": rows[-1]["state"] if rows else None,
        "sim_time_s": t,
    }
    # ---- landing diagnostics ----
    fi = fire_info or {}
    summary.update({
        "fire_t": fi.get("fire_t"), "fire_h_true": fi.get("fire_h_true"),
        "fire_v_true": fi.get("fire_v_true"),
        "fire_kf_alt": fi.get("fire_kf_alt"),
        "fire_kf_vel": fi.get("fire_kf_vel"),
        "fire_tilt_deg": fi.get("fire_tilt_deg"),
        "fire_tilt_prev_deg": fi.get("fire_tilt_prev_deg"),  # >= 15: tilt-limited fire
        "fire_hspeed_true": fi.get("fire_hspeed_true"),
        "land_ignite_t": pl.state.landing_ignite_t,
        "hop": hop, "hop_stop_h": hop_stop_h, "hop_apex_h": hop_apex_h,
        "touchdown_hspeed_mps": touchdown_hspeed,
        "abort_state": abort_state,
        "lag_tau_h_s": (lag_shv / lag_svv) if lag_svv > 0 else None,
        "lag_tau_v_s": (lag_sva / lag_saa) if lag_saa > 0 else None,
        "descent_kf_eh_mean": (lag_eh_sum / lag_n) if lag_n else None,
        "descent_kf_ev_mean": (lag_ev_sum / lag_n) if lag_n else None,
    })
    if pl.ground is not None:
        summary["ground"] = pl.ground_summary()
        summary["contact_vspeed_mps"] = touchdown_speed
        summary["landing_outcome"] = landing_outcome.classify(summary, mode=mode)
    result = {"rows": rows, "events": events, "summary": summary}
    if record_inputs:
        result["inputs"] = inputs
        result["arm_info"] = arm_info
    return result


def main():
    import vehicle
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")  # outcome labels use the ≈ sign
    mode = sys.argv[1] if len(sys.argv) > 1 else "land"
    dll = core.default_dll()
    vcfg = vehicle.default_vehicle_config(dll)
    result = simulate(vcfg, mode=mode, dll_path=dll)
    s = result["summary"]
    print("mode:", mode)
    for k, v in s.items():
        print("  {}: {}".format(k, v))
    return 0


if __name__ == "__main__":
    sys.exit(main())
