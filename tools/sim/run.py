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
import sys

import core
import plant as plant_mod
import sensors as sensors_mod
import servo as servo_mod


def simulate(vcfg, mode="land", dll_path=None, seed=42, arm_t=2.6,
            countdown_s=0.5, max_t=60.0, dt=0.002, baro_hz=100.0,
            record=True, dud_landing_igniter=False):
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
    """
    dll_path = dll_path or core.default_dll()
    snap = core.FlightCore.config_snapshot(dll_path)
    baro_every = max(1, round((1.0 / baro_hz) / dt))

    pl = plant_mod.Plant(vcfg)
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
    events = []
    max_tilt = 0.0
    max_kf_err = 0.0
    touchdown_speed = None
    abort_reason = None
    abort_t = None

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

            for ev in fc.pop_events():
                ev["t"] = t
                events.append(ev)
                if ev["code"] == core.FlightEvent.FIRE_LANDING and not dud_landing_igniter:
                    # e-match ignition delay: burn_table.h's ignition altitudes
                    # already account for this gap between the fire COMMAND and
                    # actual thrust onset (see tools/burn_table.py / IGN_DELAY
                    # in tools/synth_flight.py) -- omitting it here would fire
                    # the motor too early relative to what the table assumed.
                    pl.ignite_landing(t + vcfg.ignition_delay_s)
                elif ev["code"] == core.FlightEvent.FIRE_CHUTE:
                    pl.deploy_chute(t)
                elif ev["code"] == core.FlightEvent.ABORT_DET:
                    abort_reason = core.AbortReason(int(ev["value"]))
                    abort_t = t
                elif ev["code"] == core.FlightEvent.TOUCHDOWN_DET:
                    pass  # plant's own touchdown_t is authoritative for the summary

            # ---- servo sign round-trip (see module docstring) ----
            true_cmd_x = out["gimbal_x"] * snap["servoASign"]
            true_cmd_y = out["gimbal_y"] * snap["servoBSign"]
            act_x, act_y = sv.step(true_cmd_x, true_cmd_y)

            pl.step(dt, act_x, act_y)

            if launched:
                max_tilt = max(max_tilt, out["tilt_deg"])
                if pl.state.touchdown_t is None:
                    err = abs(out["kf_alt"] - s.pos.z)
                    if err > max_kf_err:
                        max_kf_err = err

            if pl.state.touchdown_t is not None and touchdown_speed is None:
                touchdown_speed = abs(s.vel.z)

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
                    "fire_chute": out["fire_chute"], "fire_landing": out["fire_landing"],
                })

            t += dt
            i += 1
            if pl.state.landed:
                break
    finally:
        fc.close()

    summary = {
        "apogee_t": pl.state.apogee_t, "apogee_h": pl.state.apogee_h,
        "touchdown_t": pl.state.touchdown_t,
        "touchdown_speed_mps": touchdown_speed,
        "max_tilt_deg": max_tilt, "max_kf_alt_err_m": max_kf_err,
        "abort_reason": abort_reason.name if abort_reason else None,
        "abort_t": abort_t,
        "final_state": rows[-1]["state"] if rows else None,
        "sim_time_s": t,
    }
    return {"rows": rows, "events": events, "summary": summary}


def main():
    import vehicle
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
