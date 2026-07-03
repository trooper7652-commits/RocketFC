#!/usr/bin/env python3
"""Generate synthetic flight sensor traces for the PC replay harness.

Simulates vertical flight dynamics + a prescribed (wobbling, rolling)
attitude, then produces the sensor data the flight computer would have seen:
500 Hz body-frame accel/gyro (noise + bias) and 100 Hz barometer pressure
(noise + lag), plus truth columns for assertions.

The landing-burn ignition point is derived from the SAME generated
src/core/burn_table.h the firmware compiles, so the C++ state machine's fire
decision can be checked against the physics.

Scenarios (all written to tools/replay/cases/):
  nominal_chute  CHUTE_TEST mode, clean flight
  nominal_land   FULL_LANDING mode, clean flight with F15 landing burn
  dud_igniter    FULL_LANDING, fire command but the motor never lights
  high_tilt      CHUTE_TEST, tilt runs away during boost -> abort
  baro_glitch    CHUTE_TEST, barometer spikes the KF must reject

Pure stdlib:  python tools/synth_flight.py
"""
import json
import math
import os
import random
import re

import burn_table as bt

G = 9.80665
RHO = 1.225
DT = 0.002          # 500 Hz rows
BARO_EVERY = 5      # 100 Hz baro
P0 = 101325.0

HERE = os.path.dirname(os.path.abspath(__file__))
CASES = os.path.join(HERE, "replay", "cases")

# Must match the burn-table generation & config.h placeholders.
MASS_DESCENT = 0.95     # kg, after ascent burnout (F15 loaded)
CDA = 0.004             # m^2
IGN_DELAY = 0.5         # s
CDA_CHUTE = 0.45        # m^2 -> ~5.5 m/s terminal
PAD_S = 3.0             # stillness before ignition
ARM_T = 2.6             # harness arms here (meta)


# ---------------------------------------------------------------------------
def load_burn_table():
    path = os.path.join(HERE, "..", "src", "core", "burn_table.h")
    src = open(path).read()
    vmin = float(re.search(r"V_MIN = ([0-9.]+)f", src).group(1))
    vstep = float(re.search(r"V_STEP = ([0-9.]+)f", src).group(1))
    body = re.search(r"H_IGNITE\[N\] = \{(.*?)\};", src, re.S).group(1)
    table = [float(x) for x in re.findall(r"([0-9.]+)f", body)]

    def hi(v):
        if v <= vmin:
            return table[0]
        idx = (v - vmin) / vstep
        i = int(idx)
        if i >= len(table) - 1:
            return table[-1]
        f = idx - i
        return table[i] * (1 - f) + table[i + 1] * f
    return hi, vmin


class Quat:
    __slots__ = ("w", "x", "y", "z")

    def __init__(self, w=1.0, x=0.0, y=0.0, z=0.0):
        self.w, self.x, self.y, self.z = w, x, y, z

    @staticmethod
    def axis_angle(ax, ay, az, ang):
        n = math.sqrt(ax * ax + ay * ay + az * az) or 1.0
        s = math.sin(ang / 2)
        return Quat(math.cos(ang / 2), ax / n * s, ay / n * s, az / n * s)

    def mul(self, r):
        return Quat(
            self.w * r.w - self.x * r.x - self.y * r.y - self.z * r.z,
            self.w * r.x + self.x * r.w + self.y * r.z - self.z * r.y,
            self.w * r.y - self.x * r.z + self.y * r.w + self.z * r.x,
            self.w * r.z + self.x * r.y - self.y * r.x + self.z * r.w)

    def conj(self):
        return Quat(self.w, -self.x, -self.y, -self.z)

    def rotate_inv(self, v):
        # world -> body
        q = self.conj()
        qv = (q.x, q.y, q.z)
        t = tuple(2 * a for a in cross(qv, v))
        c = cross(qv, t)
        return tuple(v[i] + q.w * t[i] + c[i] for i in range(3))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def attitude(t, t_launch, tilt_deg_fn):
    """Prescribed attitude: tilt about a slowly-precessing world axis, plus
    body roll. Returns quaternion (body->world). Callers freeze `t` at
    touchdown so the attitude (and thus the gyro trace) stays continuous."""
    tilt = math.radians(tilt_deg_fn(t))
    psi = 0.4 * (t - t_launch)               # tilt-axis precession, rad/s
    roll = math.radians(45.0) * max(0.0, t - t_launch)
    q_tilt = Quat.axis_angle(math.cos(psi), math.sin(psi), 0.0, tilt)
    q_roll = Quat.axis_angle(0.0, 0.0, 1.0, roll)
    return q_tilt.mul(q_roll)


def body_rates(q0, q1, dt):
    """omega_body = 2 * vec(q0^-1 * (q1-q0)/dt)"""
    dq = Quat((q1.w - q0.w) / dt, (q1.x - q0.x) / dt,
              (q1.y - q0.y) / dt, (q1.z - q0.z) / dt)
    p = q0.conj().mul(dq)
    return (2 * p.x, 2 * p.y, 2 * p.z)


# ---------------------------------------------------------------------------
def simulate(scenario):
    rng = random.Random(42)
    hi, _ = load_burn_table()

    ascent = bt.Motor(os.path.join(HERE, "motors", "Estes_F15.eng"))
    lander = bt.Motor(os.path.join(HERE, "motors", "Estes_F15.eng"))
    m_pad = MASS_DESCENT + ascent.prop_kg

    mode = scenario["mode"]
    dud = scenario.get("dud", False)
    tilt_runaway = scenario.get("tilt_runaway", False)
    glitch = scenario.get("baro_glitch", False)

    def tilt_deg_fn(t):
        if t < PAD_S:
            return 0.0
        if tilt_runaway:
            return min(40.0, 25.0 * max(0.0, t - (PAD_S + 1.0)))
        ft = t - PAD_S
        return 2.0 * math.exp(-0.15 * ft) * math.sin(2 * math.pi * 0.8 * ft)

    # --- state ---
    h, v = 0.0, 0.0
    t = 0.0
    t_launch = PAD_S
    land_fire_cmd_t = None    # when physics ignition condition tripped
    land_thrust_t0 = None     # when landing thrust actually starts
    chute_open_t = None
    ascent_burnout_t = None
    apogee_t, apogee_h = None, 0.0
    touchdown_t = None
    h_lag = 0.0
    rows = []
    gyro_bias = (math.radians(0.3), math.radians(-0.2), math.radians(0.15))
    accel_bias = (0.03, -0.02, 0.04)
    q_prev = attitude(0.0, t_launch, tilt_deg_fn)

    glitch_times = [PAD_S + 6.0, PAD_S + 9.5, PAD_S + 12.0] if glitch else []

    max_t = 60.0
    i = 0
    while t < max_t:
        airborne = h > 0.001 or (v > 0)
        tb_a = t - t_launch

        # thrust
        T = ascent.thrust(tb_a)
        m = m_pad - ascent.mass_burned(tb_a)
        if ascent_burnout_t is None and tb_a > ascent.burn_time:
            ascent_burnout_t = t
        if land_thrust_t0 is not None:
            tb_l = t - land_thrust_t0
            T += lander.thrust(tb_l)
            m -= lander.mass_burned(tb_l)

        # drag
        cda = CDA
        if chute_open_t is not None:
            k = min(1.0, (t - chute_open_t) / 0.4)   # inflation ramp
            cda = CDA + k * CDA_CHUTE

        tilt_r = math.radians(tilt_deg_fn(t))
        on_ground = h <= 0.0 and t < t_launch + 1.0
        drag_a = 0.5 * RHO * cda * v * abs(v) / m
        a = T * math.cos(tilt_r) / m - G - drag_a
        if on_ground and a < 0:
            a = 0.0
        if touchdown_t is not None:
            # Ground contact: legs/ground kill the remaining velocity at ~12 g.
            # The accelerometer must SEE this impulse — an instantaneous v=0
            # would leave the KF believing the vehicle is still descending.
            if v < -1e-3:
                a = min(-v / DT, 12.0 * G)
            else:
                a, v = 0.0, 0.0

        v += a * DT
        h += v * DT
        if h <= 0.0 and t > t_launch + 1.0 and touchdown_t is None:
            touchdown_t = t

        # events driven by physics truth
        if apogee_t is None and airborne and v < 0 and tb_a > 1.0:
            apogee_t, apogee_h = t, h
        if apogee_t is not None and touchdown_t is None:
            vd = -v
            if (mode == "land" and land_fire_cmd_t is None and vd > 2.0
                    and h <= hi(vd)):
                land_fire_cmd_t = t
                if not dud:
                    land_thrust_t0 = t + IGN_DELAY
            if mode == "chute" and chute_open_t is None and t > apogee_t + 0.6:
                chute_open_t = t
            if dud and land_fire_cmd_t is not None and chute_open_t is None \
                    and t > land_fire_cmd_t + IGN_DELAY + 0.7 + 0.4:
                chute_open_t = t      # FSM dud-abort fires the chute
        if tilt_runaway and chute_open_t is None and ascent_burnout_t is not None \
                and t > ascent_burnout_t + 0.5:
            chute_open_t = t          # FSM tilt-abort waits for burnout

        # ---- sensors ----
        t_att = min(t, touchdown_t) if touchdown_t is not None else t
        q = attitude(t_att, t_launch, tilt_deg_fn)
        gx, gy, gz = body_rates(q_prev, q, DT)
        q_prev = q
        f_world = (0.0, 0.0, a + G)   # specific force
        fb = q.rotate_inv(f_world)
        sig_a = 0.8 if T > 1.0 else 0.12
        ax = fb[0] + accel_bias[0] + rng.gauss(0, sig_a)
        ay = fb[1] + accel_bias[1] + rng.gauss(0, sig_a)
        az = fb[2] + accel_bias[2] + rng.gauss(0, sig_a)
        sg = 0.005 if T > 1.0 else 0.0015
        gx += gyro_bias[0] + rng.gauss(0, sg)
        gy += gyro_bias[1] + rng.gauss(0, sg)
        gz += gyro_bias[2] + rng.gauss(0, sg)

        baro_new = (i % BARO_EVERY) == 0
        p = ""
        if baro_new:
            h_lag += (h - h_lag) * (BARO_EVERY * DT / 0.025)  # ~25 ms lag
            h_meas = h_lag
            for gt in glitch_times:
                if abs(t - gt) < 0.021:
                    h_meas += 40.0 if (int(gt) % 2 == 0) else -40.0
            p_val = P0 * (1.0 - min(h_meas, 40000.0) / 44330.0) ** 5.2553
            p = f"{p_val + rng.gauss(0, 2.0):.2f}"

        rows.append(f"{t:.3f},{ax:.4f},{ay:.4f},{az:.4f},"
                    f"{gx:.5f},{gy:.5f},{gz:.5f},{1 if baro_new else 0},{p},"
                    f"{h:.3f},{v:.3f},{math.degrees(tilt_r):.2f},{T:.1f}")

        t += DT
        i += 1
        if touchdown_t is not None and t > touchdown_t + 5.0:
            break

    return rows, {
        "t_launch": t_launch,
        "apogee_t": apogee_t, "apogee_h": apogee_h,
        "land_fire_cmd_t": land_fire_cmd_t,
        "touchdown_t": touchdown_t,
    }


# ---------------------------------------------------------------------------
SCENARIOS = {
    "nominal_chute": {
        "mode": "chute",
        "expect_states": ["IDLE", "ARMED", "BOOST", "COAST", "APOGEE",
                          "DESCENT_CHUTE", "TOUCHDOWN"],
        "max_kf_alt_err": 3.0, "expect_abort": None,
    },
    "nominal_land": {
        "mode": "land",
        "expect_states": ["IDLE", "ARMED", "BOOST", "COAST", "APOGEE",
                          "DESCENT", "LANDING_BURN", "TOUCHDOWN"],
        "max_kf_alt_err": 3.0, "expect_abort": None,
    },
    "dud_igniter": {
        "mode": "land", "dud": True,
        "expect_states": ["IDLE", "ARMED", "BOOST", "COAST", "APOGEE",
                          "DESCENT", "LANDING_BURN", "ABORT", "DESCENT_CHUTE",
                          "TOUCHDOWN"],
        "max_kf_alt_err": 3.0, "expect_abort": "DUD_IGNITER",
    },
    "high_tilt": {
        "mode": "chute", "tilt_runaway": True,
        "expect_states": ["IDLE", "ARMED", "BOOST", "ABORT", "DESCENT_CHUTE",
                          "TOUCHDOWN"],
        "max_kf_alt_err": 4.0, "expect_abort": "TILT",
    },
    "baro_glitch": {
        "mode": "chute", "baro_glitch": True,
        "expect_states": ["IDLE", "ARMED", "BOOST", "COAST", "APOGEE",
                          "DESCENT_CHUTE", "TOUCHDOWN"],
        "max_kf_alt_err": 5.0, "expect_abort": None,
    },
}


def main():
    os.makedirs(CASES, exist_ok=True)
    for name, sc in SCENARIOS.items():
        rows, truth = simulate(sc)
        meta = {
            "scenario": name,
            "mode": sc["mode"],
            "arm_t": ARM_T,
            "expect_states": sc["expect_states"],
            "expect_abort": sc["expect_abort"],
            "max_kf_alt_err": sc["max_kf_alt_err"],
            "boost_end_t": None,
            "fire_window": ([truth["land_fire_cmd_t"] - 0.5,
                             truth["land_fire_cmd_t"] + 0.5]
                            if truth["land_fire_cmd_t"] else None),
            "touchdown_t": truth["touchdown_t"],
            "apogee_h": truth["apogee_h"],
        }
        path = os.path.join(CASES, name + ".csv")
        with open(path, "w", newline="\n") as f:
            f.write("#META " + json.dumps(meta) + "\n")
            f.write("t,ax,ay,az,gx,gy,gz,baro_new,p_pa,h_true,v_true,"
                    "tilt_true_deg,thrust_n\n")
            f.write("\n".join(rows) + "\n")
        print(f"{name:15s} rows={len(rows):6d} apogee={truth['apogee_h']:6.1f} m "
              f"touchdown={truth['touchdown_t'] and round(truth['touchdown_t'],1)} s "
              f"-> {os.path.relpath(path, HERE)}")


if __name__ == "__main__":
    main()
