"""The PC half of a closed-loop bench flight: the simulated rocket.

tools/sim's plant (truth), sensor model and servo model, plus the scenario's
fault injection, stepped to the FLIGHT COMPUTER's clock. One class serves both
sides of the comparison, so they can only differ in who runs the flight code:

  runner.py   the real Teensy (telemetry in, sensor frames out, over USB)
  twin.py     the same firmware built for the PC (tools/sim/core.py DLL)

Per flight-computer tick k (time t_k), in this order:
  advance(t_k)   plant over [t_(k-1), t_k] with the commands already held
  command(...)   the tick's new commands (edges: landing fire, chute
                 release, legs) take effect from t_k
  frame()        the sensor frame for t_k, which the NEXT tick uses

That is one tick (2 ms) more sensor-to-actuator lag than tools/sim/run.py,
which is exactly what the USB loop has; the twin reproduces it.
Mirrors run.simulate() wherever it can (same models, same seeds, same pad
calibration, same row and summary fields).
"""
import copy
import math
import os
import random
import sys

LIVE_DIR = os.path.dirname(os.path.abspath(__file__))
HIL_DIR = os.path.dirname(LIVE_DIR)
REPO_ROOT = os.path.dirname(os.path.dirname(HIL_DIR))
SIM_DIR = os.path.join(REPO_ROOT, "tools", "sim")
SKETCH_DIR = os.path.join(HIL_DIR, "build", "RocketFC_HIL")
if SIM_DIR not in sys.path:
    sys.path.insert(0, SIM_DIR)

import core  # noqa: E402
import landing_outcome  # noqa: E402
import plant as plant_mod  # noqa: E402
import sensors as sensors_mod  # noqa: E402
import servo as servo_mod  # noqa: E402
import vehicle  # noqa: E402

G0 = 9.80665
NAN = float("nan")


def _ramp(t, t0, duration):
    return min(1.0, max(0.0, (t - t0) / max(duration, 1e-6)))


def load_profile(sketch_dir=SKETCH_DIR):
    import json
    path = os.path.join(sketch_dir, "hil_profile.json")
    if not os.path.isfile(path):
        return {"name": "none"}
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def flight_dll(sketch_dir=SKETCH_DIR):
    """The flight core compiled from the HIL build tree's src/ (the same
    config.h + burn table the Teensy was built with)."""
    return core.build_from_src(os.path.join(sketch_dir, "src"))


def vehicle_for(dll, profile):
    """Plant nominal: the build's config.h, then the profile's plant settings."""
    vc = vehicle.default_vehicle_config(dll)
    for k, v in (profile.get("plant") or {}).items():
        if not hasattr(vc, k):
            raise ValueError("profile plant setting {} is not a VehicleConfig field".format(k))
        if k.endswith("_path") and isinstance(v, str) and not os.path.isabs(v):
            v = os.path.join(REPO_ROOT, v)
        setattr(vc, k, v)
    return vc


class LiveFlight:
    DT = 0.002       # plant substep (= the core's 500 Hz tick)
    BARO_DT = 0.01   # 100 Hz barometer, as run.py

    def __init__(self, scen, vcfg, snap):
        v = copy.deepcopy(vcfg)
        v.wind_mps = scen.wind_mps
        v.gust_mps = scen.gust_mps if scen.gust_s > 0 else 0.0
        v.gust_time_s = 0.0          # set at launch (scenario times are launch-relative)
        v.gust_dur_s = max(scen.gust_s, 1e-3)
        v.chute_stuck_releases = int(scen.chute_stuck_releases)
        v.landing_thrust_scale = scen.landing_thrust_pct / 100.0
        if scen.ignition_delay_s >= 0:
            v.ignition_delay_s = scen.ignition_delay_s
        self.v = v
        self.sc = scen
        self.snap = snap
        self.pl = plant_mod.Plant(v)
        if self.pl.ground is not None:
            r = random.Random(scen.seed * 7907 + 3)   # run.py's slope stream
            self.pl.ground.set_slope(r.uniform(0.0, v.ground_slope_max_deg),
                                     r.uniform(0.0, 360.0))
        self.sm = sensors_mod.SensorModel(v, seed=scen.seed)
        self.sv = servo_mod.GimbalServos(math.radians(v.servo_slew_dps),
                                         v.servo_lag_tau_s, snap["gimbalMaxRad"], self.DT)
        self.frng = random.Random(scen.seed * 31 + 7)   # fault noise, own stream
        self.t = 0.0
        self.launch_t = None
        self.cmd = {"gimbal_x": 0.0, "gimbal_y": 0.0, "fire": False,
                    "chute_release": False, "legs": False}
        self.act = (0.0, 0.0)
        self.legs_frac = 0.0
        self.fire_t = None
        self.fire_truth = {}
        self.legs_on_t = None
        self.chute_releases = 0
        # pad calibration (run.py's), until finish_cal()
        self._gs = [0.0, 0.0, 0.0]
        self._as = [0.0, 0.0, 0.0]
        self._n = 0
        self._pn = 0
        self._pmean = 0.0
        self._pm2 = 0.0
        self.cal = None
        self.p0 = 101325.0
        self.gyro_bias = (0.0, 0.0, 0.0)
        # sensors
        self.seq = 0
        self._next_baro = 0.0
        self._last_imu_t = -self.DT
        self._frozen = None
        self.last_baro_alt = 0.0
        self.last_baro_pa = 101325.0
        self.last_frame = None
        # diagnostics (run.py's)
        self.touchdown_speed = None
        self.touchdown_hspeed = None
        self.hop = False
        self.hop_stop_h = None
        self.hop_apex_h = None

    # -------------------------------------------------------------- time
    def rel(self):
        """Seconds since the Launch command (None before)."""
        return None if self.launch_t is None else self.t - self.launch_t

    def launch(self):
        self.launch_t = self.t
        self.pl.ignite_ascent(self.t)
        if self.v.gust_mps:
            self.pl.v.gust_time_s = self.t + self.sc.gust_at_s

    def advance(self, t_target):
        s = self.pl.state
        sa, sb = self.snap["servoASign"], self.snap["servoBSign"]
        while self.t + 0.5 * self.DT <= t_target:
            if s.landed:            # on the ground: time runs, the plant is done
                self.t = t_target
                break
            # servo sign round-trip, as run.py
            self.act = self.sv.step(self.cmd["gimbal_x"] * sa, self.cmd["gimbal_y"] * sb)
            self.legs_frac = (_ramp(self.t, self.legs_on_t + self.v.legs_cut_s,
                                    self.v.legs_swing_s)
                              if self.legs_on_t is not None else 0.0)
            self.pl.legs_frac = self.legs_frac
            self.pl.step(self.DT, self.act[0], self.act[1])
            self.t += self.DT
            self._diagnostics()

    def _diagnostics(self):
        s = self.pl.state
        if (s.landing_ignite_t is not None and self.t >= s.landing_ignite_t
                and s.touchdown_t is None):
            if not self.hop and s.vel.z >= 0.0 and s.pos.z > 0.25:
                self.hop = True
                self.hop_stop_h = s.pos.z
            if self.hop:
                self.hop_apex_h = max(self.hop_apex_h or 0.0, s.pos.z)
        if s.touchdown_t is not None and self.touchdown_speed is None:
            if self.pl.contact is not None:
                cv = self.pl.contact["vel"]
                self.touchdown_speed = abs(cv[2])
                self.touchdown_hspeed = math.hypot(cv[0], cv[1])
            else:
                self.touchdown_speed = abs(s.vel.z)
                self.touchdown_hspeed = math.hypot(s.vel.x, s.vel.y)

    def command(self, gimbal_x, gimbal_y, fire, chute_release, legs, t_cmd):
        """The flight computer's outputs for the tick at t_cmd."""
        s = self.pl.state
        if fire and not self.cmd["fire"] and self.fire_t is None:
            self.fire_t = t_cmd
            self.fire_truth = {"fire_h_true": s.pos.z, "fire_v_true": s.vel.z,
                               "fire_hspeed_true": math.hypot(s.vel.x, s.vel.y)}
            if not self.sc.dud_landing_igniter:
                self.pl.ignite_landing(t_cmd + self.v.ignition_delay_s)
        if chute_release and not self.cmd["chute_release"]:
            self.chute_releases += 1        # spring latch: the first N stick
            if self.chute_releases > self.v.chute_stuck_releases:
                self.pl.deploy_chute(t_cmd)
        if legs and not self.cmd["legs"] and self.legs_on_t is None:
            self.legs_on_t = t_cmd
        self.cmd = {"gimbal_x": gimbal_x, "gimbal_y": gimbal_y, "fire": bool(fire),
                    "chute_release": bool(chute_release), "legs": bool(legs)}

    # ------------------------------------------------------------ sensors
    def finish_cal(self):
        """End the pad calibration (run.py's): gyro bias, pad accel, baro var.
        Returns (pad_accel, baro_var) for `live` / pad_level_init()."""
        n = max(self._n, 1)
        self.gyro_bias = tuple(g / n for g in self._gs)
        pad = tuple(a / n for a in self._as)
        self.p0 = self._pmean if self._pmean > 0 else 101325.0
        m_per_pa = 8434.6 / self.p0
        var = (self._pm2 / (self._pn - 1)) * m_per_pa ** 2 if self._pn > 1 else 0.25
        self.cal = {"pad_accel": pad, "baro_var": max(var, 0.0025), "t": self.t}
        return pad, self.cal["baro_var"]

    def frame(self):
        """Sensor frame for now: (seq, flags, accel, gyro, baro_alt), or None
        while an IMU dropout is injected (no frame at all -> link loss)."""
        s = self.pl.state
        rel = self.rel()
        if s.landed:
            sf = s.q.rotate_inv(plant_mod.V3(0.0, 0.0, G0))
            om = plant_mod.V3(0.0, 0.0, 0.0)
            thrust = 0.0
        else:
            sf, om = s.specific_force_body, s.omega
            _, thrust = self.pl.mass_and_thrust(self.t)
        dt = max(self.t - self._last_imu_t, 1e-4)
        self._last_imu_t = self.t
        (ax, ay, az), (gx, gy, gz) = self.sm.imu(sf, om, dt, thrust)
        if self.cal is None:
            for i, x in enumerate((gx, gy, gz)):
                self._gs[i] += x
            for i, x in enumerate((ax, ay, az)):
                self._as[i] += x
            self._n += 1

        flags = 0
        if self.t + 1e-9 >= self._next_baro:
            while self._next_baro <= self.t + 1e-9:
                self._next_baro += self.BARO_DT
            if getattr(self.v, "baro_q_coeff", 0.0) != 0.0:
                rel_air = s.vel - self.pl._wind(self.t)
                p = self.sm.baro(s.pos.z, self.BARO_DT, airspeed=rel_air.norm())
            else:
                p = self.sm.baro(s.pos.z, self.BARO_DT)
            if self.cal is None:
                self._pn += 1
                d = p - self._pmean
                self._pmean += d / self._pn
                self._pm2 += d * (p - self._pmean)
            alt = 44330.0 * (1.0 - (p / self.p0) ** 0.190295)
            if self.sc.window("baro_offset", rel):
                alt += self.sc.baro_offset_m
            if self.sc.window("baro_noise", rel):
                alt += self.frng.gauss(0.0, self.sc.baro_noise_m)
            if not self.sc.window("baro_dropout", rel):
                flags |= 1
                self.last_baro_alt = alt
                self.last_baro_pa = p

        bx, by, bz = self.gyro_bias
        a = (ax, ay, az)
        g = (gx - bx, gy - by, gz - bz)
        if self.sc.window("imu_frozen", rel):
            if self._frozen is None:
                self._frozen = (a, g)
            a, g = self._frozen
        else:
            self._frozen = None
        if self.sc.window("imu_dropout", rel):
            return None
        self.seq = (self.seq + 1) & 0xFFFF
        self.last_frame = (self.seq, flags, a, g, self.last_baro_alt)
        return self.last_frame

    # --------------------------------------------------------------- rows
    def row(self, out):
        """One row in run.simulate()'s format. `out`: the flight computer's
        outputs for this tick (state name, kf_alt/vel, tilt, quat, gimbal,
        chute/fire/legs levels, PID terms)."""
        s = self.pl.state
        thrust = 0.0 if s.landed else self.pl.mass_and_thrust(self.t)[1]
        lf = self.last_frame
        r = {
            "t": self.t, "state": out["state"],
            "h_true": s.pos.z, "v_true": s.vel.z, "x_true": s.pos.x, "y_true": s.pos.y,
            "quat_true": (s.q.w, s.q.x, s.q.y, s.q.z),
            "quat_est": out.get("quat", (1.0, 0.0, 0.0, 0.0)),
            "kf_alt": out["kf_alt"], "kf_vel": out["kf_vel"],
            "kf_bias": out.get("kf_bias", NAN), "innovation": out.get("innovation", NAN),
            "tilt_deg": out["tilt_deg"],
            "gimbal_x_cmd": out["gimbal_x"], "gimbal_y_cmd": out["gimbal_y"],
            "gimbal_x_act": self.act[0], "gimbal_y_act": self.act[1],
            "us_a": out.get("us_a", 0), "us_b": out.get("us_b", 0),
            "baro_new": bool(lf and lf[1] & 1),
            "baro_alt": self.last_baro_alt, "baro_pa": self.last_baro_pa,
            "chute_release": out["chute_release"], "chute_detected": out["chute_detected"],
            "chute_frac": (_ramp(self.t, self.pl.chute_open_t, self.v.chute_inflation_s)
                           if self.pl.chute_deployed else 0.0),
            "legs_frac": self.legs_frac, "thrust_n": thrust,
            "fire_landing": out["fire_landing"], "legs_burn": out["legs_burn"],
        }
        for k in ("p_x", "i_x", "d_x", "p_y", "i_y", "d_y"):
            r[k] = out.get(k, NAN)
        if lf:
            r["ax"], r["ay"], r["az"] = lf[2]
            r["gx"], r["gy"], r["gz"] = lf[3]
        return r


# ------------------------------------------------------------------ results
_BY_LOG_NAME = {core.log_event_name(c): c for c in core.FlightEvent}


def event_code(name):
    """A console/log event name (e.g. "APOGEE") or enum name ("APOGEE_DET")
    -> core.FlightEvent (the name itself if unknown)."""
    if name in _BY_LOG_NAME:
        return _BY_LOG_NAME[name]
    try:
        return core.FlightEvent[name]
    except KeyError:
        return name


def summarize(fl, rows, events, mode):
    """run.simulate()'s summary from a LiveFlight, its rows and its events
    ({t, code, value}). Plant time throughout (see rebase())."""
    st = fl.pl.state
    launch_t = fl.launch_t if fl.launch_t is not None else 0.0
    max_tilt = 0.0
    max_kf_err = 0.0
    for r in rows:
        if r["t"] >= launch_t and fl.launch_t is not None:
            max_tilt = max(max_tilt, r["tilt_deg"])
            if st.touchdown_t is None or r["t"] < st.touchdown_t:
                max_kf_err = max(max_kf_err, abs(r["kf_alt"] - r["h_true"]))
    name = lambda e: getattr(e["code"], "name", str(e["code"]))  # noqa: E731
    abort_reason = abort_t = chute_detected_t = legs_off_t = None
    chute_unconfirmed = False
    abort_state = None
    fire_ev = None
    for e in events:
        n = name(e)
        if n == "ABORT_DET" and abort_t is None:
            abort_t = e["t"]
            try:
                abort_reason = core.AbortReason(int(e["value"])).name
            except ValueError:
                abort_reason = str(e["value"])
            prev = [r["state"] for r in rows if r["t"] < e["t"]]
            abort_state = prev[-1] if prev else None
        elif n == "CHUTE_DETECTED" and chute_detected_t is None:
            chute_detected_t = e["t"]
        elif n == "CHUTE_UNCONFIRMED":
            chute_unconfirmed = True
        elif n == "LEGS_BURN_OFF":
            legs_off_t = e["t"]
        elif n == "FIRE_LANDING" and fire_ev is None:
            fire_ev = e
    fire_row = None
    if fl.fire_t is not None:
        prev = [r for r in rows if r["t"] <= fl.fire_t + 1e-9]
        fire_row = prev[-1] if prev else None
    prev_tilt = None
    if fire_row is not None:
        before = [r for r in rows if r["t"] < fire_row["t"] - 1e-9]
        prev_tilt = before[-1]["tilt_deg"] if before else None
    legs_out_t = legs_margin = None
    if fl.legs_on_t is not None:
        legs_out_t = fl.legs_on_t + fl.v.legs_cut_s + fl.v.legs_swing_s
        if st.touchdown_t is not None:
            legs_margin = st.touchdown_t - legs_out_t
    s = {
        "apogee_t": st.apogee_t, "apogee_h": st.apogee_h,
        "touchdown_t": st.touchdown_t, "touchdown_speed_mps": fl.touchdown_speed,
        "max_tilt_deg": max_tilt, "max_kf_alt_err_m": max_kf_err,
        "abort_reason": abort_reason, "abort_t": abort_t,
        "chute_releases": fl.chute_releases, "chute_detected_t": chute_detected_t,
        "chute_unconfirmed": chute_unconfirmed,
        "legs_on_t": fl.legs_on_t, "legs_off_t": legs_off_t,
        "legs_out_t": legs_out_t, "legs_margin_s": legs_margin,
        "final_state": rows[-1]["state"] if rows else None,
        "sim_time_s": fl.t,
        "fire_t": fl.fire_t,
        "fire_h_true": fl.fire_truth.get("fire_h_true"),
        "fire_v_true": fl.fire_truth.get("fire_v_true"),
        "fire_kf_alt": fire_row["kf_alt"] if fire_row else None,
        "fire_kf_vel": fire_row["kf_vel"] if fire_row else None,
        "fire_tilt_deg": fire_row["tilt_deg"] if fire_row else None,
        "fire_tilt_prev_deg": prev_tilt,
        "fire_hspeed_true": fl.fire_truth.get("fire_hspeed_true"),
        "land_ignite_t": st.landing_ignite_t,
        "hop": fl.hop, "hop_stop_h": fl.hop_stop_h, "hop_apex_h": fl.hop_apex_h,
        "touchdown_hspeed_mps": fl.touchdown_hspeed,
        "abort_state": abort_state,
        "launch_t": fl.launch_t,
    }
    if fl.pl.ground is not None:
        s["ground"] = fl.pl.ground_summary()
        s["contact_vspeed_mps"] = fl.touchdown_speed
        s["landing_outcome"] = landing_outcome.classify(s, mode=mode)
    return s


_SUMMARY_TIMES = ("apogee_t", "touchdown_t", "abort_t", "chute_detected_t", "legs_on_t",
                  "legs_off_t", "legs_out_t", "fire_t", "land_ignite_t", "launch_t")


def rebase(result, origin, mode):
    """Shift every time in a result by -origin (so a long pad wait doesn't
    push the flight off the right of the dashboard) and drop rows before it."""
    result["rows"] = [dict(r, t=r["t"] - origin) for r in result["rows"] if r["t"] >= origin]
    result["events"] = [dict(e, t=e["t"] - origin) for e in result["events"] if e["t"] >= origin]
    s = result["summary"]
    for k in _SUMMARY_TIMES:
        if s.get(k) is not None:
            s[k] -= origin
    g = s.get("ground")
    if g:
        if g.get("contact") and g["contact"].get("t") is not None:
            g["contact"] = dict(g["contact"], t=g["contact"]["t"] - origin)
        for k in ("fall_t", "rest_upright_t"):
            if g.get(k) is not None:
                g[k] -= origin
    s["sim_time_s"] = (s.get("sim_time_s") or origin) - origin
    if g is not None:   # its verdict carries times: classify again on the new base
        s["landing_outcome"] = landing_outcome.classify(s, mode=mode)
    return result
