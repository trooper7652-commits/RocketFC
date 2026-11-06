"""Port the landing-study analysis tooling into a RocketFC tree's tools/sim.

Same edits as scaffold_patch.py (verified result-neutral in the workspace
copies), minus the analysis-only forced-fire hook, with repo-style comments:
  - vehicle.py: landing_thrust_scale, landing_time_scale, baro_q_coeff
    (all default to "no change")
  - plant.py / sensors.py: use them only when non-default
  - run.py: extra summary fields (fire state, hop, abort phase, lag, KF error)
usage: python -I port_sim_tooling.py <RocketFC root>
"""
import os
import sys


def patch(path, pairs):
    with open(path, encoding="utf-8", newline="") as f:
        s = f.read()
    for old, new in pairs:
        n = s.count(old)
        if n != 1:
            raise SystemExit(f"{path}: expected 1 match, found {n}:\n{old}")
        s = s.replace(old, new)
    with open(path, "w", encoding="utf-8", newline="") as f:
        f.write(s)


def main():
    sim = os.path.join(os.path.abspath(sys.argv[1]), "tools", "sim")

    patch(os.path.join(sim, "vehicle.py"), [(
        "    ignition_delay_s: float = 0.5        # [MEASURE on ThrustStand] e-match -> thrust (s)\n",
        "    ignition_delay_s: float = 0.5        # [MEASURE on ThrustStand] e-match -> thrust (s)\n"
        "    # Plant-side motor mismatch for robustness studies (defaults = no change):\n"
        "    # the flight software keeps assuming the nominal .eng curve.\n"
        "    landing_thrust_scale: float = 1.0    # multiplies landing thrust (impulse scales too)\n"
        "    landing_time_scale: float = 1.0      # stretches the landing burn, impulse kept\n",
    ), (
        "    baro_lag_s: float = 0.025           # barometer response lag (s)\n",
        "    baro_lag_s: float = 0.025           # barometer response lag (s); applied once per\n"
        "                                        #   baro sample, so values <= the sample\n"
        "                                        #   interval (10 ms) act like no lag\n"
        "    # Static-pressure error from airflow past the vent holes, as a fraction of\n"
        "    # dynamic pressure q = rho*v^2/2 (+ = reads high pressure = low altitude).\n"
        "    baro_q_coeff: float = 0.0\n",
    )])

    patch(os.path.join(sim, "plant.py"), [(
        "        self.landing_motor = bt.Motor(vcfg.landing_eng_path)\n",
        "        _ls = getattr(vcfg, \"landing_thrust_scale\", 1.0)\n"
        "        self.landing_motor = (bt.Motor(vcfg.landing_eng_path) if _ls == 1.0\n"
        "                              else bt.Motor(vcfg.landing_eng_path, scale=_ls))\n"
        "        self._land_k = getattr(vcfg, \"landing_time_scale\", 1.0)\n",
    ), (
        "            tb = t - self.state.landing_ignite_t\n"
        "            thrust += self.landing_motor.thrust(tb)\n"
        "            m -= self.landing_motor.mass_burned(tb)\n",
        "            tb = t - self.state.landing_ignite_t\n"
        "            if self._land_k == 1.0:\n"
        "                thrust += self.landing_motor.thrust(tb)\n"
        "                m -= self.landing_motor.mass_burned(tb)\n"
        "            else:  # stretched burn, same impulse: T'(t) = T(t/k)/k\n"
        "                thrust += self.landing_motor.thrust(tb / self._land_k) / self._land_k\n"
        "                m -= self.landing_motor.mass_burned(tb / self._land_k)\n",
    )])

    patch(os.path.join(sim, "sensors.py"), [(
        "    def baro(self, true_alt_m, dt_since_last_sample):\n",
        "    def baro(self, true_alt_m, dt_since_last_sample, airspeed=0.0):\n",
    ), (
        "        return p + self.rng.gauss(0, v.baro_noise_pa)\n",
        "        qc = getattr(v, \"baro_q_coeff\", 0.0)\n"
        "        if qc != 0.0:  # static-port error: a fraction of dynamic pressure\n"
        "            p += qc * 0.5 * 1.225 * airspeed * airspeed\n"
        "        return p + self.rng.gauss(0, v.baro_noise_pa)\n",
    )])

    patch(os.path.join(sim, "run.py"), [(
        "    legs_on_t = None\n    legs_off_t = None\n",
        "    legs_on_t = None\n    legs_off_t = None\n"
        "    # ---- landing diagnostics (read-only; never fed back into the flight) ----\n"
        "    fire_info = None\n"
        "    prev_tilt = None\n"
        "    prev_state = None\n"
        "    abort_state = None\n"
        "    hop = False\n"
        "    hop_stop_h = None\n"
        "    hop_apex_h = None\n"
        "    touchdown_hspeed = None\n"
        "    lag_shv = lag_svv = lag_sva = lag_saa = 0.0\n"
        "    lag_n = 0\n"
        "    lag_eh_sum = lag_ev_sum = 0.0\n"
        "    prev_v_true = None\n",
    ), (
        "                p_pa = sm.baro(s.pos.z, baro_every * dt)\n",
        "                if getattr(vcfg, \"baro_q_coeff\", 0.0) != 0.0:\n"
        "                    _w = pl._wind(t)\n"
        "                    _rel = s.vel - _w\n"
        "                    p_pa = sm.baro(s.pos.z, baro_every * dt, airspeed=_rel.norm())\n"
        "                else:\n"
        "                    p_pa = sm.baro(s.pos.z, baro_every * dt)\n",
    ), (
        "            for ev in fc.pop_events():\n"
        "                ev[\"t\"] = t\n"
        "                events.append(ev)\n",
        "            for ev in fc.pop_events():\n"
        "                ev[\"t\"] = t\n"
        "                events.append(ev)\n"
        "                if ev[\"code\"] == core.FlightEvent.FIRE_LANDING and fire_info is None:\n"
        "                    fire_info = {\n"
        "                        \"fire_t\": t, \"fire_h_true\": s.pos.z,\n"
        "                        \"fire_v_true\": s.vel.z,\n"
        "                        \"fire_kf_alt\": out[\"kf_alt\"],\n"
        "                        \"fire_kf_vel\": out[\"kf_vel\"],\n"
        "                        \"fire_tilt_deg\": out[\"tilt_deg\"],\n"
        "                        \"fire_tilt_prev_deg\": prev_tilt,\n"
        "                        \"fire_hspeed_true\": math.hypot(s.vel.x, s.vel.y),\n"
        "                    }\n"
        "                if ev[\"code\"] == core.FlightEvent.ABORT_DET:\n"
        "                    abort_state = prev_state\n",
    ), (
        "            pl.step(dt, act_x, act_y)\n",
        "            # Kalman error vs truth in the fall (> 5 m/s), fitted as a lag:\n"
        "            # altitude error ~ |v| tau_h, velocity error ~ |a| tau_v.\n"
        "            if (out[\"state\"] == core.FlightState.DESCENT and s.vel.z < -5.0\n"
        "                    and prev_v_true is not None):\n"
        "                _a = (s.vel.z - prev_v_true) / dt\n"
        "                _eh = out[\"kf_alt\"] - s.pos.z\n"
        "                _ev = out[\"kf_vel\"] - s.vel.z\n"
        "                lag_shv += _eh * (-s.vel.z)\n"
        "                lag_svv += s.vel.z * s.vel.z\n"
        "                lag_sva += _ev * (-_a)\n"
        "                lag_saa += _a * _a\n"
        "                lag_eh_sum += _eh\n"
        "                lag_ev_sum += _ev\n"
        "                lag_n += 1\n"
        "            prev_v_true = s.vel.z\n"
        "            prev_tilt = out[\"tilt_deg\"]\n"
        "            prev_state = out[\"state\"].name\n"
        "\n"
        "            pl.step(dt, act_x, act_y)\n"
        "\n"
        "            # Hop: after landing thrust starts, the rocket stopped falling\n"
        "            # while still > 0.25 m up (it will climb, burn out and fall back).\n"
        "            if (pl.state.landing_ignite_t is not None and t >= pl.state.landing_ignite_t\n"
        "                    and pl.state.touchdown_t is None):\n"
        "                if not hop and s.vel.z >= 0.0 and s.pos.z > 0.25:\n"
        "                    hop = True\n"
        "                    hop_stop_h = s.pos.z\n"
        "                if hop:\n"
        "                    hop_apex_h = max(hop_apex_h or 0.0, s.pos.z)\n",
    ), (
        "            if pl.state.touchdown_t is not None and touchdown_speed is None:\n"
        "                touchdown_speed = abs(s.vel.z)\n",
        "            if pl.state.touchdown_t is not None and touchdown_speed is None:\n"
        "                touchdown_speed = abs(s.vel.z)\n"
        "                touchdown_hspeed = math.hypot(s.vel.x, s.vel.y)\n",
    ), (
        "        \"final_state\": rows[-1][\"state\"] if rows else None,\n"
        "        \"sim_time_s\": t,\n"
        "    }\n",
        "        \"final_state\": rows[-1][\"state\"] if rows else None,\n"
        "        \"sim_time_s\": t,\n"
        "    }\n"
        "    # ---- landing diagnostics ----\n"
        "    fi = fire_info or {}\n"
        "    summary.update({\n"
        "        \"fire_t\": fi.get(\"fire_t\"), \"fire_h_true\": fi.get(\"fire_h_true\"),\n"
        "        \"fire_v_true\": fi.get(\"fire_v_true\"),\n"
        "        \"fire_kf_alt\": fi.get(\"fire_kf_alt\"),\n"
        "        \"fire_kf_vel\": fi.get(\"fire_kf_vel\"),\n"
        "        \"fire_tilt_deg\": fi.get(\"fire_tilt_deg\"),\n"
        "        \"fire_tilt_prev_deg\": fi.get(\"fire_tilt_prev_deg\"),  # >= 15: tilt-limited fire\n"
        "        \"fire_hspeed_true\": fi.get(\"fire_hspeed_true\"),\n"
        "        \"land_ignite_t\": pl.state.landing_ignite_t,\n"
        "        \"hop\": hop, \"hop_stop_h\": hop_stop_h, \"hop_apex_h\": hop_apex_h,\n"
        "        \"touchdown_hspeed_mps\": touchdown_hspeed,\n"
        "        \"abort_state\": abort_state,\n"
        "        \"lag_tau_h_s\": (lag_shv / lag_svv) if lag_svv > 0 else None,\n"
        "        \"lag_tau_v_s\": (lag_sva / lag_saa) if lag_saa > 0 else None,\n"
        "        \"descent_kf_eh_mean\": (lag_eh_sum / lag_n) if lag_n else None,\n"
        "        \"descent_kf_ev_mean\": (lag_ev_sum / lag_n) if lag_n else None,\n"
        "    })\n",
    )])
    print("sim tooling patched:", sim)


if __name__ == "__main__":
    main()
