"""Apply the SIM-ONLY analysis scaffolding to one workspace copy of RocketFC.

Never ported to the real project. Every change is neutral by default:
  - tools/sim/run.py: extra summary fields (fire state, hop, lag, abort phase,
    horizontal touchdown speed) computed from values the loop already has; an
    optional oracle_fire_ms hook (only callable against a DLL built from the
    ws_oracle copy, which exports rfc_oracle_set_fire_ms).
  - tools/sim/vehicle.py: plant/sensor perturbation knobs, all defaulting to
    "no change" (thrust scale 1, time scale 1, baro q-error 0).
  - tools/sim/plant.py / sensors.py: use those knobs only when non-default, so
    the default code path is arithmetically identical to the original.

usage: python -I scaffold_patch.py <workspace>/RocketFC [--oracle]
  --oracle additionally adds the forced-fire hook to src/core/flight_state.h
  and tools/sim/bridge.cpp (ws_oracle copy only).
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
    root = os.path.abspath(sys.argv[1])
    oracle = "--oracle" in sys.argv
    sim = os.path.join(root, "tools", "sim")

    # ---------------- vehicle.py: perturbation knobs ----------------
    patch(os.path.join(sim, "vehicle.py"), [(
        "    ignition_delay_s: float = 0.5        # [MEASURE on ThrustStand] e-match -> thrust (s)\n",
        "    ignition_delay_s: float = 0.5        # [MEASURE on ThrustStand] e-match -> thrust (s)\n"
        "    # ANALYSIS SCAFFOLDING (landing-solver study): plant-side model mismatch.\n"
        "    landing_thrust_scale: float = 1.0    # multiplies landing thrust (impulse scales too)\n"
        "    landing_time_scale: float = 1.0      # stretches the landing burn, impulse kept\n",
    ), (
        "    baro_lag_s: float = 0.025           # barometer response lag (s)\n",
        "    baro_lag_s: float = 0.025           # barometer response lag (s)\n"
        "    # ANALYSIS SCAFFOLDING: static-pressure error from airflow, as a fraction\n"
        "    # of dynamic pressure q = rho*v^2/2 (+ = port reads high pressure = low alt).\n"
        "    baro_q_coeff: float = 0.0\n",
    )])

    # ---------------- plant.py: thrust/time scale ----------------
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

    # ---------------- sensors.py: baro q-error ----------------
    patch(os.path.join(sim, "sensors.py"), [(
        "    def baro(self, true_alt_m, dt_since_last_sample):\n",
        "    def baro(self, true_alt_m, dt_since_last_sample, airspeed=0.0):\n",
    ), (
        "        return p + self.rng.gauss(0, v.baro_noise_pa)\n",
        "        qc = getattr(v, \"baro_q_coeff\", 0.0)\n"
        "        if qc != 0.0:\n"
        "            p += qc * 0.5 * 1.225 * airspeed * airspeed\n"
        "        return p + self.rng.gauss(0, v.baro_noise_pa)\n",
    )])

    # ---------------- run.py: instrumentation + oracle hook ----------------
    patch(os.path.join(sim, "run.py"), [(
        "import math\nimport sys\n",
        "import ctypes\nimport math\nimport sys\n",
    ), (
        "            record=True, dud_landing_igniter=False):\n",
        "            record=True, dud_landing_igniter=False, oracle_fire_ms=None):\n",
    ), (
        "    fc.set_mode(core.FlightMode.FULL_LANDING if mode == \"land\"\n"
        "               else core.FlightMode.CHUTE_TEST)\n",
        "    fc.set_mode(core.FlightMode.FULL_LANDING if mode == \"land\"\n"
        "               else core.FlightMode.CHUTE_TEST)\n"
        "    if oracle_fire_ms is not None:\n"
        "        # ANALYSIS ONLY (ws_oracle DLL): force the landing fire command at\n"
        "        # this flight-computer millisecond, bypassing the solver.\n"
        "        fc._lib.rfc_oracle_set_fire_ms.argtypes = [ctypes.c_void_p, ctypes.c_uint32]\n"
        "        fc._lib.rfc_oracle_set_fire_ms(fc._h, int(oracle_fire_ms))\n",
    ), (
        "    legs_on_t = None\n    legs_off_t = None\n",
        "    legs_on_t = None\n    legs_off_t = None\n"
        "    # ---- analysis instrumentation (read-only; never fed back) ----\n"
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
        "    # ---- analysis instrumentation ----\n"
        "    fi = fire_info or {}\n"
        "    summary.update({\n"
        "        \"fire_t\": fi.get(\"fire_t\"), \"fire_h_true\": fi.get(\"fire_h_true\"),\n"
        "        \"fire_v_true\": fi.get(\"fire_v_true\"),\n"
        "        \"fire_kf_alt\": fi.get(\"fire_kf_alt\"),\n"
        "        \"fire_kf_vel\": fi.get(\"fire_kf_vel\"),\n"
        "        \"fire_tilt_deg\": fi.get(\"fire_tilt_deg\"),\n"
        "        \"fire_tilt_prev_deg\": fi.get(\"fire_tilt_prev_deg\"),\n"
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

    if not oracle:
        print("scaffolding applied:", root)
        return

    # ---------------- oracle hook (ws_oracle only) ----------------
    patch(os.path.join(root, "src", "core", "flight_state.h"), [(
        "  FlightState state() const { return state_; }\n",
        "  FlightState state() const { return state_; }\n"
        "  // ANALYSIS-ONLY (ws_oracle copy): force the landing fire command at this\n"
        "  // ms, bypassing the solver, its interlocks and the missed-window abort.\n"
        "  void oracleSetFireMs(uint32_t ms) { oracleFireMs_ = ms; }\n",
    ), (
        "    // Missed the window? (Interlocks never cleared, or table said \"already\n",
        "    if (oracleFireMs_ != 0) {\n"
        "      if (in.ms >= oracleFireMs_) {\n"
        "        out.fireLanding = true;\n"
        "        landFireMs_ = in.ms;\n"
        "        ignitionConfirmed_ = false;\n"
        "        pushEvent(in.ms, FlightEvent::FIRE_LANDING, in.kfAlt);\n"
        "        enter(FlightState::LANDING_BURN, in.ms);\n"
        "        out.tvcActive = true;\n"
        "        out.useLandingGains = true;\n"
        "      }\n"
        "      return;\n"
        "    }\n"
        "\n"
        "    // Missed the window? (Interlocks never cleared, or table said \"already\n",
    ), (
        "  float maxAlt_ = 0;\n",
        "  float maxAlt_ = 0;\n"
        "  uint32_t oracleFireMs_ = 0;\n",
    )])
    patch(os.path.join(root, "tools", "sim", "bridge.cpp"), [(
        "int32_t rfc_pop_event(void* h, CEvent* ev) {\n",
        "void rfc_oracle_set_fire_ms(void* h, uint32_t ms) {\n"
        "  if (h) static_cast<FlightCore*>(h)->fsm().oracleSetFireMs(ms);\n"
        "}\n"
        "\n"
        "int32_t rfc_pop_event(void* h, CEvent* ev) {\n",
    )])
    print("scaffolding + oracle hook applied:", root)


if __name__ == "__main__":
    main()
