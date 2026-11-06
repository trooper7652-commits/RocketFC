"""Apply the new-ideas candidates to the sw/ideas workspace (all OFF by default).
Each edit must match exactly once; files keep their CRLF line endings."""
import os
import sys

SCR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WS = os.path.join(SCR, "sw", "ideas")
from patchlib import edit  # noqa: E402,F401

# ---------------- firmware: config.h ----------------
edit("src/config.h",
"""constexpr float IMPACT_CENTER_G = 0.0f;      // 0 = off
""",
"""constexpr float IMPACT_CENTER_G = 0.0f;      // 0 = off

// ---- New-ideas study (2026-10-19), all OFF = today's behaviour ----
// B1 boost drift nulling: during BOOST, lean the TVC target against the
// dead-reckoned sideways velocity (relative to the wind estimate when
// WIND_MATCH), so the rocket reaches burnout moving with the air -- no
// crossflow in the unpowered coast for the fin-less airframe to diverge on.
constexpr float BOOST_DRIFT_GAIN = 0.0f;         // 1/s; 0 = off
constexpr float BOOST_DRIFT_MAX_TILT_DEG = 5.0f; // cap on the commanded lean
constexpr float BOOST_DRIFT_START_S = 0.3f;      // lean only after this long into BOOST
constexpr float BOOST_DRIFT_END_S = 0.6f;        // ...and stand upright this long before ASCENT_BURN_MS
constexpr bool  WIND_MATCH = false;              // target the wind estimate instead of 0
""")

# ---------------- firmware: flight_core.h ----------------
edit("src/core/flight_core.h",
"""  void step(const CoreInput& in, CoreOutput& out) {
""",
"""  // Horizontal wind estimate (m/s, world x/y) for WIND_MATCH. The sim can feed
  // the truth here (an oracle upper bound) or an estimator can set it.
  void windEstimate(float wx, float wy) { windX_ = wx; windY_ = wy; }

  void step(const CoreInput& in, CoreOutput& out) {
""")
edit("src/core/flight_core.h",
"""      wasLanding_ = fo.useLandingGains;
      ctl_.update(e.x, e.y, in.gyro.x, in.gyro.y, in.dt, true);
""",
"""      if (!fo.useLandingGains && cfg::BOOST_DRIFT_GAIN > 0.0f &&
          fo.state == FlightState::BOOST) {
        // B1: arrive at burnout moving with the air (or at rest, without
        // WIND_MATCH), upright for the last BOOST_DRIFT_END_S.
        if (!wasActive_) boostMs_ = in.ms;
        const float tb = (in.ms - boostMs_) * 0.001f;
        const float aT = in.accel.norm();
        if (aT > 3.0f && tb >= cfg::BOOST_DRIFT_START_S &&
            tb <= cfg::ASCENT_BURN_MS * 0.001f - cfg::BOOST_DRIFT_END_S) {
          const float tx = cfg::WIND_MATCH ? windX_ : 0.0f;
          const float ty = cfg::WIND_MATCH ? windY_ : 0.0f;
          float hx = -cfg::BOOST_DRIFT_GAIN * (velHx_ - tx) / aT;
          float hy = -cfg::BOOST_DRIFT_GAIN * (velHy_ - ty) / aT;
          const float hn = std::sqrt(hx * hx + hy * hy);
          const float cap = std::tan(cfg::BOOST_DRIFT_MAX_TILT_DEG * cfg::DEG2RAD);
          if (hn > cap) { hx *= cap / hn; hy *= cap / hn; }
          const float n = std::sqrt(hx * hx + hy * hy + 1.0f);
          e = ahrs_.tiltErrorBodyTo(Vec3{hx / n, hy / n, 1.0f / n});
        }
      }
      wasLanding_ = fo.useLandingGains;
      ctl_.update(e.x, e.y, in.gyro.x, in.gyro.y, in.dt, true);
""")
edit("src/core/flight_core.h",
"""  uint32_t burnMs_ = 0;
""",
"""  uint32_t burnMs_ = 0;
  uint32_t boostMs_ = 0;
  float windX_ = 0.0f, windY_ = 0.0f;
""")

# ---------------- bridge + ctypes ----------------
edit("tools/sim/bridge.cpp",
"""void rfc_gps_vel(void* h, float vx, float vy) {
  static_cast<FlightCore*>(h)->gpsVelocity(vx, vy);
}
""",
"""void rfc_gps_vel(void* h, float vx, float vy) {
  static_cast<FlightCore*>(h)->gpsVelocity(vx, vy);
}

void rfc_wind_est(void* h, float wx, float wy) {
  static_cast<FlightCore*>(h)->windEstimate(wx, wy);
}
""")
edit("tools/sim/core.py",
"""    lib.rfc_gps_vel.restype = None
""",
"""    lib.rfc_gps_vel.restype = None
    lib.rfc_wind_est.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float]
    lib.rfc_wind_est.restype = None
""")
edit("tools/sim/core.py",
"""    def gps_vel(self, vx, vy):
        self._lib.rfc_gps_vel(self._h, vx, vy)
""",
"""    def gps_vel(self, vx, vy):
        self._lib.rfc_gps_vel(self._h, vx, vy)

    def wind_est(self, wx, wy):
        self._lib.rfc_wind_est(self._h, wx, wy)
""")

# ---------------- plant / vehicle / run ----------------
edit("tools/sim/vehicle.py",
"""    gps_latency_s: float = 0.1         # [ESTIMATE] measurement age when it arrives (s)
""",
"""    gps_latency_s: float = 0.1         # [ESTIMATE] measurement age when it arrives (s)

    # ---- New-ideas study switches (defaults = earlier behaviour) ----
    pad_hold: bool = False             # pad cradle/clamp: no rotation before ascent ignition
    wind_oracle: bool = False          # feed the TRUE wind to the firmware (WIND_MATCH bound)
""")
edit("tools/sim/plant.py",
"""        alpha = torque_body * (1.0 / max(inertia, 1e-6))
        s.omega = s.omega + alpha * dt
        s.q.integrate(s.omega, dt)

        s.t += dt
""",
"""        alpha = torque_body * (1.0 / max(inertia, 1e-6))
        if getattr(v, "pad_hold", False) and s.ascent_ignite_t is None:
            s.omega = V3(0.0, 0.0, 0.0)  # held in the pad cradle until ignition
        else:
            s.omega = s.omega + alpha * dt
            s.q.integrate(s.omega, dt)

        s.t += dt
""")
edit("tools/sim/run.py",
"""            gyro_corrected = (gx - gbx, gy - gby, gz - gbz)
            ms = int(t * 1000.0 + 0.5)
""",
"""            gyro_corrected = (gx - gbx, gy - gby, gz - gbz)
            ms = int(t * 1000.0 + 0.5)
            if getattr(vcfg, "wind_oracle", False):
                _w = pl._wind(t)
                fc.wind_est(_w.x, _w.y)
""")
# read-only diagnostics: air-relative sideways speed at ascent burnout, true tilt at apogee
edit("tools/sim/run.py",
"""            prev_v_true = s.vel.z
            prev_tilt = out["tilt_deg"]
""",
"""            if bo_diag is None and pl.state.ascent_burnout_t is not None:
                _w = pl._wind(t)
                _zb = s.q.rotate(plant_mod.V3(0.0, 0.0, 1.0))
                bo_diag = {"bo_hs_air": math.hypot(s.vel.x - _w.x, s.vel.y - _w.y),
                           "bo_hs": math.hypot(s.vel.x, s.vel.y),
                           "bo_tilt_true": math.degrees(math.acos(max(-1.0, min(1.0, _zb.z)))),
                           "bo_rate_dps": math.degrees(math.hypot(s.omega.x, s.omega.y))}
            if ap_diag is None and pl.state.apogee_t is not None:
                _zb = s.q.rotate(plant_mod.V3(0.0, 0.0, 1.0))
                ap_diag = {"ap_tilt_true": math.degrees(math.acos(max(-1.0, min(1.0, _zb.z))))}
            prev_v_true = s.vel.z
            prev_tilt = out["tilt_deg"]
""")
edit("tools/sim/run.py",
"""    prev_v_true = None
""",
"""    prev_v_true = None
    bo_diag = None
    ap_diag = None
""")
edit("tools/sim/run.py",
"""    # ---- landing diagnostics ----
    fi = fire_info or {}
""",
"""    summary.update(bo_diag or {})
    summary.update(ap_diag or {})
    # ---- landing diagnostics ----
    fi = fire_info or {}
""")
print("done")
