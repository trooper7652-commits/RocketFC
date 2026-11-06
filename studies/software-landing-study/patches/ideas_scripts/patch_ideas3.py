"""B3: in-flight wind estimate from the boost's sideways aerodynamic force."""
from patchlib import edit

edit("src/config.h",
"""constexpr bool  WIND_MATCH = false;              // target the wind estimate instead of 0
""",
"""constexpr bool  WIND_MATCH = false;              // target the wind estimate instead of 0
// B3 in-flight wind estimate (needs WIND_MATCH): during BOOST the accelerometer's
// sideways reading, minus the gimbal's share of the thrust, is the airframe's
// normal force = 0.5 rho |V| (CN_alpha A_ref) * crossflow. That gives the air's
// sideways velocity relative to the rocket; plus the dead-reckoned rocket
// velocity, the wind. Samples are weighted by how much force a 1 m/s crossflow
// makes at that airspeed. WIND_EST_SCALE < 1 biases it low on purpose: the sim
// says an underestimate costs little and an overestimate costs a lot.
constexpr bool  WIND_EST = false;
constexpr float AERO_CNA_AREF_M2 = 0.00884f;     // [ESTIMATE] CN_alpha (2/rad) * A_ref (75 mm tube)
constexpr float WIND_EST_SCALE = 0.8f;
constexpr float WIND_EST_MIN_SPEED = 4.0f;       // m/s airspeed before samples count
""")
edit("src/core/flight_core.h",
"""      if (!fo.useLandingGains && cfg::BOOST_DRIFT_GAIN > 0.0f &&
          fo.state == FlightState::BOOST) {
""",
"""      if (cfg::WIND_EST && fo.state == FlightState::BOOST) windEstStep(in);
      if (!fo.useLandingGains && cfg::BOOST_DRIFT_GAIN > 0.0f &&
          fo.state == FlightState::BOOST) {
""")
edit("src/core/flight_core.h",
""" private:
  Ahrs ahrs_;
""",
""" private:
  // B3: one boost sample of the aerodynamic wind estimate (see config.h).
  void windEstStep(const CoreInput& in) {
    const float az = in.accel.z;                    // ~thrust / mass
    if (az < 3.0f) return;
    // Sideways specific force left after removing the gimbal's thrust share
    // (body: fx = -T sin(gimbalY), fy = +T sin(gimbalX)), i.e. aero normal force / m.
    const float nx = in.accel.x + az * std::sin(ctl_.gimbalY());
    const float ny = in.accel.y - az * std::sin(ctl_.gimbalX());
    const float vUp = kf_.velocity();
    const float dwx = velHx_ - windX_, dwy = velHy_ - windY_;
    const float speed = std::sqrt(vUp * vUp + dwx * dwx + dwy * dwy);
    if (speed < cfg::WIND_EST_MIN_SPEED) return;
    const float k = 0.5f * 1.225f * cfg::AERO_CNA_AREF_M2 / cfg::MASS_DESCENT_KG * speed;
    // Body-frame air-relative velocity of the rocket: n = -k * v_t.
    const float vtx = -nx / k, vty = -ny / k;
    float vax2 = speed * speed - vtx * vtx - vty * vty;
    const Vec3 vrelW = ahrs_.quat().rotate(Vec3{vtx, vty, vax2 > 0.0f ? std::sqrt(vax2) : 0.0f});
    const float wx = velHx_ - vrelW.x, wy = velHy_ - vrelW.y;
    const float wgt = k * k;
    windSx_ += wgt * wx;
    windSy_ += wgt * wy;
    windSw_ += wgt;
    windX_ = cfg::WIND_EST_SCALE * windSx_ / windSw_;
    windY_ = cfg::WIND_EST_SCALE * windSy_ / windSw_;
  }

  Ahrs ahrs_;
""")
edit("src/core/flight_core.h",
"""  float windX_ = 0.0f, windY_ = 0.0f;
""",
"""  float windX_ = 0.0f, windY_ = 0.0f;
  float windSx_ = 0.0f, windSy_ = 0.0f, windSw_ = 0.0f;

 public:
  float windEstX() const { return windX_; }
  float windEstY() const { return windY_; }

 private:
""")
edit("tools/sim/bridge.cpp",
"""void rfc_wind_est(void* h, float wx, float wy) {
  static_cast<FlightCore*>(h)->windEstimate(wx, wy);
}
""",
"""void rfc_wind_est(void* h, float wx, float wy) {
  static_cast<FlightCore*>(h)->windEstimate(wx, wy);
}

float rfc_wind_est_x(void* h) { return static_cast<FlightCore*>(h)->windEstX(); }
float rfc_wind_est_y(void* h) { return static_cast<FlightCore*>(h)->windEstY(); }
""")
edit("tools/sim/core.py",
"""    lib.rfc_wind_est.restype = None
""",
"""    lib.rfc_wind_est.restype = None
    lib.rfc_wind_est_x.argtypes = [ctypes.c_void_p]
    lib.rfc_wind_est_x.restype = ctypes.c_float
    lib.rfc_wind_est_y.argtypes = [ctypes.c_void_p]
    lib.rfc_wind_est_y.restype = ctypes.c_float
""")
edit("tools/sim/core.py",
"""    def wind_est(self, wx, wy):
        self._lib.rfc_wind_est(self._h, wx, wy)
""",
"""    def wind_est(self, wx, wy):
        self._lib.rfc_wind_est(self._h, wx, wy)

    def wind_est_xy(self):
        return self._lib.rfc_wind_est_x(self._h), self._lib.rfc_wind_est_y(self._h)
""")
# diagnostics: the firmware's wind estimate at burnout vs the truth
edit("tools/sim/run.py",
"""                bo_diag = {"bo_hs_air": math.hypot(s.vel.x - _w.x, s.vel.y - _w.y),
""",
"""                _we = fc.wind_est_xy()
                bo_diag = {"bo_hs_air": math.hypot(s.vel.x - _w.x, s.vel.y - _w.y),
                           "bo_wind_est": [_we[0], _we[1]], "bo_wind_true": [_w.x, _w.y],
""")
print("done")
