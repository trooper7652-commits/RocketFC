"""Add the study's code candidates to the candidate workspace's firmware COPY.
Every candidate is behind a config constant whose default is OFF, so the
defaults must reproduce today's flights exactly."""
import sys

root = sys.argv[1]


def patch(rel, pairs):
    p = root + "/" + rel
    s = open(p, encoding="utf-8").read()
    for a, b in pairs:
        assert s.count(a) == 1, (rel, a[:70])
        s = s.replace(a, b)
    open(p, "w", encoding="utf-8").write(s)


patch("src/config.h", [(
    "constexpr float TILT_IGNITE_HARD_MAX_DEG = 25.0f;\n",
    """constexpr float TILT_IGNITE_HARD_MAX_DEG = 25.0f;

// ---- Software-landing study candidates (all OFF = today's behaviour) ----
// S3 drift cancelling: during the landing burn, lean the TVC target against
// the sideways velocity dead-reckoned from launch (world-frame accel).
constexpr float DRIFT_GAIN = 0.0f;           // 1/s; 0 = off
constexpr float DRIFT_MAX_TILT_DEG = 5.0f;   // cap on the commanded lean
constexpr float DRIFT_START_S = 0.0f;        // S3 option: only lean after this long into the burn
// S4b coast grace: no tilt abort in COAST/APOGEE (TVC is off there anyway),
// nor for this long into DESCENT, so the airframe can weathervane tail-first.
constexpr bool  COAST_TILT_GRACE = false;
constexpr float DESCENT_TILT_GRACE_MS = 0.0f;
// S6: centre the gimbal once |accel| exceeds this after ignition (ground
// impact), instead of steering into the ground until touchdown is debounced.
constexpr float IMPACT_CENTER_G = 0.0f;      // 0 = off
""")])

patch("src/core/ahrs.h", [(
    """  // Vertical (world-Z) component of specific force, for the altitude KF:""",
    """  // Tilt error toward an arbitrary world-frame target direction (unit
  // vector), same convention as tiltErrorBody() (which is target = up).
  Vec3 tiltErrorBodyTo(const Vec3& target) const {
    const Vec3 zbWorld = q_.rotate(Vec3{0, 0, 1});
    const Vec3 axis = zbWorld.cross(target);
    const float s = axis.norm();
    if (s < 1e-6f) return {0, 0, 0};
    const float ang = std::atan2(s, zbWorld.x * target.x + zbWorld.y * target.y +
                                        zbWorld.z * target.z);
    return q_.rotateInv(axis * (ang / s));
  }

  // Vertical (world-Z) component of specific force, for the altitude KF:""")])

patch("src/core/flight_core.h", [
    ("""    // --- altitude / velocity estimation ---
    kf_.predict(ahrs_.worldVerticalAccel(in.accel, cfg::G0), in.dt);""",
     """    // --- altitude / velocity estimation ---
    kf_.predict(ahrs_.worldVerticalAccel(in.accel, cfg::G0), in.dt);
    // Sideways velocity, dead-reckoned from launch: gravity is vertical, so
    // the world-frame horizontal specific force IS the horizontal accel.
    if (onPad) {
      velHx_ = velHy_ = 0.0f;
    } else {
      const Vec3 aw = ahrs_.quat().rotate(in.accel);
      velHx_ += aw.x * in.dt;
      velHy_ += aw.y * in.dt;
    }"""),
    ("""      const Vec3 e = ahrs_.tiltErrorBody();
      ctl_.update(e.x, e.y, in.gyro.x, in.gyro.y, in.dt, true);""",
     """      Vec3 e = ahrs_.tiltErrorBody();
      if (fo.useLandingGains) {
        if (!wasLanding_) burnMs_ = in.ms;
        const float aT = in.accel.norm();   // ~thrust/mass while burning
        if (cfg::DRIFT_GAIN > 0.0f && aT > 3.0f &&
            (in.ms - burnMs_) * 0.001f >= cfg::DRIFT_START_S) {
          // Want horizontal accel = -K v: lean the thrust axis by -K v / aT.
          float hx = -cfg::DRIFT_GAIN * velHx_ / aT;
          float hy = -cfg::DRIFT_GAIN * velHy_ / aT;
          const float hn = std::sqrt(hx * hx + hy * hy);
          const float cap = std::tan(cfg::DRIFT_MAX_TILT_DEG * cfg::DEG2RAD);
          if (hn > cap) { hx *= cap / hn; hy *= cap / hn; }
          const float n = std::sqrt(hx * hx + hy * hy + 1.0f);
          e = ahrs_.tiltErrorBodyTo(Vec3{hx / n, hy / n, 1.0f / n});
        }
      }
      wasLanding_ = fo.useLandingGains;
      ctl_.update(e.x, e.y, in.gyro.x, in.gyro.y, in.dt, true);"""),
    ("""  bool wasActive_ = false;
};""",
     """  bool wasActive_ = false;
  bool wasLanding_ = false;
  uint32_t burnMs_ = 0;
  float velHx_ = 0.0f, velHy_ = 0.0f;
};"""),
])

patch("src/core/flight_state.h", [
    ("""    AbortReason r = AbortReason::NONE;
    if (dbTilt_.check(in.tiltRad > cfg::TILT_ABORT_DEG * cfg::DEG2RAD,
                      in.dtMs, cfg::TILT_ABORT_MS))""",
     """    AbortReason r = AbortReason::NONE;
    const bool tiltGrace =
        cfg::COAST_TILT_GRACE &&
        (state_ == FlightState::COAST || state_ == FlightState::APOGEE ||
         (state_ == FlightState::DESCENT &&
          sinceEntry(in) < cfg::DESCENT_TILT_GRACE_MS));
    if (!tiltGrace &&
        dbTilt_.check(in.tiltRad > cfg::TILT_ABORT_DEG * cfg::DEG2RAD,
                      in.dtMs, cfg::TILT_ABORT_MS))"""),
    ("""    // Leg release: LEGS_DELAY_MS after the fire command (= state entry), and""",
     """    // S6: the ground stops the rocket with a spike far above the burn's
    // ~1.6 g -- stop steering the (still burning) motor into the ground.
    if (cfg::IMPACT_CENTER_G > 0.0f && ignitionConfirmed_ &&
        sinceEntry(in) > cfg::IGNITION_DELAY_MS + 300.0f &&
        in.accelNormG > cfg::IMPACT_CENTER_G) {
      impactSeen_ = true;
    }
    if (impactSeen_) out.tvcActive = false;

    // Leg release: LEGS_DELAY_MS after the fire command (= state entry), and"""),
    ("""  LandingSolver solver_;""", """  LandingSolver solver_;
  bool impactSeen_ = false;"""),
])
print("patched", root)
