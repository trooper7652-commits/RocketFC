"""STUDY-ONLY firmware switches for scratch copies (never ported). Every switch
is a constexpr in the copy's src/config.h, OFF by default, so the all-off build
behaves exactly like the unmodified tree and --fw-overrides can sweep them.

  STUDY_WP_D_M   (float, 0 = off)  T5 "window passed" abort
  STUDY_DUD_GUARD (int, 1 = on)    T6 re-enable TVC if a dud igniter lights late
  STUDY_D2       (int, 1 = on)     T7a predicted-tilt ignition interlock
  STUDY_WMA      (int, 1 = on)     T7b wind-matched ascent (wind estimate + boost lean)
plus plant knob VehicleConfig.gimbal_offset_deg (tools/sim/vehicle.py, run.py).

usage: python patch_study.py <copy dir>      (see patch_study_NOTES.md)
"""
import os, sys


def sub(p, old, new):
    t = open(p, encoding="utf-8", newline="").read()
    assert t.count(old) == 1, (p, old[:70], t.count(old))
    open(p, "w", encoding="utf-8", newline="").write(t.replace(old, new))


root = sys.argv[1]
cfgh = os.path.join(root, "src", "config.h")
fsm = os.path.join(root, "src", "core", "flight_state.h")
core = os.path.join(root, "src", "core", "flight_core.h")
veh = os.path.join(root, "tools", "sim", "vehicle.py")
run = os.path.join(root, "tools", "sim", "run.py")

# ---------------------------------------------------------------- config.h
sub(cfgh, "constexpr float DRIFT_MAX_TILT_DEG = 5.0f;   // cap on the commanded lean\n",
    "constexpr float DRIFT_MAX_TILT_DEG = 5.0f;   // cap on the commanded lean\n"
    """
// ---- STUDY switches (scratch copy only; all OFF = unmodified behaviour) ----
// T5: in DESCENT, abort (MISSED_WINDOW) once the Kalman altitude is more than
// this many metres below the table's ignition altitude while the interlocks
// still block the fire. 0 = off.
constexpr float STUDY_WP_D_M = 0.0f;
// T6: after a DUD_IGNITER abort, if real thrust shows up later (accel above
// IGNITE_CONFIRM_G, above TOUCHDOWN_ALT_M) hold the TVC (landing gains) until
// burnout / touchdown. The chute release is unchanged. 1 = on.
constexpr int   STUDY_DUD_GUARD = 0;
constexpr float STUDY_DUD_GUARD_WIN_MS = 250.0f;  // thrust latch only within this long after the DUD abort
constexpr float STUDY_DUD_CONFIRM_MS = 20.0f;     // accel must stay above IGNITE_CONFIRM_G this long
// T7a: D2 predicted-tilt interlock (software-landing study S4a): also allow
// the fire when tilt >= TILT_IGNITE_MAX_DEG if the tilt predicted at thrust
// onset (tilt + filtered tilt rate * IGNITION_DELAY_MS) is below it and the
// tilt now is below the hard cap. 1 = on.
constexpr int   STUDY_D2 = 0;
constexpr float STUDY_D2_HARD_MAX_DEG = 25.0f;
constexpr float STUDY_D2_RATE_LPF_HZ = 5.0f;    // tilt-rate low-pass
constexpr int   STUDY_D2_WARMUP_TICKS = 25;     // ticks before the rate is trusted
// T7b: wind-matched ascent (new-ideas-2026-10-19 s4): in-flight wind estimate
// during BOOST + boost lean toward it, upright for the last END_S. 1 = on.
constexpr int   STUDY_WMA = 0;
constexpr float STUDY_WMA_GAIN = 1.0f;          // 1/s boost drift gain
constexpr float STUDY_WMA_MAX_TILT_DEG = 5.0f;
constexpr float STUDY_WMA_START_S = 0.3f;
constexpr float STUDY_WMA_END_S = 0.6f;
constexpr float STUDY_WMA_CNA_AREF_M2 = 0.00884f;
constexpr float STUDY_WMA_SCALE = 0.8f;
constexpr float STUDY_WMA_MIN_SPEED = 4.0f;
constexpr float STUDY_WMA_PRIOR_MPS = 1.0f;
""")

# ---------------------------------------------------------- flight_state.h
# D2: reset on entering DESCENT
sub(fsm, "    } else {\n      enter(FlightState::DESCENT, in.ms);\n",
    "    } else {\n      d2Reset();  // STUDY\n      enter(FlightState::DESCENT, in.ms);\n")

# D2 rate update (every DESCENT tick) + D2 interlock + T5 rule
sub(fsm, "  void updateDescent(const FsmInput& in, FsmOutput& out) {\n    if (checkAborts(in, out)) return;\n",
    "  void updateDescent(const FsmInput& in, FsmOutput& out) {\n"
    "    if (cfg::STUDY_D2 != 0) d2UpdateRate(in);  // STUDY\n"
    "    if (checkAborts(in, out)) return;\n")

sub(fsm, """    const bool interlocks =
        in.tiltRad < cfg::TILT_IGNITE_MAX_DEG * cfg::DEG2RAD &&
        in.kfHealthy && in.contLanding;
    if (windowOpen && interlocks) {""",
    """    bool tiltOk = in.tiltRad < cfg::TILT_IGNITE_MAX_DEG * cfg::DEG2RAD;
    if (!tiltOk && cfg::STUDY_D2 != 0 && d2Valid_) {  // STUDY T7a
      const float tiltOn = in.tiltRad + d2Rate_ * (cfg::IGNITION_DELAY_MS * 0.001f);
      tiltOk = tiltOn < cfg::TILT_IGNITE_MAX_DEG * cfg::DEG2RAD &&
               in.tiltRad < cfg::STUDY_D2_HARD_MAX_DEG * cfg::DEG2RAD;
    }
    const bool interlocks = tiltOk && in.kfHealthy && in.contLanding;
    if (windowOpen && interlocks) {""")

sub(fsm, """      out.tvcActive = true;
      out.useLandingGains = true;
    }
  }

  void updateLandingBurn(""",
    """      out.tvcActive = true;
      out.useLandingGains = true;
    } else if (cfg::STUDY_WP_D_M > 0.0f && !interlocks &&  // STUDY T5
               in.kfAlt < hIgn - cfg::STUDY_WP_D_M) {
      doAbort(in, out, AbortReason::MISSED_WINDOW);
    }
  }

  // STUDY T7a: first-order low-passed derivative of the tilt angle.
  void d2Reset() {
    d2Prev_ = 0.0f; d2Rate_ = 0.0f;
    d2HavePrev_ = false; d2Valid_ = false; d2N_ = 0;
  }
  void d2UpdateRate(const FsmInput& in) {
    const float dt = in.dtMs * 0.001f;
    if (!std::isfinite(in.tiltRad) || !(dt > 0.0f)) return;
    if (d2HavePrev_) {
      const float raw = (in.tiltRad - d2Prev_) / dt;
      if (d2N_ == 0) {
        d2Rate_ = raw;
      } else {
        const float rc = 1.0f / (2.0f * cfg::PI_F * cfg::STUDY_D2_RATE_LPF_HZ);
        d2Rate_ += dt / (rc + dt) * (raw - d2Rate_);
      }
      if (d2N_ < cfg::STUDY_D2_WARMUP_TICKS) ++d2N_;
      else d2Valid_ = true;
    }
    d2Prev_ = in.tiltRad;
    d2HavePrev_ = true;
  }

  void updateLandingBurn(""")

# T6 dud guard in updateDescentChute
sub(fsm, """      enter(FlightState::TOUCHDOWN, in.ms);
    }
    (void)out;
  }

  void updateAbort(""",
    """      enter(FlightState::TOUCHDOWN, in.ms);
    }
    // STUDY T6: a DUD_IGNITER abort whose motor lights late. The chute path is
    // untouched; this only adds TVC (landing gains) while real thrust lasts.
    if (cfg::STUDY_DUD_GUARD != 0 && state_ == FlightState::DESCENT_CHUTE &&
        abortReason_ == AbortReason::DUD_IGNITER && !dudBurnedOut_) {
      if (!dudThrust_ &&
          (float)(in.ms - dudAbortMs_) <= cfg::STUDY_DUD_GUARD_WIN_MS &&
          dbDudOn_.check(in.accelNormG > cfg::IGNITE_CONFIRM_G &&
                             in.kfAlt > cfg::TOUCHDOWN_ALT_M,
                         in.dtMs, cfg::STUDY_DUD_CONFIRM_MS)) {
        dudThrust_ = true;
        dbDud_.reset();
      }
      if (dudThrust_) {
        if (dbDud_.check(in.accelNormG < cfg::BURNOUT_ACCEL_G, in.dtMs,
                         cfg::BURNOUT_MS)) {
          dudBurnedOut_ = true;
        } else {
          out.tvcActive = true;
          out.useLandingGains = true;
        }
      }
    }
    (void)out;
  }

  void updateAbort(""")

sub(fsm, "    abortReason_ = r;\n", "    abortReason_ = r;\n    dudAbortMs_ = in.ms;  // STUDY (only used for DUD_IGNITER)\n")
sub(fsm, "  ChuteDeploy chute_;\n",
    "  ChuteDeploy chute_;\n"
    "  // STUDY members (T6 dud guard, T7a D2)\n"
    "  Debounce dbDud_, dbDudOn_;\n"
    "  uint32_t dudAbortMs_ = 0;\n"
    "  bool dudThrust_ = false, dudBurnedOut_ = false;\n"
    "  float d2Prev_ = 0.0f, d2Rate_ = 0.0f;\n"
    "  bool d2HavePrev_ = false, d2Valid_ = false;\n"
    "  int d2N_ = 0;\n")

# ------------------------------------------------------------ flight_core.h
sub(core, "      velHx_ = velHy_ = 0.0f;\n",
    "      velHx_ = velHy_ = 0.0f;\n"
    "      windX_ = windY_ = windSx_ = windSy_ = windSw_ = 0.0f;  // STUDY\n"
    "      windPnx_ = windPny_ = windNv_ = 0.0f; windNn_ = 0;\n")

sub(core, "      ctl_.update(e.x, e.y, in.gyro.x, in.gyro.y, in.dt, true);\n",
    """      if (cfg::STUDY_WMA != 0 && fo.state == FlightState::BOOST) {  // STUDY T7b
        windEstStep(in);
        // arrive at burnout moving with the air, upright for the last END_S
        if (!wasActive_) boostMs_ = in.ms;
        const float tb = (in.ms - boostMs_) * 0.001f;
        const float aT = in.accel.norm();
        if (aT > 3.0f && tb >= cfg::STUDY_WMA_START_S &&
            tb <= cfg::ASCENT_BURN_MS * 0.001f - cfg::STUDY_WMA_END_S) {
          float hx = -cfg::STUDY_WMA_GAIN * (velHx_ - windX_) / aT;
          float hy = -cfg::STUDY_WMA_GAIN * (velHy_ - windY_) / aT;
          const float hn = std::sqrt(hx * hx + hy * hy);
          const float cap = std::tan(cfg::STUDY_WMA_MAX_TILT_DEG * cfg::DEG2RAD);
          if (hn > cap) { hx *= cap / hn; hy *= cap / hn; }
          const float n = std::sqrt(hx * hx + hy * hy + 1.0f);
          e = ahrs_.tiltErrorBodyTo(Vec3{hx / n, hy / n, 1.0f / n});
        }
      }
      ctl_.update(e.x, e.y, in.gyro.x, in.gyro.y, in.dt, true);
""")

sub(core, " private:\n  Ahrs ahrs_;\n",
    """ private:
  // STUDY T7b: one boost sample of the aerodynamic wind estimate. Sideways
  // specific force minus the gimbal's thrust share = airframe normal force / m
  // = -k * (air-relative sideways velocity); plus the dead-reckoned rocket
  // velocity gives the wind. Samples weighted by k^2, shrunk toward 0 by the
  // measured noise (prior), scaled low on purpose.
  void windEstStep(const CoreInput& in) {
    const float az = in.accel.z;                    // ~thrust / mass
    if (az < 3.0f) return;
    const float nx = in.accel.x + az * std::sin(ctl_.gimbalY());
    const float ny = in.accel.y - az * std::sin(ctl_.gimbalX());
    const float vUp = kf_.velocity();
    const float dwx = velHx_ - windX_, dwy = velHy_ - windY_;
    const float speed = std::sqrt(vUp * vUp + dwx * dwx + dwy * dwy);
    if (speed < cfg::STUDY_WMA_MIN_SPEED) return;
    const float k = 0.5f * 1.225f * cfg::STUDY_WMA_CNA_AREF_M2 / cfg::MASS_DESCENT_KG * speed;
    const float vtx = -nx / k, vty = -ny / k;
    const float vax2 = speed * speed - vtx * vtx - vty * vty;
    const Vec3 vrelW = ahrs_.quat().rotate(Vec3{vtx, vty, vax2 > 0.0f ? std::sqrt(vax2) : 0.0f});
    const float wx = velHx_ - vrelW.x, wy = velHy_ - vrelW.y;
    const float wgt = k * k;
    windSx_ += wgt * wx;
    windSy_ += wgt * wy;
    windSw_ += wgt;
    float shrink = 1.0f;
    if (cfg::STUDY_WMA_PRIOR_MPS > 0.0f) {
      if (windNn_ > 0) {
        const float ddx = nx - windPnx_, ddy = ny - windPny_;
        windNv_ += 0.25f * (ddx * ddx + ddy * ddy);
        ++windNn_;
      } else {
        windNn_ = 1;
      }
      windPnx_ = nx;
      windPny_ = ny;
      const float nv = windNn_ > 1 ? windNv_ / (float)(windNn_ - 1) : 1.0f;
      const float pv = cfg::STUDY_WMA_PRIOR_MPS * cfg::STUDY_WMA_PRIOR_MPS;
      shrink = windSw_ / (windSw_ + nv / pv);
    }
    windX_ = cfg::STUDY_WMA_SCALE * shrink * windSx_ / windSw_;
    windY_ = cfg::STUDY_WMA_SCALE * shrink * windSy_ / windSw_;
  }

  uint32_t boostMs_ = 0;
  float windX_ = 0.0f, windY_ = 0.0f;
  float windSx_ = 0.0f, windSy_ = 0.0f, windSw_ = 0.0f;
  float windPnx_ = 0.0f, windPny_ = 0.0f, windNv_ = 0.0f;
  int windNn_ = 0;

  Ahrs ahrs_;
""")

# -------------------------------------------------------- plant knob (sim)
sub(veh, "    cda_m2: float = 0.0040  ",
    "    gimbal_offset_deg: float = 0.0     # STUDY: gimbal zero error (deg) added to the ACTUAL x angle\n"
    "    cda_m2: float = 0.0040  ")
sub(run, "            act_x, act_y = sv.step(true_cmd_x, true_cmd_y)\n",
    "            act_x, act_y = sv.step(true_cmd_x, true_cmd_y)\n"
    "            act_x += math.radians(vcfg.gimbal_offset_deg)  # STUDY: gimbal zero error\n")
print("patched study switches into", root)
