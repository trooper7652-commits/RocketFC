#pragma once
//
// Landing-burn ignition solver: decides WHEN to send the F15 fire command.
//
// The F15 can't throttle or restart, so the fire time is the only control.
// cfg::LANDING_SOLVER selects how it is chosen:
//
//   TABLE -- the original rule: fire when the Kalman altitude is at or below
//            burn_table.h's h_ignite(descent speed) + BURN_TABLE_MARGIN_M.
//
//   RISK  -- the same physics with three corrections:
//     1. Lag compensation. The Kalman filter follows a barometer that reports
//        where the rocket WAS (bay venting + filtering), so during the fall it
//        reads high and slow. Predict it forward by LAG_TAU_S using the
//        freefall acceleration (gravity minus drag).
//     2. Risk-aware aim. landing_table.h's fire altitudes were chosen offline
//        (tools/landing_table.py) to minimize the EXPECTED touchdown speed
//        over the delay / mass / thrust uncertainty, counting a hop (stopping
//        in mid-air and falling back) at its real fall-back speed, instead of
//        aiming a nominal rocket at exactly zero.
//     3. Tilt loss (LANDING_TILT_COMP). Thrust acts along the body, so a tilted
//        ignition slows the fall less until the TVC straightens the rocket:
//        fire higher by TILT_COEF(v) * tilt^2 (tilt predicted at thrust onset).
//
// Optional, OFF by default: TILT_PREDICT_INTERLOCK also allows a fire while
// the tilt is above TILT_IGNITE_MAX_DEG if it is falling fast enough to be
// below it when thrust actually starts (IGNITION_DELAY_MS later) and is under
// TILT_IGNITE_HARD_MAX_DEG now. It never blocks a fire the original
// interlock would allow.
//
// Deterministic, allocation-free, a few dozen flops per call.
//
#include <cmath>

#include "../config.h"
#include "burn_table.h"
#include "landing_table.h"

class LandingSolver {
 public:
  // Defaults come from config.h; the replay tests override them.
  struct Options {
    cfg::LandingSolverMode mode = cfg::LANDING_SOLVER;
    float lagTauS = cfg::LAG_TAU_S;
    bool tiltComp = cfg::LANDING_TILT_COMP;
    bool predictInterlock = cfg::TILT_PREDICT_INTERLOCK;
    float hardMaxDeg = cfg::TILT_IGNITE_HARD_MAX_DEG;
  };

  LandingSolver() = default;
  explicit LandingSolver(const Options& o) : opt_(o) {}

  void reset() { *this = LandingSolver{opt_}; }

  // Call EVERY tick of DESCENT (the tilt-rate filter needs continuity).
  // Returns true when the fire command should be issued now.
  bool update(float kfAlt, float kfVel, float tiltRad, float dtMs,
              bool kfHealthy, bool contLanding) {
    updateTiltRate(tiltRad, dtMs);
    if (opt_.mode == cfg::LandingSolverMode::TABLE)
      return tableRule(kfAlt, kfVel, tiltRad, kfHealthy, contLanding);
    return riskRule(kfAlt, kfVel, tiltRad, kfHealthy, contLanding);
  }

  // Diagnostics of the last call (for logs and tests).
  float hNow() const { return hNow_; }
  float hReq() const { return hReq_; }
  float vDown() const { return vDown_; }
  float tiltOnRad() const { return tiltOn_; }
  float tiltRateRadS() const { return tiltRate_; }

 private:
  bool tableRule(float kfAlt, float kfVel, float tiltRad, bool kfHealthy,
                 bool contLanding) {
    const float vDown = -kfVel;
    hNow_ = kfAlt;
    vDown_ = vDown;
    tiltOn_ = tiltRad;
    if (!std::isfinite(kfAlt) || !std::isfinite(vDown)) return false;
    if (vDown < cfg::BURN_MIN_SPEED_MS) return false;
    const float hIgn = burntable::hIgnite(vDown) + cfg::BURN_TABLE_MARGIN_M;
    hReq_ = hIgn;
    const bool windowOpen = kfAlt <= hIgn;
    const bool interlocks =
        tiltRad < cfg::TILT_IGNITE_MAX_DEG * cfg::DEG2RAD && kfHealthy &&
        contLanding;
    return windowOpen && interlocks;
  }

  bool riskRule(float kfAlt, float kfVel, float tiltRad, bool kfHealthy,
                bool contLanding) {
    if (!std::isfinite(kfAlt) || !std::isfinite(kfVel) ||
        !std::isfinite(tiltRad)) {
      hNow_ = hReq_ = vDown_ = tiltOn_ = 0.0f;
      return false;
    }
    // 1. lag compensation (freefall: a = -g + k v^2, k = rho CdA / (2 m))
    const float tau = opt_.lagTauS;
    const float k = 0.5f * kRho * cfg::CDA_M2 / cfg::MASS_DESCENT_KG;
    const float aFree = -cfg::G0 + k * kfVel * kfVel;
    const float h = kfAlt + kfVel * tau;
    const float v = kfVel + aFree * tau;
    const float vDown = -v;
    hNow_ = h;
    vDown_ = vDown;

    // Tilt when thrust will actually start (IGNITION_DELAY_MS from now),
    // extrapolated from the filtered tilt rate; the current tilt until the
    // rate filter has settled.
    float tiltOn = tiltRad;
    if (tiltRateValid_) {
      tiltOn = tiltRad + tiltRate_ * (cfg::IGNITION_DELAY_MS * 0.001f);
      if (tiltOn < 0.0f) tiltOn = 0.0f;
    }
    tiltOn_ = tiltOn;

    if (vDown < cfg::BURN_MIN_SPEED_MS) return false;

    // 2. risk-aware aim + 3. tilt loss
    float hReq = landingtable::hFire(vDown) + cfg::BURN_TABLE_MARGIN_M;
    if (opt_.tiltComp) hReq += landingtable::tiltCoef(vDown) * tiltOn * tiltOn;
    hReq_ = hReq;

    const float tiltMax = cfg::TILT_IGNITE_MAX_DEG * cfg::DEG2RAD;
    bool tiltOk = tiltRad < tiltMax;
    if (!tiltOk && opt_.predictInterlock)
      tiltOk = tiltRateValid_ && tiltOn < tiltMax &&
               tiltRad < opt_.hardMaxDeg * cfg::DEG2RAD;
    return h <= hReq && tiltOk && kfHealthy && contLanding;
  }

  // First-order low-passed derivative of the tilt angle.
  void updateTiltRate(float tiltRad, float dtMs) {
    const float dt = dtMs * 0.001f;
    if (!std::isfinite(tiltRad) || !(dt > 0.0f)) return;
    if (havePrev_) {
      const float raw = (tiltRad - prevTilt_) / dt;
      if (rateSamples_ == 0) {
        tiltRate_ = raw;
      } else {
        const float rc = 1.0f / (2.0f * cfg::PI_F * cfg::TILT_RATE_LPF_HZ);
        tiltRate_ += dt / (rc + dt) * (raw - tiltRate_);
      }
      if (rateSamples_ < kRateWarmup) ++rateSamples_;
      else tiltRateValid_ = true;
    }
    prevTilt_ = tiltRad;
    havePrev_ = true;
  }

  static constexpr float kRho = 1.225f;   // kg/m^3, sea level (as the tables)
  static constexpr int kRateWarmup = 25;  // ticks before the rate is trusted

  Options opt_;
  float prevTilt_ = 0.0f, tiltRate_ = 0.0f;
  bool havePrev_ = false, tiltRateValid_ = false;
  int rateSamples_ = 0;
  float hNow_ = 0.0f, hReq_ = 0.0f, vDown_ = 0.0f, tiltOn_ = 0.0f;
};
