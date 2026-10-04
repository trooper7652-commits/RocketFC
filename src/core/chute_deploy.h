#pragma once
//
// Parachute deploy supervisor. Pure logic (no hardware): owned and stepped by
// FlightStateMachine, which decides WHEN to deploy; this decides where the
// spring-latch servo should be from then on, and whether it worked.
//
// The chute is spring-ejected and a servo holds the latch. After each release
// we watch |specific force| for the canopy: in freefall it reads ~0 g, under a
// canopy ~1 g with a multi-g opening spike. If nothing shows within
// CHUTE_CONFIRM_MS, the latch is re-cycled (back to LOCK for a short dwell,
// then RELEASE again) to shake a sticky latch loose, up to CHUTE_MAX_RELEASES
// releases in total. After that the servo holds RELEASE for good.
//
// Detection is measured against a baseline taken at each release, not a fixed
// threshold: drag on a fast, tumbling rocket after an abort can already read
// ~0.5 g, and keeps rising as it falls faster. The baseline is re-taken at
// every re-release so that slow drag build-up isn't mistaken for a canopy.
//
// Bias is deliberately toward "not detected": with one parachute and the
// spring already let go, re-cycling the latch when the chute is in fact out
// does no harm, while a false "detected" gives up on a stuck latch.
//
#include <cstdint>

#include "../config.h"

class ChuteDeploy {
 public:
  // One-shot flags from update(), for the state machine's event log.
  struct Tick {
    bool released = false;     // a re-cycle just released the latch again
    bool detected = false;     // canopy confirmed this tick
    bool unconfirmed = false;  // out of releases without seeing a canopy
  };

  void reset() { *this = ChuteDeploy{}; }

  // Call every tick, also before start(): keeps the |specific force| low-pass
  // that each release snapshots as its detection baseline. altM is the KF
  // altitude AGL.
  Tick update(uint32_t ms, float dtMs, float accelNormG, float altM) {
    Tick t;
    if (!lpfInit_) {
      lpfG_ = accelNormG;
      lpfInit_ = true;
    } else {
      lpfG_ += dtMs / (kBaselineTauMs + dtMs) * (accelNormG - lpfG_);
    }
    if (!started_) return t;

    // Detection stops once we give up: the last baseline is stale by then,
    // and drag building toward terminal velocity would eventually "confirm"
    // a canopy that isn't there. The log still has the raw accel.
    // It is also off near the ground, where a jolt is far more likely the
    // impact than a canopy -- a crash must never be logged as a good chute.
    if (!detected_ && !unconfirmed_) {
      const bool rise = accelNormG > baselineG_ + cfg::CHUTE_DETECT_RISE_G;
      detectAcc_ = rise && altM > cfg::TOUCHDOWN_ALT_M ? detectAcc_ + dtMs
                                                       : 0.0f;
      if (detectAcc_ >= cfg::CHUTE_DETECT_MS) {
        detected_ = true;
        t.detected = true;
        servoRelease_ = true;  // mid re-cycle? go straight back to RELEASE
        phase_ = Phase::HOLD;
        return t;
      }
    }

    switch (phase_) {
      case Phase::WAIT:  // released, waiting for the canopy
        if (since(ms) >= cfg::CHUTE_CONFIRM_MS) {
          if (releases_ < cfg::CHUTE_MAX_RELEASES) {
            servoRelease_ = false;  // swing back toward LOCK...
            phase_ = Phase::LOCK_DWELL;
            phaseMs_ = ms;
          } else {
            unconfirmed_ = true;
            t.unconfirmed = true;
            phase_ = Phase::HOLD;
          }
        }
        break;
      case Phase::LOCK_DWELL:  // ...and release again once it got there
        if (since(ms) >= cfg::CHUTE_RECYCLE_LOCK_MS) {
          release(ms);
          t.released = true;
        }
        break;
      case Phase::HOLD:
        break;
    }
    return t;
  }

  // First release. Returns false (and does nothing) if already started.
  bool start(uint32_t ms) {
    if (started_) return false;
    started_ = true;
    release(ms);
    return true;
  }

  // Stop re-cycling and hold RELEASE (touchdown: nothing left to fix).
  void stop() {
    if (!started_) return;
    servoRelease_ = true;
    phase_ = Phase::HOLD;
  }

  bool started() const { return started_; }
  bool servoRelease() const { return servoRelease_; }  // level: servo at RELEASE now
  bool detected() const { return detected_; }
  bool unconfirmed() const { return unconfirmed_; }
  int releases() const { return releases_; }

 private:
  enum class Phase : uint8_t { WAIT, LOCK_DWELL, HOLD };

  static constexpr float kBaselineTauMs = 50.0f;

  void release(uint32_t ms) {
    servoRelease_ = true;
    ++releases_;
    baselineG_ = lpfG_;
    phase_ = Phase::WAIT;
    phaseMs_ = ms;
  }

  float since(uint32_t ms) const { return (float)(int32_t)(ms - phaseMs_); }

  Phase phase_ = Phase::WAIT;
  bool started_ = false, servoRelease_ = false;
  bool detected_ = false, unconfirmed_ = false;
  bool lpfInit_ = false;
  int releases_ = 0;
  uint32_t phaseMs_ = 0;
  float lpfG_ = 0, baselineG_ = 0, detectAcc_ = 0;
};
