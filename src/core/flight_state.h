#pragma once
//
// Flight state machine. Pure logic (no hardware): consumes estimator outputs
// and health flags, produces state + one-shot pyro commands + TVC enables.
//
// Every transition is debounced (condition must persist) and the critical ones
// have redundant backup criteria (timers, altitude-drop) so a single bad
// signal can't strand the vehicle in the wrong state.
//
// Chute-abort policy: aborts are monitored during BOOST/COAST/DESCENT. An
// abort during BOOST waits for motor burnout before firing the chute. Aborts
// are NOT taken during LANDING_BURN — at that point active TVC is the least
// bad option, so the burn rides out (logged for post-flight review).
//
#include <cmath>

#include "../config.h"
#include "burn_table.h"

enum class FlightState : uint8_t {
  IDLE = 0,       // disarmed on the bench/pad, pyros inhibited
  ARMED,          // pre-launch: calibrated, zeroed, launch detection live
  BOOST,          // launch / powered ascent — TVC active
  COAST,          // unpowered ascent — servos centered
  APOGEE,         // transient marker state (one tick)
  DESCENT,        // freefall, waiting for the ignition window (FULL_LANDING)
  LANDING_BURN,   // F15 lit — TVC active
  DESCENT_CHUTE,  // under parachute (nominal in CHUTE_TEST, or post-abort)
  TOUCHDOWN,      // landed, everything safed — terminal
  ABORT,          // transient: inhibits landing motor, manages chute
};

enum class AbortReason : uint8_t {
  NONE = 0,
  TILT,            // attitude beyond TILT_ABORT_DEG
  IMU_FAIL,
  KF_UNHEALTHY,
  MISSED_WINDOW,   // fell below minimum ignition altitude without firing
  DUD_IGNITER,     // fired the landing motor, no thrust appeared
};

enum class FlightEvent : uint8_t {
  NONE = 0, ARM, DISARM, LAUNCH, BURNOUT, APOGEE_DET, FIRE_CHUTE,
  FIRE_LANDING, IGNITION_CONFIRMED, LANDING_BURNOUT, TOUCHDOWN_DET, ABORT_DET,
};

struct FsmInput {
  uint32_t ms = 0;       // monotonic milliseconds
  float dtMs = 2.0f;     // time since last update
  float kfAlt = 0;       // m AGL
  float kfVel = 0;       // m/s, +up
  bool kfHealthy = true;
  float tiltRad = 0;
  float accelLongG = 1;  // body +Z specific force, g (pad ≈ +1)
  float accelNormG = 1;  // |specific force|, g (freefall ≈ 0)
  bool imuHealthy = true;
  bool contChute = true;
  bool contLanding = true;
};

struct FsmOutput {
  FlightState state = FlightState::IDLE;
  bool tvcActive = false;
  bool useLandingGains = false;
  bool fireChute = false;    // one-shot pulse: actuator runs its own fire timer
  bool fireLanding = false;  // one-shot pulse
  bool inFlight = false;
  bool logFast = false;
  AbortReason abortReason = AbortReason::NONE;
};

class FlightStateMachine {
 public:
  void reset() { *this = FlightStateMachine{}; }

  void setMode(cfg::FlightMode m) { mode_ = m; }
  cfg::FlightMode mode() const { return mode_; }

  // Caller (CLI / arm switch) invokes these only after pre-arm checks pass.
  bool requestArm(uint32_t ms) {
    if (state_ != FlightState::IDLE) return false;
    enter(FlightState::ARMED, ms);
    pushEvent(ms, FlightEvent::ARM);
    return true;
  }
  bool requestDisarm(uint32_t ms) {
    if (state_ != FlightState::ARMED) return false;
    enter(FlightState::IDLE, ms);
    pushEvent(ms, FlightEvent::DISARM);
    return true;
  }

  FlightState state() const { return state_; }
  AbortReason abortReason() const { return abortReason_; }

  void update(const FsmInput& in, FsmOutput& out) {
    out = FsmOutput{};
    trackMaxAlt(in);

    switch (state_) {
      case FlightState::IDLE:
      case FlightState::ARMED:
        updateGround(in, out);
        break;
      case FlightState::BOOST:
        updateBoost(in, out);
        break;
      case FlightState::COAST:
        updateCoast(in, out);
        break;
      case FlightState::APOGEE:
        updateApogee(in, out);
        break;
      case FlightState::DESCENT:
        updateDescent(in, out);
        break;
      case FlightState::LANDING_BURN:
        updateLandingBurn(in, out);
        break;
      case FlightState::DESCENT_CHUTE:
        updateDescentChute(in, out);
        break;
      case FlightState::ABORT:
        updateAbort(in, out);
        break;
      case FlightState::TOUCHDOWN:
        break;  // terminal
    }

    out.state = state_;
    out.abortReason = abortReason_;
    out.inFlight = state_ >= FlightState::BOOST &&
                   state_ != FlightState::TOUCHDOWN;
    out.logFast = state_ != FlightState::IDLE;
  }

  // --- event queue (drained by the logger) ---
  struct Event {
    uint32_t ms = 0;
    FlightEvent code = FlightEvent::NONE;
    float value = 0;
  };
  bool popEvent(Event& e) {
    if (evtHead_ == evtTail_) return false;
    e = events_[evtTail_];
    evtTail_ = (evtTail_ + 1) % kMaxEvents;
    return true;
  }

  static const char* stateName(FlightState s) {
    switch (s) {
      case FlightState::IDLE: return "IDLE";
      case FlightState::ARMED: return "ARMED";
      case FlightState::BOOST: return "BOOST";
      case FlightState::COAST: return "COAST";
      case FlightState::APOGEE: return "APOGEE";
      case FlightState::DESCENT: return "DESCENT";
      case FlightState::LANDING_BURN: return "LANDING_BURN";
      case FlightState::DESCENT_CHUTE: return "DESCENT_CHUTE";
      case FlightState::TOUCHDOWN: return "TOUCHDOWN";
      case FlightState::ABORT: return "ABORT";
    }
    return "?";
  }
  static const char* eventName(FlightEvent e) {
    switch (e) {
      case FlightEvent::NONE: return "NONE";
      case FlightEvent::ARM: return "ARM";
      case FlightEvent::DISARM: return "DISARM";
      case FlightEvent::LAUNCH: return "LAUNCH";
      case FlightEvent::BURNOUT: return "BURNOUT";
      case FlightEvent::APOGEE_DET: return "APOGEE";
      case FlightEvent::FIRE_CHUTE: return "FIRE_CHUTE";
      case FlightEvent::FIRE_LANDING: return "FIRE_LANDING";
      case FlightEvent::IGNITION_CONFIRMED: return "IGNITION_CONFIRMED";
      case FlightEvent::LANDING_BURNOUT: return "LANDING_BURNOUT";
      case FlightEvent::TOUCHDOWN_DET: return "TOUCHDOWN";
      case FlightEvent::ABORT_DET: return "ABORT";
    }
    return "?";
  }

 private:
  // Debounce accumulator: returns true once `cond` has held for threshMs.
  struct Debounce {
    float acc = 0;
    bool check(bool cond, float dtMs, float threshMs) {
      acc = cond ? acc + dtMs : 0;
      return acc >= threshMs;
    }
    void reset() { acc = 0; }
  };

  void enter(FlightState s, uint32_t ms) {
    state_ = s;
    entryMs_ = ms;
    dbA_.reset();
    dbB_.reset();
    dbTilt_.reset();
    dbKf_.reset();
    dbKfHealth_.reset();
  }

  float sinceEntry(const FsmInput& in) const {
    return (float)(in.ms - entryMs_);
  }

  void trackMaxAlt(const FsmInput& in) {
    if (state_ >= FlightState::BOOST && state_ <= FlightState::DESCENT &&
        in.kfAlt > maxAlt_)
      maxAlt_ = in.kfAlt;
  }

  void pushEvent(uint32_t ms, FlightEvent code, float value = 0) {
    const int next = (evtHead_ + 1) % kMaxEvents;
    if (next == evtTail_) return;  // full — drop (logger has fallen behind)
    events_[evtHead_] = {ms, code, value};
    evtHead_ = next;
  }

  // Abort monitoring during BOOST/COAST/DESCENT. Returns true if aborted.
  bool checkAborts(const FsmInput& in, FsmOutput& out) {
    AbortReason r = AbortReason::NONE;
    if (dbTilt_.check(in.tiltRad > cfg::TILT_ABORT_DEG * cfg::DEG2RAD,
                      in.dtMs, cfg::TILT_ABORT_MS))
      r = AbortReason::TILT;
    else if (dbKf_.check(!in.imuHealthy, in.dtMs, 200.0f))
      r = AbortReason::IMU_FAIL;
    else if (state_ != FlightState::BOOST &&
             dbKfHealth_.check(!in.kfHealthy, in.dtMs,
                               cfg::KF_UNHEALTHY_ABORT_MS))
      r = AbortReason::KF_UNHEALTHY;
    if (r == AbortReason::NONE) return false;
    doAbort(in, out, r);
    return true;
  }

  void doAbort(const FsmInput& in, FsmOutput& out, AbortReason r) {
    abortReason_ = r;
    abortedFromBoost_ = (state_ == FlightState::BOOST);
    pushEvent(in.ms, FlightEvent::ABORT_DET, (float)(uint8_t)r);
    enter(FlightState::ABORT, in.ms);
    (void)out;
  }

  void fireChuteOnce(const FsmInput& in, FsmOutput& out) {
    if (chuteFired_) return;
    chuteFired_ = true;
    out.fireChute = true;
    pushEvent(in.ms, FlightEvent::FIRE_CHUTE, in.kfAlt);
  }

  // --- per-state handlers ---

  void updateGround(const FsmInput& in, FsmOutput& out) {
    if (state_ != FlightState::ARMED) return;
    if (dbA_.check(in.accelLongG > cfg::LAUNCH_ACCEL_G, in.dtMs,
                   cfg::LAUNCH_MS)) {
      launchMs_ = in.ms;
      pushEvent(in.ms, FlightEvent::LAUNCH);
      enter(FlightState::BOOST, in.ms);
      out.tvcActive = true;
    }
  }

  void updateBoost(const FsmInput& in, FsmOutput& out) {
    out.tvcActive = true;
    if (checkAborts(in, out)) return;
    const bool burnout =
        dbA_.check(in.accelNormG < cfg::BURNOUT_ACCEL_G, in.dtMs,
                   cfg::BURNOUT_MS) ||
        sinceEntry(in) > cfg::BOOST_MAX_MS;
    if (burnout) {
      pushEvent(in.ms, FlightEvent::BURNOUT, in.kfAlt);
      enter(FlightState::COAST, in.ms);
      out.tvcActive = false;
    }
  }

  void updateCoast(const FsmInput& in, FsmOutput& out) {
    if (checkAborts(in, out)) return;
    const bool apogee =
        dbA_.check(in.kfVel <= 0.0f, in.dtMs, cfg::APOGEE_VEL_MS) ||
        (in.kfAlt < maxAlt_ - cfg::APOGEE_DROP_M) ||
        sinceEntry(in) > cfg::COAST_MAX_MS;
    if (apogee) {
      pushEvent(in.ms, FlightEvent::APOGEE_DET, maxAlt_);
      enter(FlightState::APOGEE, in.ms);
    }
  }

  void updateApogee(const FsmInput& in, FsmOutput& out) {
    if (mode_ == cfg::FlightMode::CHUTE_TEST) {
      fireChuteOnce(in, out);
      enter(FlightState::DESCENT_CHUTE, in.ms);
    } else {
      enter(FlightState::DESCENT, in.ms);
    }
  }

  void updateDescent(const FsmInput& in, FsmOutput& out) {
    if (checkAborts(in, out)) return;

    // Missed the window? (Interlocks never cleared, or table said "already
    // too low".) Below the floor there is nothing left to time — abort.
    if (in.kfAlt < cfg::MISSED_WINDOW_ALT_M) {
      doAbort(in, out, AbortReason::MISSED_WINDOW);
      return;
    }

    const float vDown = -in.kfVel;
    if (vDown < cfg::BURN_MIN_SPEED_MS) return;

    const float hIgn = burntable::hIgnite(vDown) + cfg::BURN_TABLE_MARGIN_M;
    const bool windowOpen = in.kfAlt <= hIgn;
    const bool interlocks =
        in.tiltRad < cfg::TILT_IGNITE_MAX_DEG * cfg::DEG2RAD &&
        in.kfHealthy && in.contLanding;
    if (windowOpen && interlocks) {
      out.fireLanding = true;
      landFireMs_ = in.ms;
      ignitionConfirmed_ = false;
      pushEvent(in.ms, FlightEvent::FIRE_LANDING, in.kfAlt);
      enter(FlightState::LANDING_BURN, in.ms);
      out.tvcActive = true;
      out.useLandingGains = true;
    }
  }

  void updateLandingBurn(const FsmInput& in, FsmOutput& out) {
    out.tvcActive = true;
    out.useLandingGains = true;

    if (!ignitionConfirmed_) {
      if (sinceEntry(in) > 50.0f && in.accelNormG > cfg::IGNITE_CONFIRM_G) {
        ignitionConfirmed_ = true;
        pushEvent(in.ms, FlightEvent::IGNITION_CONFIRMED, in.kfAlt);
      } else if (sinceEntry(in) >
                 cfg::IGNITION_DELAY_MS + cfg::DUD_WINDOW_MS) {
        doAbort(in, out, AbortReason::DUD_IGNITER);
        out.tvcActive = false;
        return;
      }
    } else if (!landingBurnoutSeen_ &&
               dbB_.check(in.accelNormG < cfg::BURNOUT_ACCEL_G, in.dtMs,
                          cfg::BURNOUT_MS)) {
      landingBurnoutSeen_ = true;
      pushEvent(in.ms, FlightEvent::LANDING_BURNOUT, in.kfAlt);
    }

    const bool touchdown =
        dbA_.check(in.kfAlt < cfg::TOUCHDOWN_ALT_M &&
                       std::fabs(in.kfVel) < cfg::TOUCHDOWN_VEL_MS,
                   in.dtMs, cfg::TOUCHDOWN_MS) ||
        sinceEntry(in) > cfg::LANDING_BURN_MAX_MS;
    if (touchdown) {
      pushEvent(in.ms, FlightEvent::TOUCHDOWN_DET, in.kfAlt);
      enter(FlightState::TOUCHDOWN, in.ms);
      out.tvcActive = false;
    }
  }

  void updateDescentChute(const FsmInput& in, FsmOutput& out) {
    if (dbA_.check(in.kfAlt < cfg::TOUCHDOWN_ALT_M &&
                       std::fabs(in.kfVel) < cfg::TOUCHDOWN_VEL_MS,
                   in.dtMs, cfg::TOUCHDOWN_MS)) {
      pushEvent(in.ms, FlightEvent::TOUCHDOWN_DET, in.kfAlt);
      enter(FlightState::TOUCHDOWN, in.ms);
    }
    (void)out;
  }

  void updateAbort(const FsmInput& in, FsmOutput& out) {
    // Landing motor is inhibited for good (we never leave via DESCENT).
    if (abortedFromBoost_) {
      // Wait for the ascent motor to die before opening a chute into thrust.
      const bool motorDead =
          dbA_.check(in.accelNormG < cfg::BURNOUT_ACCEL_G, in.dtMs,
                     cfg::BURNOUT_MS) ||
          (in.ms - launchMs_) > cfg::BOOST_MAX_MS;
      if (!motorDead) return;
    }
    if (cfg::ABORT_FIRES_CHUTE_ALWAYS && !chuteFired_) fireChuteOnce(in, out);
    enter(FlightState::DESCENT_CHUTE, in.ms);
  }

  FlightState state_ = FlightState::IDLE;
  cfg::FlightMode mode_ = cfg::DEFAULT_MODE;
  AbortReason abortReason_ = AbortReason::NONE;
  uint32_t entryMs_ = 0, launchMs_ = 0, landFireMs_ = 0;
  float maxAlt_ = 0;
  bool chuteFired_ = false, ignitionConfirmed_ = false;
  bool landingBurnoutSeen_ = false, abortedFromBoost_ = false;
  Debounce dbA_, dbB_, dbTilt_, dbKf_, dbKfHealth_;

  static constexpr int kMaxEvents = 16;
  Event events_[kMaxEvents];
  int evtHead_ = 0, evtTail_ = 0;
};
