#pragma once
//
// FlightCore: composes AHRS + altitude KF + TVC controller + state machine
// into one step() call. Both the Teensy firmware (RocketFC.ino) and the PC
// replay harness (tools/replay) drive THIS object, so the logic that flies is
// byte-for-byte the logic that was tested.
//
// Inputs are already conditioned: body-frame, axis-mapped, gyro bias removed
// (the sensors module / harness handles that part).
//
#include "ahrs.h"
#include "altitude_kf.h"
#include "control.h"
#include "flight_state.h"

struct CoreInput {
  uint32_t ms = 0;
  float dt = cfg::FAST_DT;   // seconds since last step
  Vec3 accel;                // specific force, m/s^2, body frame
  Vec3 gyro;                 // rad/s, body frame, bias-corrected
  bool baroNew = false;
  float baroAlt = 0;         // m AGL (referenced to pad)
  bool imuHealthy = true;
  bool contLanding = true;
};

struct CoreOutput {
  FlightState state = FlightState::IDLE;
  AbortReason abortReason = AbortReason::NONE;
  bool tvcActive = false;
  float gimbalX = 0, gimbalY = 0;  // rad
  bool chuteRelease = false;   // LEVEL: chute latch servo at RELEASE now
  bool chuteDetected = false;  // canopy confirmed by the accelerometer
  bool fireLanding = false;    // one-shot pulse
  bool legsBurn = false;       // LEVEL: leg-release nichrome on now
  float kfAlt = 0, kfVel = 0, kfBias = 0, innovation = 0;
  float tiltDeg = 0;
  Quat attitude;
  bool inFlight = false, logFast = false;
};

class FlightCore {
 public:
  void begin() {
    kf_.configure(cfg::KF_SIGMA_ACCEL, cfg::KF_SIGMA_BIAS_RW,
                  cfg::KF_GATE_SIGMA, cfg::KF_GATE_FORCE_ACCEPT);
    kf_.reset(0);
    kf_.setBaroVar(cfg::BARO_R_FALLBACK_M2);
    ctl_.configure(cfg::GIMBAL_MAX_RAD, cfg::GIMBAL_SLEW_RADPS, cfg::D_LPF_HZ);
    ctl_.reset();
    ahrs_.reset();
    fsm_.reset();
  }

  // --- pad / calibration hooks (called by CLI / arming sequence) ---
  void padLevelInit(const Vec3& accelAvg) { ahrs_.initFromAccel(accelAvg); }
  void setBaroNoiseVar(float var) { kf_.setBaroVar(var); }
  void zeroAltitude() { kf_.reset(0); }
  bool requestArm(uint32_t ms) { return fsm_.requestArm(ms); }
  bool requestDisarm(uint32_t ms) { return fsm_.requestDisarm(ms); }
  void setMode(cfg::FlightMode m) { fsm_.setMode(m); }
  cfg::FlightMode mode() const { return fsm_.mode(); }
  FlightState state() const { return fsm_.state(); }
  FlightStateMachine& fsm() { return fsm_; }
  const TvcControl& control() const { return ctl_; }
  const Ahrs& ahrs() const { return ahrs_; }

  void step(const CoreInput& in, CoreOutput& out) {
    // --- attitude ---
    if (!ahrs_.initialized()) ahrs_.initFromAccel(in.accel);
    ahrs_.update(in.gyro, in.dt);
    const bool onPad = fsm_.state() == FlightState::IDLE ||
                       fsm_.state() == FlightState::ARMED;
    if (onPad) {
      // Gentle accel leveling while at rest keeps the init fresh on the pad
      // (time constant ~1 s at 500 Hz). Disabled from liftoff: in flight the
      // accelerometer measures thrust, not gravity.
      ahrs_.accelLevelBlend(in.accel, 0.002f);
    }

    // --- altitude / velocity estimation ---
    kf_.predict(ahrs_.worldVerticalAccel(in.accel, cfg::G0), in.dt);
    if (in.baroNew) {
      kf_.setRInflation(fsm_.state() == FlightState::BOOST
                            ? cfg::KF_R_BOOST_INFLATE : 1.0f);
      kf_.update(in.baroAlt);
    }

    // --- state machine ---
    FsmInput fi;
    fi.ms = in.ms;
    fi.dtMs = in.dt * 1000.0f;
    fi.kfAlt = kf_.altitude();
    fi.kfVel = kf_.velocity();
    fi.kfHealthy = kf_.healthy();
    fi.tiltRad = ahrs_.tiltRad();
    fi.accelLongG = in.accel.z / cfg::G0;
    fi.accelNormG = in.accel.norm() / cfg::G0;
    fi.imuHealthy = in.imuHealthy;
    fi.contLanding = in.contLanding;
    FsmOutput fo;
    fsm_.update(fi, fo);

    // --- control ---
    if (fo.tvcActive) {
      ctl_.setGains(fo.useLandingGains ? cfg::GAINS_LANDING
                                       : cfg::GAINS_BOOST);
      const Vec3 e = ahrs_.tiltErrorBody();
      ctl_.update(e.x, e.y, in.gyro.x, in.gyro.y, in.dt, true);
    } else if (wasActive_) {
      ctl_.reset();  // clean integrators/slew state for the next burn phase
    }
    wasActive_ = fo.tvcActive;

    // --- outputs ---
    out.state = fo.state;
    out.abortReason = fo.abortReason;
    out.tvcActive = fo.tvcActive;
    out.gimbalX = fo.tvcActive ? ctl_.gimbalX() : 0.0f;
    out.gimbalY = fo.tvcActive ? ctl_.gimbalY() : 0.0f;
    out.chuteRelease = fo.chuteRelease;
    out.chuteDetected = fo.chuteDetected;
    out.fireLanding = fo.fireLanding;
    out.legsBurn = fo.legsBurn;
    out.kfAlt = kf_.altitude();
    out.kfVel = kf_.velocity();
    out.kfBias = kf_.bias();
    out.innovation = kf_.innovation();
    out.tiltDeg = ahrs_.tiltRad() * cfg::RAD2DEG;
    out.attitude = ahrs_.quat();
    out.inFlight = fo.inFlight;
    out.logFast = fo.logFast;
  }

 private:
  Ahrs ahrs_;
  AltitudeKf kf_;
  TvcControl ctl_;
  FlightStateMachine fsm_;
  bool wasActive_ = false;
};
