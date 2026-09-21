//
// RocketFC PC simulator bridge.
//
// Flat extern "C" ABI over FlightCore (../../src/core/flight_core.h) so the
// EXACT flight code (the same headers the Teensy compiles) can be driven from
// Python via ctypes, closed-loop, against a 6-DOF plant in tools/sim/.
//
// This mirrors what tools/replay/main.cpp already does (native C++ build of
// src/core, fed synthetic sensor rows) but as a shared library with a stable
// C struct layout instead of a standalone test binary, so tools/sim/core.py
// can call step() once per simulation tick.
//
// Build: see Makefile (g++ -shared -> rocketfc_core.dll). Structs are POD
// floats/ints only -- no STL crosses this boundary.
//
#include <cstdint>
#include <cstring>

#include "../../src/core/flight_core.h"

extern "C" {

// ---------------------------------------------------------------------------
// Mirrors CoreInput (flight_core.h). Kept as a separate flat struct (rather
// than reusing CoreInput's layout directly) so this ABI does not silently
// break if CoreInput's field order or types ever change.
// ---------------------------------------------------------------------------
struct CInput {
  uint32_t ms;
  float dt;
  float accel[3];       // specific force, m/s^2, body frame (sensor axes already mapped)
  float gyro[3];         // rad/s, body frame, bias-corrected
  int32_t baroNew;       // bool
  float baroAlt;         // m AGL
  int32_t imuHealthy;    // bool
  int32_t contChute;     // bool
  int32_t contLanding;   // bool
};

// Mirrors CoreOutput.
struct COutput {
  int32_t state;         // FlightState
  int32_t abortReason;   // AbortReason
  int32_t tvcActive;     // bool
  float gimbalX, gimbalY; // rad
  int32_t fireChute, fireLanding; // bool, one-shot pulses
  float kfAlt, kfVel, kfBias, innovation;
  float tiltDeg;
  float quat[4];          // w, x, y, z
  int32_t inFlight, logFast; // bool
  // PID terms, for logging/plotting (not in CoreOutput; pulled from control()).
  float pTermX, iTermX, dTermX, pTermY, iTermY, dTermY;
  // Servo microseconds, computed here from gimbalX/Y via the same mapping
  // Actuators::writeGimbal() uses (config-driven center/scale/sign/clamp).
  float usA, usB;
};

struct CEvent {
  uint32_t ms;
  int32_t code;   // FlightEvent
  float value;
};

// ---------------------------------------------------------------------------
// Config snapshot: the compiled-in cfg:: constants the sim/GUI needs to
// display or reason about. This is a READ of the values FlightCore was built
// with -- when tools/sim/core.py patches config.h and rebuilds, a fresh call
// into the freshly-loaded DLL reflects the patched values.
// ---------------------------------------------------------------------------
struct CConfigSnapshot {
  float imuRSb[9];
  float massPadKg, massDescentKg, cdaM2, ascentBurnMs, ignitionDelayMs;
  float gainsBoostKp, gainsBoostKi, gainsBoostKd, gainsBoostIMax;
  float gainsLandKp, gainsLandKi, gainsLandKd, gainsLandIMax;
  float gimbalMaxRad, gimbalSlewRadps, dLpfHz;
  float servoACenterUs, servoBCenterUs, servoAUsPerDeg, servoBUsPerDeg;
  float servoASign, servoBSign, servoMinUs, servoMaxUs;
  float launchAccelG, launchMs, burnoutAccelG, burnoutMs, boostMaxMs;
  float apogeeVelMs, apogeeDropM, coastMaxMs;
  float tiltAbortDeg, tiltAbortMs, tiltIgniteMaxDeg;
  float burnTableMarginM, burnMinSpeedMs, missedWindowAltM;
  float igniteConfirmG, dudWindowMs, landingBurnMaxMs;
  float touchdownAltM, touchdownVelMs, touchdownMs;
  float prearmTiltMaxDeg;
  int32_t abortFiresChuteAlways;
  float kfUnhealthyAbortMs;
  float kfSigmaAccel, kfSigmaBiasRw, kfGateSigma, kfRBoostInflate;
  int32_t kfGateForceAccept;
  float baroRFallbackM2;
  float g0, fastDt;
};

void rfc_config_snapshot(CConfigSnapshot* out) {
  if (!out) return;
  std::memset(out, 0, sizeof(*out));
  for (int i = 0; i < 9; ++i) out->imuRSb[i] = cfg::IMU_R_SB[i];
  out->massPadKg = cfg::MASS_PAD_KG;
  out->massDescentKg = cfg::MASS_DESCENT_KG;
  out->cdaM2 = cfg::CDA_M2;
  out->ascentBurnMs = cfg::ASCENT_BURN_MS;
  out->ignitionDelayMs = cfg::IGNITION_DELAY_MS;
  out->gainsBoostKp = cfg::GAINS_BOOST.kp;
  out->gainsBoostKi = cfg::GAINS_BOOST.ki;
  out->gainsBoostKd = cfg::GAINS_BOOST.kd;
  out->gainsBoostIMax = cfg::GAINS_BOOST.iMaxRad;
  out->gainsLandKp = cfg::GAINS_LANDING.kp;
  out->gainsLandKi = cfg::GAINS_LANDING.ki;
  out->gainsLandKd = cfg::GAINS_LANDING.kd;
  out->gainsLandIMax = cfg::GAINS_LANDING.iMaxRad;
  out->gimbalMaxRad = cfg::GIMBAL_MAX_RAD;
  out->gimbalSlewRadps = cfg::GIMBAL_SLEW_RADPS;
  out->dLpfHz = cfg::D_LPF_HZ;
  out->servoACenterUs = cfg::SERVO_A_CENTER_US;
  out->servoBCenterUs = cfg::SERVO_B_CENTER_US;
  out->servoAUsPerDeg = cfg::SERVO_A_US_PER_GIMBAL_DEG;
  out->servoBUsPerDeg = cfg::SERVO_B_US_PER_GIMBAL_DEG;
  out->servoASign = cfg::SERVO_A_SIGN;
  out->servoBSign = cfg::SERVO_B_SIGN;
  out->servoMinUs = cfg::SERVO_MIN_US;
  out->servoMaxUs = cfg::SERVO_MAX_US;
  out->launchAccelG = cfg::LAUNCH_ACCEL_G;
  out->launchMs = cfg::LAUNCH_MS;
  out->burnoutAccelG = cfg::BURNOUT_ACCEL_G;
  out->burnoutMs = cfg::BURNOUT_MS;
  out->boostMaxMs = cfg::BOOST_MAX_MS;
  out->apogeeVelMs = cfg::APOGEE_VEL_MS;
  out->apogeeDropM = cfg::APOGEE_DROP_M;
  out->coastMaxMs = cfg::COAST_MAX_MS;
  out->tiltAbortDeg = cfg::TILT_ABORT_DEG;
  out->tiltAbortMs = cfg::TILT_ABORT_MS;
  out->tiltIgniteMaxDeg = cfg::TILT_IGNITE_MAX_DEG;
  out->burnTableMarginM = cfg::BURN_TABLE_MARGIN_M;
  out->burnMinSpeedMs = cfg::BURN_MIN_SPEED_MS;
  out->missedWindowAltM = cfg::MISSED_WINDOW_ALT_M;
  out->igniteConfirmG = cfg::IGNITE_CONFIRM_G;
  out->dudWindowMs = cfg::DUD_WINDOW_MS;
  out->landingBurnMaxMs = cfg::LANDING_BURN_MAX_MS;
  out->touchdownAltM = cfg::TOUCHDOWN_ALT_M;
  out->touchdownVelMs = cfg::TOUCHDOWN_VEL_MS;
  out->touchdownMs = cfg::TOUCHDOWN_MS;
  out->prearmTiltMaxDeg = cfg::PREARM_TILT_MAX_DEG;
  out->abortFiresChuteAlways = cfg::ABORT_FIRES_CHUTE_ALWAYS ? 1 : 0;
  out->kfUnhealthyAbortMs = cfg::KF_UNHEALTHY_ABORT_MS;
  out->kfSigmaAccel = cfg::KF_SIGMA_ACCEL;
  out->kfSigmaBiasRw = cfg::KF_SIGMA_BIAS_RW;
  out->kfGateSigma = cfg::KF_GATE_SIGMA;
  out->kfRBoostInflate = cfg::KF_R_BOOST_INFLATE;
  out->kfGateForceAccept = cfg::KF_GATE_FORCE_ACCEPT;
  out->baroRFallbackM2 = cfg::BARO_R_FALLBACK_M2;
  out->g0 = cfg::G0;
  out->fastDt = cfg::FAST_DT;
}

// ---------------------------------------------------------------------------
// FlightCore lifecycle. Opaque handle -- Python never looks inside.
// ---------------------------------------------------------------------------
void* rfc_create() { return new FlightCore(); }
void rfc_destroy(void* h) { delete static_cast<FlightCore*>(h); }

void rfc_begin(void* h) { static_cast<FlightCore*>(h)->begin(); }

void rfc_set_mode(void* h, int32_t mode) {
  static_cast<FlightCore*>(h)->setMode(
      mode == 1 ? cfg::FlightMode::FULL_LANDING : cfg::FlightMode::CHUTE_TEST);
}

void rfc_pad_level_init(void* h, float ax, float ay, float az) {
  static_cast<FlightCore*>(h)->padLevelInit(Vec3{ax, ay, az});
}

void rfc_set_baro_noise_var(void* h, float var) {
  static_cast<FlightCore*>(h)->setBaroNoiseVar(var);
}

void rfc_zero_altitude(void* h) { static_cast<FlightCore*>(h)->zeroAltitude(); }

int32_t rfc_request_arm(void* h, uint32_t ms) {
  return static_cast<FlightCore*>(h)->requestArm(ms) ? 1 : 0;
}

int32_t rfc_request_disarm(void* h, uint32_t ms) {
  return static_cast<FlightCore*>(h)->requestDisarm(ms) ? 1 : 0;
}

int32_t rfc_state(void* h) {
  return static_cast<int32_t>(static_cast<FlightCore*>(h)->state());
}

// Same servo mapping Actuators::toUs() uses (src/hw/actuators.h), copied here
// as pure math because actuators.h pulls in <Arduino.h>/<Servo.h> and cannot
// be compiled on PC. Keep this in sync with actuators.h::toUs() by eye --
// tools/replay's cross-checks don't cover the actuator layer, so a drift here
// would silently mismap gimbal commands to servo microseconds in the sim only.
static float toUsPure(float rad, float center, float usPerDeg, float sign,
                      float minUs, float maxUs) {
  float us = center + sign * rad * cfg::RAD2DEG * usPerDeg;
  if (us < minUs) us = minUs;
  if (us > maxUs) us = maxUs;
  return us;
}

void rfc_step(void* h, const CInput* in, COutput* out) {
  if (!h || !in || !out) return;
  FlightCore* core = static_cast<FlightCore*>(h);

  CoreInput ci;
  ci.ms = in->ms;
  ci.dt = in->dt;
  ci.accel = Vec3{in->accel[0], in->accel[1], in->accel[2]};
  ci.gyro = Vec3{in->gyro[0], in->gyro[1], in->gyro[2]};
  ci.baroNew = in->baroNew != 0;
  ci.baroAlt = in->baroAlt;
  ci.imuHealthy = in->imuHealthy != 0;
  ci.contChute = in->contChute != 0;
  ci.contLanding = in->contLanding != 0;

  CoreOutput co;
  core->step(ci, co);

  std::memset(out, 0, sizeof(*out));
  out->state = static_cast<int32_t>(co.state);
  out->abortReason = static_cast<int32_t>(co.abortReason);
  out->tvcActive = co.tvcActive ? 1 : 0;
  out->gimbalX = co.gimbalX;
  out->gimbalY = co.gimbalY;
  out->fireChute = co.fireChute ? 1 : 0;
  out->fireLanding = co.fireLanding ? 1 : 0;
  out->kfAlt = co.kfAlt;
  out->kfVel = co.kfVel;
  out->kfBias = co.kfBias;
  out->innovation = co.innovation;
  out->tiltDeg = co.tiltDeg;
  out->quat[0] = co.attitude.w;
  out->quat[1] = co.attitude.x;
  out->quat[2] = co.attitude.y;
  out->quat[3] = co.attitude.z;
  out->inFlight = co.inFlight ? 1 : 0;
  out->logFast = co.logFast ? 1 : 0;

  const TvcControl& ctl = core->control();
  out->pTermX = ctl.pTerm(0);
  out->iTermX = ctl.iTerm(0);
  out->dTermX = ctl.dTerm(0);
  out->pTermY = ctl.pTerm(1);
  out->iTermY = ctl.iTerm(1);
  out->dTermY = ctl.dTerm(1);

  out->usA = toUsPure(co.gimbalX, cfg::SERVO_A_CENTER_US,
                      cfg::SERVO_A_US_PER_GIMBAL_DEG, cfg::SERVO_A_SIGN,
                      cfg::SERVO_MIN_US, cfg::SERVO_MAX_US);
  out->usB = toUsPure(co.gimbalY, cfg::SERVO_B_CENTER_US,
                      cfg::SERVO_B_US_PER_GIMBAL_DEG, cfg::SERVO_B_SIGN,
                      cfg::SERVO_MIN_US, cfg::SERVO_MAX_US);
}

int32_t rfc_pop_event(void* h, CEvent* ev) {
  if (!h || !ev) return 0;
  FlightStateMachine::Event e;
  if (!static_cast<FlightCore*>(h)->fsm().popEvent(e)) return 0;
  ev->ms = e.ms;
  ev->code = static_cast<int32_t>(e.code);
  ev->value = e.value;
  return 1;
}

}  // extern "C"
