#pragma once
//
// Actuator layer: TVC gimbal servos, the parachute latch servo, and the
// landing-motor pyro channel.
//
// Gimbal servos: gimbal command (rad, torque about body X/Y) -> microseconds
// via per-axis center trim, scale, and sign from config. Hard-clamped to the
// servo travel limits on top of the controller's own gimbal clamp.
//
// Chute servo: holds the spring-ejection latch at CHUTE_LOCK_US; the flight
// logic's chuteRelease level moves it to CHUTE_RELEASE_US (and back to LOCK
// briefly when re-cycling a stuck latch). On the ground the flight logic
// never moves it, so the CLI can hold it open while the spring is loaded.
//
// Pyro: MOSFET gate driven HIGH for PYRO_FIRE_MS, then released, on a
// non-blocking timer. firePyro() only acts when enablePyro() has been set by
// the flight logic; testFire() is the explicit bench-test bypass used by the
// CLI (two-step confirmation lives there).
//
// Leg release: a second MOSFET powering the nichrome that burns the legs'
// rubber band. Unlike the e-match it is a LEVEL: on for as long as the flight
// logic says (fire + LEGS_DELAY_MS until touchdown), behind the same pyro
// enable, with an independent LEGS_BURN_MAX_MS cutoff. testLegs() is the
// bench bypass, on its own timer.
//
#include <Arduino.h>
#include <Servo.h>

#include "../config.h"

class Actuators {
 public:
  // Widest pulse range the `chute us` jog may command (typical servo limits).
  static constexpr int CHUTE_JOG_MIN_US = 500;
  static constexpr int CHUTE_JOG_MAX_US = 2500;

  enum class ChutePos : uint8_t { LOCK, RELEASE, JOG };

  void begin(float trimAUs, float trimBUs) {
    // Chute latch first, and atomically: Teensy 4's Servo::attach() loads the
    // 1500 us default and its pulse ISR may read it before our write, which
    // could nudge the latch toward release at power-up. With interrupts off
    // across attach + write, the very first pulse is already LOCK.
    noInterrupts();
    chute_.attach(cfg::PIN_SERVO_CHUTE, CHUTE_JOG_MIN_US, CHUTE_JOG_MAX_US);
    chute_.writeMicroseconds((int)cfg::CHUTE_LOCK_US);
    interrupts();
    chuteUs_ = cfg::CHUTE_LOCK_US;
    chutePos_ = ChutePos::LOCK;

    trimA_ = trimAUs;
    trimB_ = trimBUs;
    servoA_.attach(cfg::PIN_SERVO_A, (int)cfg::SERVO_MIN_US, (int)cfg::SERVO_MAX_US);
    servoB_.attach(cfg::PIN_SERVO_B, (int)cfg::SERVO_MIN_US, (int)cfg::SERVO_MAX_US);
    center();

    pinMode(cfg::PIN_PYRO_LAND, OUTPUT);
    digitalWrite(cfg::PIN_PYRO_LAND, LOW);
    pinMode(cfg::PIN_PYRO_LEGS, OUTPUT);
    digitalWrite(cfg::PIN_PYRO_LEGS, LOW);
    pinMode(cfg::PIN_ARM_SWITCH, INPUT_PULLUP);
  }

  // ---- gimbal servos ----
  void writeGimbal(float gimXRad, float gimYRad) {
    // gimX (torque about body X) -> servo A; gimY -> servo B.
    lastUsA_ = toUs(gimXRad, cfg::SERVO_A_CENTER_US + trimA_,
                    cfg::SERVO_A_US_PER_GIMBAL_DEG, cfg::SERVO_A_SIGN);
    lastUsB_ = toUs(gimYRad, cfg::SERVO_B_CENTER_US + trimB_,
                    cfg::SERVO_B_US_PER_GIMBAL_DEG, cfg::SERVO_B_SIGN);
    servoA_.writeMicroseconds((int)lastUsA_);
    servoB_.writeMicroseconds((int)lastUsB_);
  }
  void center() { writeGimbal(0, 0); }
  void writeRawUs(int usA, int usB) {  // bench tests only
    lastUsA_ = constrain((float)usA, cfg::SERVO_MIN_US, cfg::SERVO_MAX_US);
    lastUsB_ = constrain((float)usB, cfg::SERVO_MIN_US, cfg::SERVO_MAX_US);
    servoA_.writeMicroseconds((int)lastUsA_);
    servoB_.writeMicroseconds((int)lastUsB_);
  }
  float lastUsA() const { return lastUsA_; }
  float lastUsB() const { return lastUsB_; }
  void setTrims(float a, float b) { trimA_ = a; trimB_ = b; }
  float trimA() const { return trimA_; }
  float trimB() const { return trimB_; }

  // ---- parachute latch servo ----
  // Flight routing, called every fast tick with the core's chuteRelease level.
  // RELEASE is honored whenever commanded (the core only commands it in
  // flight; the arm switch is the same second layer as for the pyro). LOCK is
  // only applied in flight -- that's the re-cycle swing -- so on the ground a
  // position set from the CLI is left alone, and after touchdown the latch
  // stays open.
  void applyChute(bool releaseCmd, bool inFlight) {
    if (releaseCmd) {
      if (cfg::REQUIRE_ARM_SWITCH && !armSwitchOn()) return;
      if (chutePos_ != ChutePos::RELEASE) chuteRelease();
    } else if (inFlight && chutePos_ != ChutePos::LOCK) {
      chuteLock();
    }
  }
  void chuteLock() { chuteWrite(cfg::CHUTE_LOCK_US, ChutePos::LOCK); }
  void chuteRelease() { chuteWrite(cfg::CHUTE_RELEASE_US, ChutePos::RELEASE); }
  void chuteJogUs(int us) {  // bench tests only: find the LOCK/RELEASE values
    chuteWrite((float)constrain(us, CHUTE_JOG_MIN_US, CHUTE_JOG_MAX_US),
               ChutePos::JOG);
  }
  ChutePos chutePos() const { return chutePos_; }
  bool chuteLocked() const { return chutePos_ == ChutePos::LOCK; }
  bool chuteReleased() const { return chutePos_ == ChutePos::RELEASE; }
  float chuteUs() const { return chuteUs_; }

  // ---- landing-motor pyro ----
  void enablePyro(bool en) { pyroEnabled_ = en; }
  bool pyroEnabled() const { return pyroEnabled_; }

  bool firePyro(uint32_t ms) {
    if (!pyroEnabled_) return false;
    if (cfg::REQUIRE_ARM_SWITCH && !armSwitchOn()) return false;
    return gateOn(ms);
  }
  bool testFire(uint32_t ms) { return gateOn(ms); }

  void pyroTick(uint32_t ms) {
    if (fireEndMs_ != 0 && (int32_t)(ms - fireEndMs_) >= 0) {
      digitalWrite(cfg::PIN_PYRO_LAND, LOW);
      fireEndMs_ = 0;
    }
    if (legsSrc_ == LegsSrc::TEST && (int32_t)(ms - legsTestEndMs_) >= 0)
      legsOff();
  }
  bool pyroActive() const { return fireEndMs_ != 0; }

  float contVolts() const {
    return analogRead(cfg::PIN_CONT_LAND) * (3.3f / 1023.0f) *
           cfg::CONT_DIVIDER_RATIO;
  }
  bool continuity() const { return contVolts() > cfg::CONT_THRESHOLD_V; }

  // ---- leg-release nichrome ----
  // Flight routing, called every fast tick with the core's legsBurn level.
  // Turning ON needs the pyro enable (in flight) and the arm switch; turning
  // OFF only touches a flight burn, so the core's steady "off" doesn't cancel
  // a bench test -- and it still works after touchdown, when the enable has
  // already dropped. Past LEGS_BURN_MAX_MS the gate is cut and stays cut for
  // the rest of the flight, whatever the core says.
  void applyLegs(bool on, uint32_t ms) {
    if (legsSrc_ == LegsSrc::FLIGHT) {
      if (!on) {
        legsOff();
      } else if ((float)(ms - legsStartMs_) >= cfg::LEGS_BURN_MAX_MS) {
        legsOff();
        legsCutoff_ = true;
      }
    } else if (on && legsSrc_ == LegsSrc::NONE && !legsCutoff_ &&
               pyroEnabled_ && (!cfg::REQUIRE_ARM_SWITCH || armSwitchOn())) {
      digitalWrite(cfg::PIN_PYRO_LEGS, HIGH);
      legsSrc_ = LegsSrc::FLIGHT;
      legsStartMs_ = ms;
    }
  }
  void testLegs(uint32_t ms, uint32_t durMs) {  // bench bypass, CLI-confirmed
    digitalWrite(cfg::PIN_PYRO_LEGS, HIGH);
    legsSrc_ = LegsSrc::TEST;
    legsTestEndMs_ = ms + durMs;
  }
  bool legsActive() const { return legsSrc_ != LegsSrc::NONE; }
  bool legsCutoff() const { return legsCutoff_; }

  float legsContVolts() const {
    return analogRead(cfg::PIN_CONT_LEGS) * (3.3f / 1023.0f) *
           cfg::CONT_DIVIDER_RATIO;
  }
  bool legsContinuity() const {
    return legsContVolts() > cfg::CONT_THRESHOLD_V;
  }

  // Bench kill switch (CLI `stop`): every pyro gate LOW right now.
  void allPyrosOff() {
    digitalWrite(cfg::PIN_PYRO_LAND, LOW);
    fireEndMs_ = 0;
    legsOff();
  }

  // ---- misc ----
  float vbat() const {
    return analogRead(cfg::PIN_VBAT) * (3.3f / 1023.0f) * cfg::VBAT_DIVIDER;
  }
  bool armSwitchOn() const { return digitalRead(cfg::PIN_ARM_SWITCH) == LOW; }

 private:
  enum class LegsSrc : uint8_t { NONE, FLIGHT, TEST };

  void legsOff() {
    digitalWrite(cfg::PIN_PYRO_LEGS, LOW);
    legsSrc_ = LegsSrc::NONE;
  }
  bool gateOn(uint32_t ms) {
    digitalWrite(cfg::PIN_PYRO_LAND, HIGH);
    fireEndMs_ = ms + (uint32_t)cfg::PYRO_FIRE_MS;
    if (fireEndMs_ == 0) fireEndMs_ = 1;  // 0 means idle
    return true;
  }
  void chuteWrite(float us, ChutePos pos) {
    chute_.writeMicroseconds((int)us);
    chuteUs_ = us;
    chutePos_ = pos;
  }
  static float toUs(float rad, float center, float usPerDeg, float sign) {
    float us = center + sign * rad * cfg::RAD2DEG * usPerDeg;
    if (us < cfg::SERVO_MIN_US) us = cfg::SERVO_MIN_US;
    if (us > cfg::SERVO_MAX_US) us = cfg::SERVO_MAX_US;
    return us;
  }

  Servo servoA_, servoB_, chute_;
  float trimA_ = 0, trimB_ = 0;
  float lastUsA_ = 1500, lastUsB_ = 1500;
  float chuteUs_ = cfg::CHUTE_LOCK_US;
  ChutePos chutePos_ = ChutePos::LOCK;
  bool pyroEnabled_ = false;
  uint32_t fireEndMs_ = 0;
  LegsSrc legsSrc_ = LegsSrc::NONE;
  uint32_t legsStartMs_ = 0, legsTestEndMs_ = 0;
  bool legsCutoff_ = false;
};
