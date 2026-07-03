#pragma once
//
// Actuator layer: TVC gimbal servos and pyro channels.
//
// Servos: gimbal command (rad, torque about body X/Y) -> microseconds via
// per-axis center trim, scale, and sign from config. Hard-clamped to the
// servo travel limits on top of the controller's own gimbal clamp.
//
// Pyros: MOSFET gates driven HIGH for PYRO_FIRE_MS, then released, on a
// non-blocking timer. firePyro() only acts when enable() has been set by the
// flight logic; testFire() is the explicit bench-test bypass used by the CLI
// (two-step confirmation lives there).
//
#include <Arduino.h>
#include <Servo.h>

#include "../config.h"

class Actuators {
 public:
  static constexpr int CH_CHUTE = 0;
  static constexpr int CH_LAND = 1;

  void begin(float trimAUs, float trimBUs) {
    trimA_ = trimAUs;
    trimB_ = trimBUs;
    servoA_.attach(cfg::PIN_SERVO_A, (int)cfg::SERVO_MIN_US, (int)cfg::SERVO_MAX_US);
    servoB_.attach(cfg::PIN_SERVO_B, (int)cfg::SERVO_MIN_US, (int)cfg::SERVO_MAX_US);
    center();

    pinMode(cfg::PIN_PYRO_CHUTE, OUTPUT);
    pinMode(cfg::PIN_PYRO_LAND, OUTPUT);
    digitalWrite(cfg::PIN_PYRO_CHUTE, LOW);
    digitalWrite(cfg::PIN_PYRO_LAND, LOW);
    pinMode(cfg::PIN_ARM_SWITCH, INPUT_PULLUP);
  }

  // ---- servos ----
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

  // ---- pyros ----
  void enablePyros(bool en) { pyrosEnabled_ = en; }
  bool pyrosEnabled() const { return pyrosEnabled_; }

  bool firePyro(int ch, uint32_t ms) {
    if (!pyrosEnabled_) return false;
    if (cfg::REQUIRE_ARM_SWITCH && !armSwitchOn()) return false;
    return gateOn(ch, ms);
  }
  bool testFire(int ch, uint32_t ms) { return gateOn(ch, ms); }

  void pyroTick(uint32_t ms) {
    for (int ch = 0; ch < 2; ++ch) {
      if (fireEndMs_[ch] != 0 && (int32_t)(ms - fireEndMs_[ch]) >= 0) {
        digitalWrite(pinFor(ch), LOW);
        fireEndMs_[ch] = 0;
      }
    }
  }
  bool pyroActive(int ch) const { return fireEndMs_[ch] != 0; }

  float contVolts(int ch) const {
    const int pin = (ch == CH_CHUTE) ? cfg::PIN_CONT_CHUTE : cfg::PIN_CONT_LAND;
    return analogRead(pin) * (3.3f / 1023.0f) * cfg::CONT_DIVIDER_RATIO;
  }
  bool continuity(int ch) const { return contVolts(ch) > cfg::CONT_THRESHOLD_V; }

  // ---- misc ----
  float vbat() const {
    return analogRead(cfg::PIN_VBAT) * (3.3f / 1023.0f) * cfg::VBAT_DIVIDER;
  }
  bool armSwitchOn() const { return digitalRead(cfg::PIN_ARM_SWITCH) == LOW; }

 private:
  int pinFor(int ch) const {
    return (ch == CH_CHUTE) ? cfg::PIN_PYRO_CHUTE : cfg::PIN_PYRO_LAND;
  }
  bool gateOn(int ch, uint32_t ms) {
    if (ch < 0 || ch > 1) return false;
    digitalWrite(pinFor(ch), HIGH);
    fireEndMs_[ch] = ms + (uint32_t)cfg::PYRO_FIRE_MS;
    if (fireEndMs_[ch] == 0) fireEndMs_[ch] = 1;  // 0 means idle
    return true;
  }
  static float toUs(float rad, float center, float usPerDeg, float sign) {
    float us = center + sign * rad * cfg::RAD2DEG * usPerDeg;
    if (us < cfg::SERVO_MIN_US) us = cfg::SERVO_MIN_US;
    if (us > cfg::SERVO_MAX_US) us = cfg::SERVO_MAX_US;
    return us;
  }

  Servo servoA_, servoB_;
  float trimA_ = 0, trimB_ = 0;
  float lastUsA_ = 1500, lastUsB_ = 1500;
  bool pyrosEnabled_ = false;
  uint32_t fireEndMs_[2] = {0, 0};
};
