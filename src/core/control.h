#pragma once
//
// TVC attitude controller: two PID loops (one per gimbal axis) acting on the
// body-frame tilt error from Ahrs::tiltErrorBody().
//
//  - P acts on the tilt error angle.
//  - D uses the low-passed gyro rate directly (cleaner than differentiating
//    the error, and it damps rotation in the right direction: as the body
//    rotates toward vertical, d(error)/dt = -rate).
//  - I is clamped (anti-windup) and only integrates while thrust is present.
//
// Output is a gimbal deflection command (rad) per body axis, limited and
// slew-rate limited. The actuator layer maps it to servo microseconds; sign
// conventions live in config and are verified by the bench direction test.
//
#include "../config.h"

class TvcControl {
 public:
  void configure(float gimbalMaxRad, float slewRadPerS, float dLpfHz) {
    max_ = gimbalMaxRad;
    slew_ = slewRadPerS;
    dLpfHz_ = dLpfHz;
  }

  void setGains(const cfg::PidGains& g) { gains_ = g; }

  void reset() {
    for (auto& a : axis_) a = Axis{};
  }

  // errX/errY: body-frame tilt error rotation components (rad).
  // rateX/rateY: body angular rates (rad/s). integrate: thrust present.
  void update(float errX, float errY, float rateX, float rateY, float dt,
              bool integrate) {
    axisUpdate(axis_[0], errX, rateX, dt, integrate);
    axisUpdate(axis_[1], errY, rateY, dt, integrate);
  }

  float gimbalX() const { return axis_[0].out; }  // torque cmd about body X
  float gimbalY() const { return axis_[1].out; }  // torque cmd about body Y

  // For logging.
  float pTerm(int i) const { return axis_[i].p; }
  float iTerm(int i) const { return axis_[i].i; }
  float dTerm(int i) const { return axis_[i].d; }

 private:
  struct Axis {
    float i = 0, rateLpf = 0, out = 0;
    float p = 0, d = 0;
  };

  void axisUpdate(Axis& a, float err, float rate, float dt, bool integrate) {
    const float alpha = dt / (dt + 1.0f / (2.0f * cfg::PI_F * dLpfHz_));
    a.rateLpf += alpha * (rate - a.rateLpf);

    if (integrate) {
      a.i += gains_.ki * err * dt;
      if (a.i > gains_.iMaxRad) a.i = gains_.iMaxRad;
      if (a.i < -gains_.iMaxRad) a.i = -gains_.iMaxRad;
    }

    a.p = gains_.kp * err;
    a.d = -gains_.kd * a.rateLpf;
    float u = a.p + a.i + a.d;

    if (u > max_) u = max_;
    if (u < -max_) u = -max_;

    const float maxStep = slew_ * dt;
    float step = u - a.out;
    if (step > maxStep) step = maxStep;
    if (step < -maxStep) step = -maxStep;
    a.out += step;
  }

  cfg::PidGains gains_ = cfg::GAINS_BOOST;
  float max_ = cfg::GIMBAL_MAX_RAD;
  float slew_ = cfg::GIMBAL_SLEW_RADPS;
  float dLpfHz_ = cfg::D_LPF_HZ;
  Axis axis_[2];
};
