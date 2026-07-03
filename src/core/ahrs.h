#pragma once
//
// AHRS: attitude tracking by quaternion integration of the BMI088 gyro.
//
// The accelerometer can only observe tilt while the vehicle is quasi-static
// (sitting on the pad): in flight it measures thrust + drag, which point along
// the body no matter which way the rocket leans, so it carries no "down"
// information. Therefore: accel sets the initial attitude (and may slowly
// correct it while ARMED on the pad), and from liftoff the attitude is pure
// gyro integration. Gyro bias is calibrated externally (sensors module) and
// must be removed from the rates passed in here.
//
#include "quat.h"

class Ahrs {
 public:
  void reset() { q_ = Quat::identity(); inited_ = false; }

  // Initialize attitude from the specific force measured at rest (body frame).
  // On the pad the accelerometer reads +1 g along whatever body axis points up,
  // so the rotation taking that reading to world-up is the (roll/yaw-free)
  // initial attitude. Yaw is unobservable and irrelevant for tilt control.
  void initFromAccel(const Vec3& accelBody) {
    // At rest the specific force is world-up, so the body->world attitude must
    // rotate the measured accel direction onto world-up.
    q_ = Quat::fromTwoVectors(accelBody, Vec3{0, 0, 1});
    q_.normalize();
    inited_ = true;
  }

  // Integrate bias-corrected body rates (rad/s).
  void update(const Vec3& gyroRadS, float dt) { q_.integrate(gyroRadS, dt); }

  // Slow complementary tilt correction from accel. ONLY call while the vehicle
  // is at rest (ARMED on the pad); alpha is the blend fraction per call.
  void accelLevelBlend(const Vec3& accelBody, float alpha) {
    if (!inited_) { initFromAccel(accelBody); return; }
    const Vec3 upBodyPred = q_.rotateInv(Vec3{0, 0, 1});
    const Quat corr = Quat::fromTwoVectors(accelBody.normalized(), upBodyPred);
    q_ = q_ * Quat::fromRotVec(corr.toRotVec() * alpha);
    q_.normalize();
  }

  const Quat& quat() const { return q_; }
  bool initialized() const { return inited_; }

  // Angle between the body long axis (+Z) and world-up, rad.
  float tiltRad() const {
    const Vec3 zb = q_.rotate(Vec3{0, 0, 1});
    float c = zb.z;
    if (c > 1.0f) c = 1.0f;
    if (c < -1.0f) c = -1.0f;
    return std::acos(c);
  }

  // Tilt error as a BODY-FRAME rotation vector: the rotation the body must
  // perform to bring its long axis onto world-up. x/y components are the
  // controllable errors for the two gimbal axes; z (roll) is uncontrollable
  // and ignored. Expressing the error in the body frame is what decouples the
  // controller from roll: it stays correct at any roll angle.
  Vec3 tiltErrorBody() const {
    const Vec3 zbWorld = q_.rotate(Vec3{0, 0, 1});
    const Vec3 axis = zbWorld.cross(Vec3{0, 0, 1});  // world-frame axis * sin
    const float s = axis.norm();
    if (s < 1e-6f) return {0, 0, 0};
    const float ang = std::atan2(s, zbWorld.z);
    const Vec3 rWorld = axis * (ang / s);
    return q_.rotateInv(rWorld);
  }

  // Vertical (world-Z) component of specific force, for the altitude KF:
  // a_world = R(q)*f_body - g. Returns m/s^2, +up.
  float worldVerticalAccel(const Vec3& accelBody, float g0) const {
    return q_.rotate(accelBody).z - g0;
  }

 private:
  Quat q_ = Quat::identity();
  bool inited_ = false;
};
