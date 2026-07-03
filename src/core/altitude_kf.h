#pragma once
//
// 3-state Kalman filter: x = [altitude (m, AGL), vertical velocity (m/s, +up),
// accel bias (m/s^2)].
//
// Predict runs at the IMU rate using world-frame vertical acceleration
// (attitude-rotated accel minus gravity); update runs whenever the barometer
// produces a new altitude. The bias state absorbs accelerometer offset and
// attitude error so velocity doesn't drift between baro updates.
//
// R (baro measurement variance) should be set from pad noise via setBaroVar().
// A gate rejects glitch innovations; after too many consecutive rejections the
// filter force-accepts one to re-converge instead of diverging forever.
//
#include <cmath>

class AltitudeKf {
 public:
  void configure(float sigmaAccel, float sigmaBiasRw, float gateSigma,
                 int gateForceAccept) {
    sigA_ = sigmaAccel;
    sigBrw_ = sigmaBiasRw;
    gate2_ = gateSigma * gateSigma;
    forceAccept_ = gateForceAccept;
  }

  void reset(float h0) {
    h_ = h0; v_ = 0; b_ = 0;
    P_[0][0] = 1.0f;  P_[0][1] = 0;     P_[0][2] = 0;
    P_[1][0] = 0;     P_[1][1] = 1.0f;  P_[1][2] = 0;
    P_[2][0] = 0;     P_[2][1] = 0;     P_[2][2] = 0.25f;
    rejects_ = 0;
    nis_ = 1.0f;
    innov_ = 0;
  }

  void setBaroVar(float r) { if (r > 1e-6f) R_ = r; }
  void setRInflation(float f) { rInfl_ = (f >= 1.0f) ? f : 1.0f; }

  // aWorldZ: world-frame vertical specific acceleration (m/s^2, +up, gravity
  // already removed).
  void predict(float aWorldZ, float dt) {
    const float a = aWorldZ - b_;
    h_ += v_ * dt + 0.5f * a * dt * dt;
    v_ += a * dt;

    // P = F P F^T + Q with F = [[1, d, -d^2/2], [0, 1, -d], [0, 0, 1]]
    const float d = dt, e = -0.5f * dt * dt, f = -dt;
    float M[3][3];
    for (int c = 0; c < 3; ++c) {
      M[0][c] = P_[0][c] + d * P_[1][c] + e * P_[2][c];
      M[1][c] = P_[1][c] + f * P_[2][c];
      M[2][c] = P_[2][c];
    }
    for (int r = 0; r < 3; ++r) {
      const float m0 = M[r][0], m1 = M[r][1], m2 = M[r][2];
      P_[r][0] = m0 + d * m1 + e * m2;
      P_[r][1] = m1 + f * m2;
      P_[r][2] = m2;
    }
    const float sa2 = sigA_ * sigA_;
    P_[0][0] += 0.25f * dt * dt * dt * dt * sa2;
    P_[0][1] += 0.5f * dt * dt * dt * sa2;
    P_[1][0] += 0.5f * dt * dt * dt * sa2;
    P_[1][1] += dt * dt * sa2;
    P_[2][2] += sigBrw_ * sigBrw_ * dt;
    symmetrize();
  }

  // Returns true if the measurement was accepted.
  bool update(float baroAlt) {
    const float Reff = R_ * rInfl_;
    const float y = baroAlt - h_;
    const float S = P_[0][0] + Reff;
    innov_ = y;

    if (y * y > gate2_ * S) {
      if (++rejects_ < forceAccept_) return false;  // glitch — ignore
      // Filter has lost the plot; accept to re-converge.
    }
    rejects_ = 0;
    nis_ = 0.98f * nis_ + 0.02f * (y * y / S);

    const float K0 = P_[0][0] / S, K1 = P_[1][0] / S, K2 = P_[2][0] / S;
    h_ += K0 * y;
    v_ += K1 * y;
    b_ += K2 * y;

    // P = (I - K H) P with H = [1 0 0]
    for (int c = 0; c < 3; ++c) {
      const float p0 = P_[0][c];
      P_[0][c] -= K0 * p0;
      P_[1][c] -= K1 * p0;
      P_[2][c] -= K2 * p0;
    }
    symmetrize();
    return true;
  }

  float altitude() const { return h_; }
  float velocity() const { return v_; }
  float bias() const { return b_; }
  float innovation() const { return innov_; }

  // Healthy = normalized innovation statistics near 1 and not stuck rejecting.
  bool healthy() const { return nis_ < 9.0f && rejects_ < forceAccept_; }

 private:
  void symmetrize() {
    P_[0][1] = P_[1][0] = 0.5f * (P_[0][1] + P_[1][0]);
    P_[0][2] = P_[2][0] = 0.5f * (P_[0][2] + P_[2][0]);
    P_[1][2] = P_[2][1] = 0.5f * (P_[1][2] + P_[2][1]);
  }

  float h_ = 0, v_ = 0, b_ = 0;
  float P_[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 0.25f}};
  float R_ = 0.25f, rInfl_ = 1.0f;
  float sigA_ = 0.5f, sigBrw_ = 0.01f, gate2_ = 25.0f;
  int forceAccept_ = 25, rejects_ = 0;
  float nis_ = 1.0f, innov_ = 0;
};
