"""B3c: solve for the wind AND a constant sideways thrust offset (gimbal zero error /
motor misalignment) together. The offset scales with thrust, the air force with
airspeed, so a least-squares fit over the boost can tell them apart. sw/ideas3."""
import os
import patchlib
patchlib.WS = os.path.join(patchlib.SCR, "sw", "ideas3")
from patchlib import edit

edit("src/config.h",
"""constexpr float WIND_EST_PRIOR_MPS = 0.0f;
""",
"""constexpr float WIND_EST_PRIOR_MPS = 0.0f;
// B3c: also estimate a constant sideways thrust offset (gimbal zero error / motor
// misalignment, as an angle). Uses WIND_EST_PRIOR_MPS as the wind prior; this is
// the offset prior. 0 = off (plain B3/B3b).
constexpr float WIND_EST_OFFSET_PRIOR_DEG = 0.0f;
""")
edit("src/core/flight_core.h",
"""  void windEstStep(const CoreInput& in) {
    const float az = in.accel.z;                    // ~thrust / mass
    if (az < 3.0f) return;
""",
"""  void windEstStep(const CoreInput& in) {
    const float az = in.accel.z;                    // ~thrust / mass
    if (az < 3.0f) return;
    if (cfg::WIND_EST_OFFSET_PRIOR_DEG > 0.0f) { windOffsetStep(in, az); return; }
""")
edit("src/core/flight_core.h",
"""  Ahrs ahrs_;
  AltitudeKf kf_;
""",
"""  // B3c: per tick, two equations in x = (wind x, wind y, offset x, offset y):
  //   n + k R^T v = k R^T w + az * offset      (body sideways axes)
  // n = sideways specific force minus the commanded gimbal's thrust share,
  // k = 0.5 rho CNa A |V| / m, R = body->world. Normal equations with priors.
  void windOffsetStep(const CoreInput& in, float az) {
    const float nx = in.accel.x + az * std::sin(ctl_.gimbalY());
    const float ny = in.accel.y - az * std::sin(ctl_.gimbalX());
    // noise variance of n from successive differences (per axis)
    if (windNn_ > 0) {
      const float ddx = nx - windPnx_, ddy = ny - windPny_;
      windNv_ += 0.25f * (ddx * ddx + ddy * ddy);
    }
    ++windNn_;
    windPnx_ = nx;
    windPny_ = ny;
    const float vUp = kf_.velocity();
    const float dwx = velHx_ - windX_ / cfg::WIND_EST_SCALE;
    const float dwy = velHy_ - windY_ / cfg::WIND_EST_SCALE;
    const float speed = std::sqrt(vUp * vUp + dwx * dwx + dwy * dwy);
    const float k = 0.5f * 1.225f * cfg::AERO_CNA_AREF_M2 / cfg::MASS_DESCENT_KG * speed;
    const Quat& q = ahrs_.quat();
    // body components of the world x / y unit vectors (columns of R^T)
    const Vec3 ex = q.rotateInv(Vec3{1, 0, 0}), ey = q.rotateInv(Vec3{0, 0, 0} + Vec3{0, 1, 0});
    const Vec3 vb = q.rotateInv(Vec3{velHx_, velHy_, vUp});
    // rows for body x and body y
    const float rx[4] = {k * ex.x, k * ey.x, az, 0.0f};
    const float ry[4] = {k * ex.y, k * ey.y, 0.0f, az};
    const float yx = nx + k * vb.x, yy = ny + k * vb.y;
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) wN_[i][j] += rx[i] * rx[j] + ry[i] * ry[j];
      wB_[i] += rx[i] * yx + ry[i] * yy;
    }
    if (windNn_ < 50) return;
    const float nv = windNv_ / (float)(windNn_ - 1);
    const float pw = cfg::WIND_EST_PRIOR_MPS > 0.0f
                         ? nv / (cfg::WIND_EST_PRIOR_MPS * cfg::WIND_EST_PRIOR_MPS) : 1e-6f;
    const float od = cfg::WIND_EST_OFFSET_PRIOR_DEG * cfg::DEG2RAD;
    const float po = nv / (od * od);
    float A[4][5];
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) A[i][j] = wN_[i][j];
      A[i][4] = wB_[i];
    }
    A[0][0] += pw; A[1][1] += pw; A[2][2] += po; A[3][3] += po;
    for (int c = 0; c < 4; ++c) {          // Gauss-Jordan, SPD so no pivoting
      const float p = A[c][c];
      if (std::fabs(p) < 1e-12f) return;
      for (int j = c; j < 5; ++j) A[c][j] /= p;
      for (int r = 0; r < 4; ++r) {
        if (r == c) continue;
        const float f = A[r][c];
        for (int j = c; j < 5; ++j) A[r][j] -= f * A[c][j];
      }
    }
    windX_ = cfg::WIND_EST_SCALE * A[0][4];
    windY_ = cfg::WIND_EST_SCALE * A[1][4];
    offX_ = A[2][4];
    offY_ = A[3][4];
  }

  Ahrs ahrs_;
  AltitudeKf kf_;
""")
edit("src/core/flight_core.h",
"""  int windNn_ = 0;
""",
"""  int windNn_ = 0;
  float wN_[4][4] = {}, wB_[4] = {};
  float offX_ = 0.0f, offY_ = 0.0f;
""")
print("done")
