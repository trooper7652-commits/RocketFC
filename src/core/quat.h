#pragma once
//
// Minimal 3-vector / quaternion math for attitude tracking and control.
// Pure C++ (shared with the PC replay harness).
//
// Conventions: Hamilton quaternions, w-first. A quaternion q represents the
// BODY -> WORLD rotation: v_world = q.rotate(v_body).
//
#include <cmath>

struct Vec3 {
  float x = 0, y = 0, z = 0;

  Vec3 operator+(const Vec3& r) const { return {x + r.x, y + r.y, z + r.z}; }
  Vec3 operator-(const Vec3& r) const { return {x - r.x, y - r.y, z - r.z}; }
  Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
  float dot(const Vec3& r) const { return x * r.x + y * r.y + z * r.z; }
  Vec3 cross(const Vec3& r) const {
    return {y * r.z - z * r.y, z * r.x - x * r.z, x * r.y - y * r.x};
  }
  float norm() const { return std::sqrt(x * x + y * y + z * z); }
  Vec3 normalized() const {
    float n = norm();
    return (n > 1e-9f) ? Vec3{x / n, y / n, z / n} : Vec3{0, 0, 1};
  }
};

struct Quat {
  float w = 1, x = 0, y = 0, z = 0;

  static Quat identity() { return {1, 0, 0, 0}; }

  // Hamilton product: this ⊗ r
  Quat operator*(const Quat& r) const {
    return {w * r.w - x * r.x - y * r.y - z * r.z,
            w * r.x + x * r.w + y * r.z - z * r.y,
            w * r.y - x * r.z + y * r.w + z * r.x,
            w * r.z + x * r.y - y * r.x + z * r.w};
  }

  Quat conj() const { return {w, -x, -y, -z}; }

  void normalize() {
    float n = std::sqrt(w * w + x * x + y * y + z * z);
    if (n > 1e-9f) { w /= n; x /= n; y /= n; z /= n; }
    else { *this = identity(); }
  }

  // Rotate a body-frame vector into the world frame.
  Vec3 rotate(const Vec3& v) const {
    const Vec3 qv{x, y, z};
    const Vec3 t = qv.cross(v) * 2.0f;
    return v + t * w + qv.cross(t);
  }

  // Rotate a world-frame vector into the body frame.
  Vec3 rotateInv(const Vec3& v) const { return conj().rotate(v); }

  // First-order integration of body angular rate (rad/s) over dt.
  //Jacobian
  void integrate(const Vec3& omegaBody, float dt) {
    const Quat wq{0, omegaBody.x, omegaBody.y, omegaBody.z};
    const Quat qd = (*this) * wq;  // q̇ = ½ q ⊗ ω
    w += 0.5f * dt * qd.w;
    x += 0.5f * dt * qd.x;
    y += 0.5f * dt * qd.y;
    z += 0.5f * dt * qd.z;
    normalize();
  }

  // Shortest rotation taking direction `from` to direction `to`.
  static Quat fromTwoVectors(const Vec3& from, const Vec3& to) {
    const Vec3 f = from.normalized(), t = to.normalized();
    const float d = f.dot(t);
    if (d < -0.999999f) {
      // Antiparallel: rotate 180° about any axis perpendicular to f.
      Vec3 axis = Vec3{1, 0, 0}.cross(f);
      if (axis.norm() < 1e-6f) axis = Vec3{0, 1, 0}.cross(f);
      axis = axis.normalized();
      return {0, axis.x, axis.y, axis.z};
    }
    const Vec3 c = f.cross(t);
    Quat q{1.0f + d, c.x, c.y, c.z};
    q.normalize();
    return q;
  }

  // Rotation vector (axis * angle, rad) equivalent of this quaternion.
  Vec3 toRotVec() const {
    Quat q = *this;
    if (q.w < 0) { q.w = -q.w; q.x = -q.x; q.y = -q.y; q.z = -q.z; }
    const float s = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
    if (s < 1e-9f) return {0, 0, 0};
    const float ang = 2.0f * std::atan2(s, q.w);
    return Vec3{q.x, q.y, q.z} * (ang / s);
  }

  static Quat fromRotVec(const Vec3& r) {
    const float ang = r.norm();
    if (ang < 1e-9f) return identity();
    const float s = std::sin(0.5f * ang) / ang;
    return {std::cos(0.5f * ang), r.x * s, r.y * s, r.z * s};
  }
};
