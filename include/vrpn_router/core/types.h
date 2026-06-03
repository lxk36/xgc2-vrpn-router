#pragma once

#include <algorithm>
#include <cmath>

namespace vrpn_router::core {

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Quat {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
};

struct Pose {
  Vec3 position;
  Quat orientation;
};

struct Transform {
  Vec3 translation;
  Quat rotation;
};

inline Vec3 operator+(const Vec3& a, const Vec3& b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

inline Vec3 operator-(const Vec3& a, const Vec3& b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

inline Vec3 operator*(double scale, const Vec3& value) {
  return {scale * value.x, scale * value.y, scale * value.z};
}

inline double dot(const Vec3& a, const Vec3& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vec3 cross(const Vec3& a, const Vec3& b) {
  return {
      a.y * b.z - a.z * b.y,
      a.z * b.x - a.x * b.z,
      a.x * b.y - a.y * b.x};
}

inline double norm(const Vec3& value) {
  return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

inline Quat normalized(Quat q) {
  const double n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (n <= 1e-12) {
    return {};
  }
  q.x /= n;
  q.y /= n;
  q.z /= n;
  q.w /= n;
  return q;
}

inline Quat multiply(const Quat& a_raw, const Quat& b_raw) {
  const Quat a = normalized(a_raw);
  const Quat b = normalized(b_raw);
  return normalized({
      a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
      a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z});
}

inline Quat inverse(const Quat& q_raw) {
  const Quat q = normalized(q_raw);
  return {-q.x, -q.y, -q.z, q.w};
}

inline Vec3 rotate(const Quat& q_raw, const Vec3& v) {
  const Quat q = normalized(q_raw);
  const Vec3 u{q.x, q.y, q.z};
  return (2.0 * dot(u, v)) * u +
         (q.w * q.w - dot(u, u)) * v +
         (2.0 * q.w) * cross(u, v);
}

inline double distance(const Pose& a, const Pose& b) {
  return norm(a.position - b.position);
}

inline double angleDeg(const Quat& a_raw, const Quat& b_raw) {
  const Quat a = normalized(a_raw);
  const Quat b = normalized(b_raw);
  const double dot = std::abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
  const double clamped = std::max(-1.0, std::min(1.0, dot));
  return 2.0 * std::acos(clamped) * 180.0 / M_PI;
}

}  // namespace vrpn_router::core
