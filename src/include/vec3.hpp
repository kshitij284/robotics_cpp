#pragma once

#include "constants.hpp"
#include <cassert>
#include <cmath>
#include <ostream>

namespace kinematics::math {

class Vec3 {
public:
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  Vec3() = default;
  Vec3(double x, double y, double z) : x(x), y(y), z(z) {}
  Vec3(const Vec3 &other) = default;
  Vec3 &operator=(const Vec3 &other) = default;
  Vec3(Vec3 &&other) noexcept = default;
  Vec3 &operator=(Vec3 &&other) noexcept = default;
  ~Vec3() = default;

  bool operator==(const Vec3 &other) const noexcept {
    return std::abs(x - other.x) < kEpsilon &&
           std::abs(y - other.y) < kEpsilon && std::abs(z - other.z) < kEpsilon;
  }

  Vec3 &operator+=(const Vec3 &other) noexcept {
    x = x + other.x;
    y = y + other.y;
    z = z + other.z;
    return *this;
  }

  Vec3 operator+(const Vec3 &other) const noexcept {
    Vec3 tmp = *this;
    tmp += other;
    return tmp;
  }

  Vec3 &operator-=(const Vec3 &other) noexcept {
    x = x - other.x;
    y = y - other.y;
    z = z - other.z;
    return *this;
  }

  Vec3 operator-() const noexcept {
    return Vec3{-x, -y, -z}; // unary minus
  }

  Vec3 operator-(const Vec3 &other) const noexcept {
    Vec3 tmp = *this;
    tmp -= other;
    return tmp;
  }

  Vec3 &operator*=(double m) noexcept {
    x = x * m;
    y = y * m;
    z = z * m;
    return *this;
  }

  Vec3 operator*(double m) const noexcept {
    Vec3 tmp = *this;
    tmp *= m;
    return tmp;
  }

  Vec3 &operator/=(double d) noexcept {
    assert((std::abs(d) > kEpsilon) && "Vec3::operator/= - division by zero");
    x = x / d;
    y = y / d;
    z = z / d;
    return *this;
  }

  // assert here is debug guard
  Vec3 operator/(double d) const noexcept {
    assert((std::abs(d) > kEpsilon) && "Vec3::operator/ - division by zero");
    Vec3 tmp = *this;
    tmp /= d;
    return tmp;
  }

  double norm() const noexcept {
    return std::sqrt((x * x) + (y * y) + (z * z));
  }

  // assert here is debug guard
  Vec3 normalize() const noexcept {
    const double n = norm();
    assert((n > kEpsilon) && "Vec3::normalize - zero-length vector");
    return *this / n;
  }

  double dot(const Vec3 &other) const noexcept {
    return (x * other.x) + (y * other.y) + (z * other.z);
  }

  Vec3 cross(const Vec3 &other) const noexcept {
    return Vec3{y * other.z - z * other.y, z * other.x - x * other.z,
                x * other.y - y * other.x};
  }

  friend std::ostream &operator<<(std::ostream &os, const Vec3 &v) {
    os << "Vec3(" << v.x << ", " << v.y << ", " << v.z << ")";
    return os;
  }
};

inline Vec3 operator*(double m, const Vec3 &other) {
  Vec3 tmp = other;
  tmp *= m;
  return tmp;
}

} // namespace kinematics::math
