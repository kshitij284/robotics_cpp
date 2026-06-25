#pragma once

#include "constants.hpp"
#include <cassert>
#include <cmath>
#include <limits>
#include <ostream>

namespace kinematics::math {

class Vec3 {
public:
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  constexpr Vec3() = default;
  constexpr Vec3(double x, double y, double z) : x(x), y(y), z(z) {}
  constexpr Vec3(const Vec3 &other) = default;
  constexpr Vec3 &operator=(const Vec3 &other) = default;
  constexpr Vec3(Vec3 &&other) noexcept = default;
  constexpr Vec3 &operator=(Vec3 &&other) noexcept = default;
  constexpr ~Vec3() = default;

  constexpr bool operator==(const Vec3 &other) const noexcept {
    return std::abs(x - other.x) < kEpsilon &&
           std::abs(y - other.y) < kEpsilon && std::abs(z - other.z) < kEpsilon;
  }

  constexpr double &operator[](size_t i) noexcept {
    assert(i < 3 && "Vec3::subscript operator - Out of bound access");
    return (i == 0) ? x : (i == 1) ? y : z;
  }

  constexpr double operator[](size_t i) const noexcept {
    assert(i < 3 && "Vec3::subscript operator - Out of bound ");
    return (i == 0) ? x : (i == 1) ? y : z;
  }

  constexpr Vec3 &operator+=(const Vec3 &other) noexcept {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
  }

  constexpr Vec3 operator+(const Vec3 &other) const noexcept {
    Vec3 tmp = *this;
    tmp += other;
    return tmp;
  }

  constexpr Vec3 &operator-=(const Vec3 &other) noexcept {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
  }

  constexpr Vec3 operator-() const noexcept {
    return Vec3{-x, -y, -z}; // unary minus
  }

  constexpr Vec3 operator-(const Vec3 &other) const noexcept {
    Vec3 tmp = *this;
    tmp -= other;
    return tmp;
  }

  constexpr Vec3 &operator*=(double m) noexcept {
    x *= m;
    y *= m;
    z *= m;
    return *this;
  }

  constexpr Vec3 operator*(double m) const noexcept {
    Vec3 tmp = *this;
    tmp *= m;
    return tmp;
  }

  constexpr Vec3 &operator/=(double d) noexcept {
    assert((std::abs(d) > kEpsilon) && "Vec3::operator/= - division by zero");
    x /= d;
    y /= d;
    z /= d;
    return *this;
  }

  constexpr Vec3 operator/(double d) const noexcept {
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

  constexpr bool is_unit() const noexcept {
    const double squared_norm = (x * x) + (y * y) + (z * z);
    return std::abs(squared_norm - 1.0) < kEpsilon;
  }

  constexpr double dot(const Vec3 &other) const noexcept {
    return (x * other.x) + (y * other.y) + (z * other.z);
  }

  constexpr Vec3 cross(const Vec3 &other) const noexcept {
    return Vec3{y * other.z - z * other.y, z * other.x - x * other.z,
                x * other.y - y * other.x};
  }

  friend std::ostream &operator<<(std::ostream &os, const Vec3 &v) {
    os << "Vec3(" << v.x << ", " << v.y << ", " << v.z << ")";
    return os;
  }
};

constexpr inline Vec3 operator*(double m, Vec3 other) {
  other *= m;
  return other;
}

} // namespace kinematics::math
