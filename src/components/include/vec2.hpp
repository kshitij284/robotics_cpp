#pragma once
#include "constants.hpp"
#include <cassert>
#include <cmath>

namespace kinematics::math {

class Vec2 {
public:
  double x = 0.0;
  double y = 0.0;

  constexpr Vec2() = default;
  constexpr Vec2(double x_val, double y_val) : x(x_val), y(y_val) {}

  constexpr bool operator==(const Vec2 &other) const {
    return (std::abs(x - other.x) < kEpsilon) &&
           (std::abs(y - other.y) < kEpsilon);
  };

  constexpr double &operator[](size_t i) noexcept {
    assert(i < 2 && "Vec2::subscript - Out of bound access");
    return (i == 0) ? x : y;
  }

  constexpr double operator[](size_t i) const noexcept {
    assert(i < 2 && "Vec2::subscript - Out of bound access");
    return (i == 0) ? x : y;
  }

  constexpr Vec2 &operator+=(const Vec2 &other) noexcept {
    x += other.x;
    y += other.y;
    return *this;
  }

  constexpr Vec2 operator+(const Vec2 &other) const noexcept {
    Vec2 temp = *this;
    temp += other;
    return temp;
  }

  constexpr Vec2 &operator-=(const Vec2 &other) noexcept {
    x -= other.x;
    y -= other.y;
    return *this;
  }

  constexpr Vec2 operator-(const Vec2 &other) const noexcept {
    Vec2 tmp = *this;
    tmp -= other;
    return tmp;
  }

  constexpr Vec2 &operator*=(double m) noexcept {
    x *= m;
    y *= m;
    return *this;
  }

  constexpr Vec2 operator*(double m) const noexcept {
    Vec2 tmp = *this;
    tmp *= m;
    return tmp;
  }

  constexpr Vec2 &operator/=(double d) noexcept {
    assert(std::abs(d) > kEpsilon && "Vec2::Division - division by zero");
    x /= d;
    y /= d;
    return *this;
  }

  constexpr Vec2 operator/(double d) const noexcept {
    Vec2 tmp = *this;
    tmp /= d;
    return tmp;
  }

  double norm() const noexcept { return std::sqrt((x * x) + (y * y)); }

  Vec2 normalize() const noexcept {
    double n = norm();
    assert((n > kEpsilon) && "Vec2::Normalize - division by zero");
    return *this / n;
  }

  constexpr double dot(const Vec2 &other) const noexcept {
    return ((x * other.x) + (y * other.y));
  }

  constexpr double determinant(const Vec2 &other) const noexcept {
    return ((x * other.y) - (y * other.x));
  }
};

inline Vec2 operator*(double m, const Vec2 &vec) noexcept { return vec * m; }

} // namespace kinematics::math
