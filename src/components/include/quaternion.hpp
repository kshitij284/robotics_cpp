#pragma once

#include "constants.hpp"
#include "mat3.hpp"
#include "vec3.hpp"
#include <cassert>
#include <cmath>
#include <ostream>

namespace kinematics::math {
class Quaternion {
public:
  double w = 1.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  constexpr Quaternion() = default;
  constexpr Quaternion(double w, double x, double y, double z)
      : w(w), x(x), y(y), z(z) {}
  constexpr Quaternion(const Quaternion &other) = default;
  constexpr Quaternion &operator=(const Quaternion &other) = default;
  constexpr Quaternion(Quaternion &&other) noexcept = default;
  constexpr Quaternion &operator=(Quaternion &&other) = default;
  constexpr ~Quaternion() = default;

  static constexpr Quaternion identity() noexcept {
    return Quaternion{1.0, 0.0, 0.0, 0.0};
  }

  static constexpr Quaternion from_axis_angle(Vec3 axis,
                                              double angle) noexcept {
    double w = std::cos(angle / 2);
    assert(axis.is_unit() &&
           "Quaternion::from_axis_angle - axis is not unit vector");
    axis *= std::sin(angle / 2);
    return Quaternion{w, axis.x, axis.y, axis.z};
  }

  static constexpr Quaternion from_matrix(Mat3 mat) noexcept {
    double tr = mat[0, 0] + mat[1, 1] + mat[2, 2];
    Quaternion q;

    if (tr > kEpsilon) {
      double S = std::sqrt(tr + 1.0) * 2.0;
      q.w = 0.25 * S;
      q.x = (mat[2, 1] - mat[1, 2]) / S;
      q.y = (mat[0, 2] - mat[2, 0]) / S;
      q.z = (mat[1, 0] - mat[0, 1]) / S;
    } else if ((mat[0, 0] > mat[1, 1]) && (mat[0, 0] > mat[2, 2])) {
      double S = std::sqrt(1.0 + mat[0, 0] - mat[1, 1] - mat[2, 2]) * 2.0;
      q.w = (mat[2, 1] - mat[1, 2]) / S;
      q.x = 0.25 * S;
      q.y = (mat[0, 1] + mat[1, 0]) / S;
      q.z = (mat[0, 2] + mat[2, 0]) / S;
    } else if (mat[1, 1] > mat[2, 2]) {
      double S = std::sqrt(1.0 + mat[1, 1] - mat[0, 0] - mat[2, 2]) * 2.0;
      q.w = (mat[0, 2] - mat[2, 0]) / S;
      q.x = (mat[0, 1] + mat[1, 0]) / S;
      q.y = 0.25 * S;
      q.z = (mat[1, 2] + mat[2, 1]) / S;
    } else {
      double S = std::sqrt(1.0 + mat[2, 2] - mat[0, 0] - mat[1, 1]) * 2.0;
      q.w = (mat[1, 0] - mat[0, 1]) / S;
      q.x = (mat[0, 2] + mat[2, 0]) / S;
      q.y = (mat[1, 2] + mat[2, 1]) / S;
      q.z = 0.25 * S;
    }
    return q;
  }

  constexpr bool operator==(const Quaternion &other) const noexcept {
    return std::abs(w - other.w) < kEpsilon &&
           std::abs(x - other.x) < kEpsilon &&
           std::abs(y - other.y) < kEpsilon && std::abs(z - other.z) < kEpsilon;
  }

  constexpr Quaternion &operator+=(const Quaternion &other) noexcept {
    w += other.w;
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
  }

  constexpr Quaternion operator+(const Quaternion &other) const noexcept {
    Quaternion tmp = *this;
    tmp += other;
    return tmp;
  }

  constexpr Quaternion &operator-=(const Quaternion &other) noexcept {
    w -= other.w;
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
  }

  constexpr Quaternion operator-(const Quaternion &other) const noexcept {
    Quaternion tmp = *this;
    tmp -= other;
    return tmp;
  }

  constexpr Quaternion operator-() const noexcept {
    return Quaternion{-w, -x, -y, -z};
  }

  constexpr Quaternion &operator*=(const Quaternion &other) noexcept {

    Vec3 v1{x, y, z};
    Vec3 v2{other.x, other.y, other.z};

    double rw = (w * other.w) - v1.dot(v2);
    Vec3 rv = (w * v2) + (other.w * v1) + v1.cross(v2);

    *this = Quaternion{rw, rv.x, rv.y, rv.z};
    return *this;
  }

  constexpr Quaternion operator*(const Quaternion &other) const noexcept {
    Quaternion tmp = *this;
    tmp *= other;
    return tmp;
  }

  constexpr Quaternion &operator*=(double factor) noexcept {
    w *= factor;
    x *= factor;
    y *= factor;
    z *= factor;
    return *this;
  }

  constexpr Quaternion operator*(double factor) const noexcept {
    Quaternion tmp = *this;
    tmp *= factor;
    return tmp;
  }

  constexpr Quaternion &operator/=(double d) noexcept {
    assert(std::abs(d) > kEpsilon &&
           "Quaternion:: operator /= - division by zero");
    w = w / d;
    x = x / d;
    y = y / d;
    z = z / d;
    return *this;
  }

  constexpr Quaternion operator/(double d) const noexcept {
    Quaternion tmp = *this;
    tmp /= d;
    return tmp;
  }

  constexpr Quaternion conjugate() const noexcept {
    return Quaternion{w, -x, -y, -z};
  }

  constexpr double norm() const noexcept {
    return std::sqrt(w * w + x * x + y * y + z * z);
  }

  constexpr Quaternion normalize() const noexcept {
    const double n = norm();
    assert((n > kEpsilon) && "Quaternion::normalize - zero length quaternion");
    return *this / n;
  }

  constexpr Quaternion inverse() const noexcept {
    const double n = norm();
    assert((n > kEpsilon) && "Quaternion::inverse - zero length quaternion");
    return conjugate() / (n * n);
  }

  constexpr double dot(const Quaternion &other) const noexcept {
    return (w * other.w + x * other.x + y * other.y + z * other.z);
  }

  constexpr Vec3 rotate(const Vec3 &vec) const noexcept {
    Quaternion vec_q(0, vec.x, vec.y, vec.z);
    Quaternion result = *this * vec_q * this->inverse();
    return Vec3{result.x, result.y, result.z};
  }

  constexpr Quaternion slerp(const Quaternion &other, double t) const noexcept {
    double dot_prd = this->dot(other);
    Quaternion other_corrected = other;
    if (dot_prd < 0.0) {
      other_corrected = -other;
      dot_prd = -dot_prd;
    }
    double omega = std::acos(std::clamp(dot_prd, -1.0, 1.0));
    if (omega < kEpsilon)
      return *this;
    return (*this * (std::sin((1.0 - t) * omega) / std::sin(omega))) +
           (other_corrected * (std::sin(t * omega) / std::sin(omega)));
  }

  constexpr Mat3 to_rotation_matrix() const noexcept {
    return Mat3{1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z),
                2.0 * (x * z + w * y),       2.0 * (x * y + w * z),
                1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x),
                2.0 * (x * z - w * y),       2.0 * (y * z + w * x),
                1.0 - 2.0 * (x * x + y * y)};
  }

  constexpr bool is_unit() const noexcept {
    const double len = std::sqrt(w * w + x * x + y * y + z * z);
    return std::abs(len - 1) < kEpsilon;
  }

  friend std::ostream &operator<<(std::ostream &os, const Quaternion &q) {
    os << "Quaternion(" << q.w << ", " << q.x << ", " << q.y << ", " << q.z
       << ")";
    return os;
  }
};

inline constexpr Quaternion operator*(double factor, Quaternion q) noexcept {
  return q * factor;
}

} // namespace kinematics::math
