#pragma once

#include "constants.hpp"
#include <cassert>
#include <cmath>
#include <ostream>

namespace kinematics::math {

class Mat3 {
public:
  double m[3][3]{};

  constexpr Mat3() = default;
  constexpr Mat3(double m00, double m01, double m02, double m10, double m11,
                 double m12, double m20, double m21, double m22)
      : m{{m00, m01, m02}, {m10, m11, m12}, {m20, m21, m22}} {}

  constexpr Mat3(const Mat3 &other) = default;
  constexpr Mat3 &operator=(const Mat3 &other) = default;
  constexpr Mat3(Mat3 &&other) noexcept = default;
  constexpr Mat3 &operator=(Mat3 &&other) noexcept = default;
  constexpr ~Mat3() = default;

  constexpr double operator[](std::size_t i, std::size_t j) const noexcept {
    assert(i < 3 && j < 3 && "Mat3::subscript operator - Out of bound access");
    return m[i][j];
  }

  constexpr double &operator[](std::size_t i, std::size_t j) noexcept {
    assert(i < 3 && j < 3 && "Mat3::subscript operator - Out of bound access");
    return m[i][j];
  }

  constexpr const double *operator[](std::size_t i) const noexcept {
    assert(i < 3 && "Mat3::subscript operator - Out of bound access");
    return m[i];
  }

  constexpr double *operator[](std::size_t i) noexcept {
    assert(i < 3 && "Mat3::subscript operator - Out of bound access");
    return m[i];
  }

  constexpr bool operator==(const Mat3 &other) const noexcept {

    const double *this_flat = &m[0][0];
    const double *other_flat = &other.m[0][0];

    for (std::size_t i = 0; i < 9; i++) {
      if (std::abs(this_flat[i] - other_flat[i]) > kEpsilon)
        return false;
    }
    return true;
  }

  constexpr Mat3 &operator+=(const Mat3 &other) noexcept {

    double *this_flat = &m[0][0];
    const double *other_flat = &other.m[0][0];

    for (std::size_t i = 0; i < 9; i++) {
      this_flat[i] += other_flat[i];
    }
    return *this;
  }

  constexpr Mat3 operator+(const Mat3 &other) const noexcept {
    Mat3 tmp = *this;
    tmp += other;
    return tmp;
  }

  constexpr Mat3 &operator-=(const Mat3 &other) noexcept {
    double *this_flat = &m[0][0];
    const double *other_flat = &other.m[0][0];

    for (std::size_t i = 0; i < 9; i++) {
      this_flat[i] -= other_flat[i];
    }
    return *this;
  }

  constexpr Mat3 operator-(const Mat3 &other) const noexcept {
    Mat3 tmp = *this;
    tmp -= other;
    return tmp;
  }

  constexpr Mat3 &operator*=(double factor) noexcept {
    double *this_flat = &m[0][0];
    for (std::size_t i = 0; i < 9; i++)
      this_flat[i] *= factor;
    return *this;
  }

  constexpr Mat3 operator*(double factor) const noexcept {
    Mat3 tmp = *this;
    tmp *= factor;
    return tmp;
  }

  constexpr Mat3 &operator/=(double d) noexcept {
    assert(std::abs(d) > kEpsilon &&
           "Mat3::division operator - division by zero");
    double *this_flat = &m[0][0];
    for (std::size_t i = 0; i < 9; i++)
      this_flat[i] /= d;
    return *this;
  }

  constexpr Mat3 operator/(double d) const noexcept {
    Mat3 tmp = *this;
    tmp /= d;
    return tmp;
  }

  friend std::ostream &operator<<(std::ostream &os, const Mat3 &mat) {
    os << "Mat3[\n"
       << "  " << mat.m[0][0] << ", " << mat.m[0][1] << ", " << mat.m[0][2]
       << "\n"
       << "  " << mat.m[1][0] << ", " << mat.m[1][1] << ", " << mat.m[1][2]
       << "\n"
       << "  " << mat.m[2][0] << ", " << mat.m[2][1] << ", " << mat.m[2][2]
       << "\n]";

    return os;
  }
};

constexpr inline Mat3 operator*(double factor, Mat3 mat) noexcept {
  return mat *= factor;
}

} // namespace kinematics::math
