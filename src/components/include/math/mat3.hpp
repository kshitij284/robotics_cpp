#pragma once

#include "constants.hpp"
#include "vec3.hpp"
#include <array>
#include <cassert>
#include <cmath>
#include <ostream>

namespace kinematics::math {

class Mat3 {
  std::array<double, 9> m{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

public:
  constexpr Mat3() = default;
  constexpr Mat3(double m00, double m01, double m02, double m10, double m11,
                 double m12, double m20, double m21, double m22)
      : m{m00, m01, m02, m10, m11, m12, m20, m21, m22} {};
  constexpr ~Mat3() = default;
  constexpr Mat3(const Mat3 &other) = default;
  constexpr Mat3 &operator=(const Mat3 &other) = default;
  constexpr Mat3(Mat3 &&other) noexcept = default;
  constexpr Mat3 &operator=(Mat3 &&other) noexcept = default;

  static constexpr Mat3 identity() noexcept {
    return Mat3{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  }

  static constexpr Mat3 zero() noexcept {
    return Mat3{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  }

  static constexpr Mat3 m90x() noexcept {
    return Mat3{1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0};
  }

  static constexpr Mat3 m90y() noexcept {
    return Mat3{0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0};
  }

  static constexpr Mat3 m90z() noexcept {
    return Mat3{0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  }

  constexpr double operator[](std::size_t i, std::size_t j) const noexcept {
    assert(i < 3 && j < 3 && "Mat3::subscript operator - Out of bound access");
    return m[i * 3 + j];
  }

  constexpr double &operator[](std::size_t i, std::size_t j) noexcept {
    assert(i < 3 && j < 3 && "Mat3::subscript operator - Out of bound access");
    return m[i * 3 + j];
  }

  constexpr bool operator==(const Mat3 &other) const noexcept {
    for (std::size_t i = 0; i < 9; i++) {
      if (std::abs(m[i] - other.m[i]) > kEpsilon)
        return false;
    }
    return true;
  }

  constexpr Mat3 &operator+=(const Mat3 &other) noexcept {
    for (std::size_t i = 0; i < 9; i++) {
      m[i] += other.m[i];
    }
    return *this;
  }

  constexpr Mat3 operator+(const Mat3 &other) const noexcept {
    Mat3 tmp = *this;
    tmp += other;
    return tmp;
  }

  constexpr Mat3 &operator-=(const Mat3 &other) noexcept {
    for (std::size_t i = 0; i < 9; i++) {
      m[i] -= other.m[i];
    }
    return *this;
  }

  constexpr Mat3 operator-(const Mat3 &other) const noexcept {
    Mat3 tmp = *this;
    tmp -= other;
    return tmp;
  }

  constexpr Mat3 &operator*=(double factor) noexcept {
    for (std::size_t i = 0; i < 9; i++)
      m[i] *= factor;
    return *this;
  }

  constexpr Mat3 operator*(double factor) const noexcept {
    Mat3 tmp = *this;
    tmp *= factor;
    return tmp;
  }

  constexpr Mat3 operator-() noexcept {
    Mat3 tmp = *this;
    for (std::size_t i = 0; i < 9; i++)
      tmp.m[i] *= -1.0;
    return tmp;
  }

  constexpr Mat3 &operator*=(const Mat3 &other) noexcept {
    Mat3 tmp = *this;
    *this = Mat3::zero();
    for (std::size_t i = 0; i < 3; i++) {
      for (std::size_t j = 0; j < 3; j++) {
        for (std::size_t k = 0; k < 3; k++) {
          m[i * 3 + j] += tmp[i, k] * other[k, j];
        }
      }
    }
    return *this;
  }

  constexpr Mat3 operator*(const Mat3 &other) const noexcept {
    Mat3 tmp = *this;
    tmp *= other;
    return tmp;
  }

  constexpr Mat3 &operator/=(double d) noexcept {
    assert(std::abs(d) > kEpsilon &&
           "Mat3::division operator - division by zero");
    for (std::size_t i = 0; i < 9; i++)
      m[i] /= d;
    return *this;
  }

  constexpr Mat3 operator/(double d) const noexcept {
    Mat3 tmp = *this;
    tmp /= d;
    return tmp;
  }

  constexpr Mat3 &transpose_inplace() noexcept {
    *this = Mat3{m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]};
    return *this;
  }

  constexpr Mat3 transpose() const noexcept {
    return Mat3{m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]};
  }

  constexpr bool is_orthogonal() const noexcept {
    return ((transpose() * (*this)) == identity());
  }

  constexpr bool is_symmetric() const noexcept {
    Mat3 tmp = *this;
    return tmp == transpose();
  }

  constexpr double determinant() const noexcept {
    return ((m[0] * m[4] * m[8]) - (m[0] * m[5] * m[7]) - (m[1] * m[3] * m[8]) +
            (m[1] * m[6] * m[5]) + (m[2] * m[3] * m[7]) - (m[2] * m[6] * m[4]));
  }

  friend std::ostream &operator<<(std::ostream &os, const Mat3 &mat) {
    os << "Mat3[\n"
       << "  " << mat[0, 0] << ", " << mat[0, 1] << ", " << mat[0, 2] << "\n"
       << "  " << mat[1, 0] << ", " << mat[1, 1] << ", " << mat[1, 2] << "\n"
       << "  " << mat[2, 0] << ", " << mat[2, 1] << ", " << mat[2, 2] << "\n]";
    return os;
  }
};

constexpr inline Mat3 operator*(double factor, Mat3 mat) noexcept {
  return mat *= factor;
}

constexpr inline Vec3 operator*(const Mat3 &mat, const Vec3 &vec) noexcept {
  return Vec3{
      (mat[0, 0] * vec[0]) + (mat[0, 1] * vec[1]) + (mat[0, 2] * vec[2]),
      (mat[1, 0] * vec[0]) + (mat[1, 1] * vec[1]) + (mat[1, 2] * vec[2]),
      (mat[2, 0] * vec[0]) + (mat[2, 1] * vec[1]) + (mat[2, 2] * vec[2])};
}
} // namespace kinematics::math
