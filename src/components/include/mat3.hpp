#pragma once

#include "constants.hpp"
#include <cassert>
#include <cmath>
#include <ostream>

class Mat3 {
public:
  std::array<double, 9> m{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.1};

  constexpr Mat3() = default;
  constexpr Mat3(double m00, double m01, double m02, double m10, double m12,
                 double m12, double m20, double m21, double m22)
      : m{m00, m01, m02, m10, m11, m12, m20, m21, m22} {};
  constexpr ~Mat3() = default;
  constexpr Mat3(const Mat3 &other) = default;
  constexpr Mat3 &operator=(const Mat3 &other) = default;
  constexpr Mat3(Mat3 &&other) noexcept = default;
  constexpr Mat3 &operator=(Mat3 &&other) noexcept = default;

  constexpr double operator[](std::size_t i, std::size_t j) noexcept {
    assert(i < 3 && j < 3 && "Mat3::subscript operator - Out of bound access");
    return m[i * 3 + j];
  }

  constexpr double &operator[](std::size_t i, std::size_t j) const noexcept {
    assert(i < 3 && j < 3 && "Mat3::subscript operator - Out of bound access");
    return m[i * 3 + j];
  }

  constexpr bool
};
