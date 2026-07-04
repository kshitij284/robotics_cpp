#pragma once

#include "constants.hpp"
#include "mat3.hpp"
#include "vec3.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

namespace kinematics::math {

class SE3 {
  Mat3 R_ = Mat3::identity();
  Vec3 t_;

public:
  constexpr SE3() = default;
  constexpr explicit SE3(Mat3 mat) : R_(mat) {}
  constexpr explicit SE3(Vec3 t) : t_(t) {}
  constexpr SE3(Mat3 mat, Vec3 t) : R_(mat), t_(t) {}

  constexpr SE3(const SE3 &other) = default;
  constexpr SE3 &operator=(const SE3 &other) = default;
  constexpr SE3(SE3 &&other) noexcept = default;
  constexpr SE3 &operator=(SE3 &&other) noexcept = default;

  constexpr const Mat3 &rotation() const noexcept { return R_; }

  constexpr const Vec3 &translation() const noexcept { return t_; }

  constexpr static SE3 identity() { return SE3(); }

  constexpr static SE3 from_rotation(const Mat3 &mat) { return SE3(mat); }

  constexpr static SE3 from_translation(const Vec3 &vec) { return SE3(vec); }

  constexpr static SE3 from_quaternion_translation(const Quaternion &q,
                                                   const Vec3 &t) {
    Mat3 mat = q.to_rotation_matrix();
    return SE3(mat, t);
  }

  constexpr bool operator==(const SE3 &other) const noexcept {
    return ((R_ == other.R_) && (t_ == other.t_));
  }

  constexpr SE3 &operator*=(const SE3 &other) noexcept {
    const Mat3 original_R = R_;
    R_ = original_R * other.R_;
    t_ = (original_R * other.t_) + t_;
    return *this;
  }

  constexpr SE3 operator*(const SE3 &other) const noexcept {
    SE3 tmp = *this;
    tmp *= other;
    return tmp;
  }

  constexpr SE3 inverse() const noexcept {
    SE3 tmp;
    tmp.R_ = R_.transpose();
    tmp.t_ = -(R_.transpose() * t_);
    return tmp;
  }

  constexpr Vec3 act_on_point(const Vec3 &p) const noexcept {
    return Vec3{(R_ * p) + t_};
  }

  constexpr Vec3 act_on_vector(const Vec3 &vec) const noexcept {
    return Vec3{R_ * vec};
  }

  friend std::ostream &operator<<(std::ostream &os, SE3 se3) {
    os << "R[\n"
       << "  " << se3.R_[0, 0] << ", " << se3.R_[0, 1] << ", " << se3.R_[0, 2]
       << "\n"
       << "  " << se3.R_[1, 0] << ", " << se3.R_[1, 1] << ", " << se3.R_[1, 2]
       << "\n"
       << "  " << se3.R_[2, 0] << ", " << se3.R_[2, 1] << ", " << se3.R_[2, 2]
       << "\n]"
       << "t[ " << se3.t_.x << ", " << se3.t_.y << ", " << se3.t_.z << "]";
    return os;
  }
};

} // namespace kinematics::math
