#pragma once

#include "math/constants.hpp"
#include "math/mat3.hpp"
#include "math/quaternion.hpp"
#include "math/se3.hpp"
#include "math/vec3.hpp"
#include <algorithm>
#include <ostream>
#include <string>
#include <variant>

namespace kinematics::math {

struct Revolute {
  Vec3 axis;
  Revolute(Vec3 axis_in) : axis(axis_in) {
    assert(axis.is_unit() && "Revolute: Axis must be unit vector");
  }
};
struct Prismatic {
  Vec3 axis;
};
struct Fixed {};

using JointType = std::variant<Revolute, Prismatic, Fixed>;

class Joint {
private:
  std::string name_ = "joint";
  JointType type_ = Fixed{};
  SE3 rest_transform_ = SE3::identity();
  // default limits are 0.0, 0.0. Callers constructing Revolute/Prismatic must
  // provide explicit limits
  double min_q_ = 0.0;
  double max_q_ = 0.0;

public:
  constexpr Joint() = default;
  constexpr Joint(std::string name, JointType type, SE3 rest_transform,
                  double min_q, double max_q)
      : name_(std::move(name)), type_(std::move(type)),
        rest_transform_(std::move(rest_transform)), min_q_(min_q),
        max_q_(max_q) {}
  constexpr Joint(const Joint &other) = default;
  constexpr Joint &operator=(const Joint &other) = default;
  constexpr Joint(Joint &&other) noexcept = default;
  constexpr Joint &operator=(Joint &&other) noexcept = default;

  [[nodiscard]] constexpr const std::string &name() const noexcept {
    return name_;
  };
  [[nodiscard]] constexpr const JointType &type() const noexcept {
    return type_;
  };
  [[nodiscard]] constexpr const SE3 &rest_transform() const noexcept {
    return rest_transform_;
  };
  [[nodiscard]] constexpr double min_q() const noexcept { return min_q_; };
  [[nodiscard]] constexpr double max_q() const noexcept { return max_q_; };

  SE3 transform(double q) const {
    // values are clamped to limits before transform is applied
    q = std::clamp(q, min_q_, max_q_);
    SE3 result = std::visit(
        [&](auto &&joint_type) -> SE3 {
          using T = std::decay_t<decltype(joint_type)>;
          if constexpr (std::is_same_v<T, Revolute>) {
            return SE3(Quaternion::from_axis_angle(joint_type.axis, q)
                           .to_rotation_matrix());
          } else if constexpr (std::is_same_v<T, Prismatic>) {
            return SE3::from_translation(joint_type.axis * q);
          } else if constexpr (std::is_same_v<T, Fixed>) {
            return SE3::identity();
          }
        },
        type_);
    return rest_transform_ * result;
  }
};

} // namespace kinematics::math
