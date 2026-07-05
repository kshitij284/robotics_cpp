#pragma once

#include "math/constants.hpp"
#include "math/mat3.hpp"
#include "math/quaternion.hpp"
#include "math/se3.hpp"
#include "math/vec3.hpp"
#include <ostream>
#include <variant>

namespace kinematics::math {

struct Revolute {
  Vec3 axis;
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
  double min_q_ = -kPi;
  double max_q_ = kPi;

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

  SE3 transform(double q) const {
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
