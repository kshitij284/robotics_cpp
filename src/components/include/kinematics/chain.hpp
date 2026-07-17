#pragma once

#include "kinematics/joint.hpp"
#include <span>
#include <vector>

namespace kinematics::math {

class KinematicChain {
private:
  std::vector<Joint> chain_{};

public:
  constexpr KinematicChain() = default;
  constexpr KinematicChain(std::vector<Joint> chain)
      : chain_(std::move(chain)) {}
  constexpr KinematicChain(const KinematicChain &other) = default;
  constexpr KinematicChain &operator=(const KinematicChain &other) = default;
  constexpr KinematicChain(KinematicChain &&other) noexcept = default;
  constexpr KinematicChain &
  operator=(KinematicChain &&other) noexcept = default;

  [[nodiscard]] constexpr size_t joint_count() const noexcept {
    return chain_.size();
  }

  [[nodiscard]] constexpr size_t dof_count() const noexcept {
    size_t count = 0;
    for (const auto &joint : chain_) {
      if (std::holds_alternative<Fixed>(joint.type()))
        continue;
      count++;
    }
    return count;
  }

  constexpr void add_joint(Joint joint) { chain_.push_back(std::move(joint)); }

  constexpr SE3
  forward_kinematics(std::span<const double> q_vals) const noexcept {
    assert(q_vals.size() == dof_count() &&
           "KinematicChain: q count must match DOF");
    SE3 T_result = SE3::identity();
    size_t qi = 0;
    for (const auto &joint : chain_) {
      double val =
          std::holds_alternative<Fixed>(joint.type()) ? 0.0 : q_vals[qi++];
      T_result = T_result * joint.transform(val);
    }
    assert(q_vals.size() == qi &&
           "KinematicChain: values not applied correctly");
    return T_result;
  }
};

} // namespace kinematics::math
