#pragma once

#include "kinematics/jacobian.hpp"
#include "kinematics/joint.hpp"
#include <cassert>
#include <span>
#include <vector>

namespace kinematics::math {

class KinematicChain {
private:
  std::vector<Joint> chain_{};
  struct ChainFrames {
    std::vector<SE3> joint_frames;
    SE3 end_effector;
  };

  ChainFrames compute_frames(std::span<const double> q_vals) const {
    ChainFrames cf;
    assert(q_vals.size() == dof_count() &&
           "KinematicChain: q count must match DOF");
    cf.joint_frames.reserve(q_vals.size());

    SE3 T_result = SE3::identity();
    size_t qi = 0;
    for (const auto &joint : chain_) {
      double val = 0.0;
      if (std::holds_alternative<Fixed>(joint.type())) {
        T_result = T_result * joint.rest_transform();
        continue;
      } else {
        val = q_vals[qi++];
        cf.joint_frames.push_back(T_result * joint.rest_transform());
        T_result = T_result * joint.transform(val);
      }
    }
    assert(q_vals.size() == qi &&
           "KinematicChain: values not applied correctly");
    cf.end_effector = T_result;
    return cf;
  }

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

  SE3 forward_kinematics(std::span<const double> q_vals) const {
    return compute_frames(q_vals).end_effector;
  }

  Jacobian jacobian(std::span<const double> q_vals) const {
    ChainFrames cf = compute_frames(q_vals);
    Vec3 pe = cf.end_effector.translation();
    Jacobian j(q_vals.size());
    long qi = 0;
    for (const auto &joint : chain_) {
      if (std::holds_alternative<Revolute>(joint.type())) {
        auto revolute = std::get<Revolute>(joint.type());
        Vec3 zi =
            cf.joint_frames[static_cast<size_t>(qi)].rotation() * revolute.axis;
        Vec3 pi = cf.joint_frames[static_cast<size_t>(qi)].translation();
        Vec3 top = zi.cross(pe - pi);
        j[0, qi] = top[0];
        j[1, qi] = top[1];
        j[2, qi] = top[2];
        j[3, qi] = zi[0];
        j[4, qi] = zi[1];
        j[5, qi] = zi[2];
        qi++;
      } else if (std::holds_alternative<Prismatic>(joint.type())) {
        auto prismatic = std::get<Prismatic>(joint.type());
        Vec3 zi = cf.joint_frames[static_cast<size_t>(qi)].rotation() *
                  prismatic.axis;
        j[0, qi] = zi[0];
        j[1, qi] = zi[1];
        j[2, qi] = zi[2];
        j[3, qi] = 0.0;
        j[4, qi] = 0.0;
        j[5, qi] = 0.0;
        qi++;
      } else
        continue;
    }
    assert(static_cast<size_t>(qi) == q_vals.size() &&
           "KinematicChain : jacobian - order mismatch");
    return j;
  }
};

} // namespace kinematics::math
