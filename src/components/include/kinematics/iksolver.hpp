#pragma once
#include "kinematics/chain.hpp"
#include "math/se3.hpp"
#include "math/vec3.hpp"
#include <algorithm>
#include <expected>
#include <span>

namespace kinematics::math {

enum class IKError { MaxIterationsReached, OutOfBound, Singularity };

class IKSolver {
private:
  KinematicChain chain_;
  double lambda_ = 30;
  double pos_tolerance_ = 1e-2;
  double ori_tolerance_ = 1e-2;
  int max_iter_ = 500;

public:
  IKSolver(const KinematicChain &chain, double lambda, double pos_tolerance,
           double ori_tolerance, int max_iter)
      : chain_(chain), lambda_(lambda), pos_tolerance_(pos_tolerance),
        ori_tolerance_(ori_tolerance), max_iter_(max_iter) {}

  IKSolver(const KinematicChain &chain) : chain_(chain) {}
  IKSolver() = default;

  Eigen::Vector<double, 6> pose_error(const SE3 &target,
                                      const SE3 &current) const {
    Eigen::Vector<double, 6> result;
    Vec3 err = target.translation() - current.translation();
    result << err[0], err[1], err[2], 0.0, 0.0, 0.0;
    return result;
  }

  std::expected<std::vector<double>, IKError>
  solve(const SE3 &target, std::span<const double> q_seed) const {
    std::vector<double> q(q_seed.begin(), q_seed.end());
    size_t q_size = q.size();
    double lambda_sqr = lambda_ * lambda_;

    for (int i = 0; i < max_iter_; i++) {
      SE3 current = chain_.forward_kinematics(q);
      Eigen::Vector<double, 6> err = pose_error(target, current);

      bool flag = true;
      for (const auto &val : err) {
        if (std::abs(val) > pos_tolerance_)
          flag = false;
      }
      if (flag)
        return q;

      Eigen::MatrixXd j = chain_.jacobian(q).matrix();
      Eigen::MatrixXd j_transpose = j.transpose();
      Eigen::VectorXd delta_q =
          j_transpose *
          ((j * j_transpose) + (lambda_sqr * Eigen::MatrixXd::Identity(6, 6)))
              .ldlt()
              .solve(err);

      size_t q_count = 0;
      while (q_count < q_size) {
        Joint joint = chain_.current_joint(q_count);
        if (std::holds_alternative<Fixed>(joint.type()))
          continue;
        double q_updated = q[q_count] + delta_q(static_cast<long>(q_count));
        q_updated = std::clamp(q_updated, joint.min_q(), joint.max_q());
        q[q_count] = q_updated;
        q_count++;
      }
    }
    return std::unexpected(IKError::MaxIterationsReached);
  }
};

} // namespace kinematics::math
