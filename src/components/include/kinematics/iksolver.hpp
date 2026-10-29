#pragma once
#include "kinematics/chain.hpp"
#include "math/se3.hpp"
#include "math/vec3.hpp"
#include <algorithm>
#include <expected>
#include <span>

namespace kinematics::math {

enum class IKError { NotConverged };

class IKSolver {
private:
  const KinematicChain &chain_;
  double lambda_ = 1e-2;
  double pos_tolerance_ = 1e-2;
  double ori_tolerance_ = 1e-2;
  int max_iter_ = 200;

public:
  IKSolver(const KinematicChain &chain, double lambda, double pos_tolerance,
           double ori_tolerance, int max_iter)
      : chain_(chain), lambda_(lambda), pos_tolerance_(pos_tolerance),
        ori_tolerance_(ori_tolerance), max_iter_(max_iter) {}

  IKSolver(const KinematicChain &chain) : chain_(chain) {}

  Eigen::Vector<double, 6> pose_error(const SE3 &target,
                                      const SE3 &current) const {
    Eigen::Vector<double, 6> result;
    Vec3 err_pos = target.translation() - current.translation();

    Mat3 err_ori = (target.rotation()) * (current.rotation().transpose());
    Quaternion q = Quaternion::from_matrix(err_ori);
    if (q.w < 0.0) {
      q.w = -q.w;
      q.x = -q.x;
      q.y = -q.y;
      q.z = -q.z;
    }

    Vec3 vec_err_ori{q.x, q.y, q.z};
    double vec_norm = vec_err_ori.norm();
    if (vec_norm < 1e-6) {

      result << err_pos[0], err_pos[1], err_pos[2], vec_err_ori.x * 2,
          vec_err_ori.y * 2, vec_err_ori.z * 2;
      return result;
    }
    double angle = 2.0 * std::atan2(vec_norm, q.w);
    vec_err_ori = vec_err_ori * (angle / vec_norm);
    result << err_pos[0], err_pos[1], err_pos[2], vec_err_ori.x, vec_err_ori.y,
        vec_err_ori.z;
    return result;
  }

  std::expected<std::vector<double>, IKError>
  solve(const SE3 &target, std::span<const double> q_seed) const {
    std::vector<double> q(q_seed.begin(), q_seed.end());
    double lambda_sqr = lambda_ * lambda_;

    for (int i = 0; i < max_iter_; i++) {
      SE3 current = chain_.forward_kinematics(q);
      Eigen::Vector<double, 6> err = pose_error(target, current);
      bool flag = true;
      for (size_t e = 0; e < 3; e++) {
        if (std::abs(err(static_cast<long>(e))) > pos_tolerance_)
          flag = false;
      }
      for (size_t e = 3; e < 6; e++) {
        if (std::abs(err(static_cast<long>(e))) > ori_tolerance_)
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
      size_t joint_count = 0;
      while (joint_count < chain_.joint_count()) {
        Joint joint = chain_.current_joint(joint_count);
        if (std::holds_alternative<Fixed>(joint.type())) {
          joint_count++;
          continue;
        }
        double q_updated = q[q_count] + delta_q(static_cast<long>(q_count));
        q_updated = std::clamp(q_updated, joint.min_q(), joint.max_q());
        q[q_count] = q_updated;
        q_count++;
        joint_count++;
      }
    }
    return std::unexpected(IKError::NotConverged);
  }
};

} // namespace kinematics::math
