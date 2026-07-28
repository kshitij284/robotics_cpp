#pragma once

#include "math/se3.hpp"
#include <Eigen/Dense>
#include <cassert>
#include <vector>

namespace kinematics::math {

class Jacobian {
private:
  Eigen::MatrixXd em;

public:
  Jacobian() = default;
  Jacobian(std::size_t chain_dof) : em(6, chain_dof) {}
  constexpr Jacobian(const Jacobian &other) = default;
  constexpr Jacobian &operator=(const Jacobian &other) = default;
  constexpr Jacobian(Jacobian &&other) = default;
  constexpr Jacobian &operator=(Jacobian &&other) = default;
  constexpr ~Jacobian() = default;

  constexpr double operator[](long r, long c) const {
    assert(r < em.rows() && c < em.cols() &&
           "Jacobian::subscript operator - out of bound access");
    return em(r, c);
  }

  constexpr double &operator[](long r, long c) {
    assert(r < em.rows() && c < em.cols() &&
           "Jacobian::subscript operator - out of bound access");
    return em(r, c);
  }

  constexpr long rows() const { return em.rows(); }

  constexpr long cols() const { return em.cols(); }

  constexpr bool operator==(const Jacobian &other) const {
    return em == other.em;
  }
};

} // namespace kinematics::math
