#include "kinematics/chain.hpp"
#include "kinematics/iksolver.hpp"
#include "kinematics/joint.hpp"
#include "math/se3.hpp"
#include "math/vec3.hpp"
#include <gtest/gtest.h>

using namespace kinematics::math;

class IKSolverTest : public ::testing::Test {
protected:
  Vec3 axis_z;
  SE3 rest_transform;
  SE3 rest_transform_rot90y_trans5x;
  Joint revolute_joint;
  Joint prismatic_joint;
  Joint fixed_joint;
  std::vector<Joint> vec_rev_joint;
  std::vector<Joint> vec_mixed_joints;
  KinematicChain planar_chain;
  KinematicChain mixed_chain;
  IKSolver iksolver_planar_rev;
  IKSolver iksolver_mixed_chain;
  std::vector<double> q_seed;
  std::vector<double> q_seed_mixed;

  IKSolverTest()
      : axis_z(Vec3{0.0, 0.0, 1.0}),
        rest_transform(SE3(Mat3::identity(), Vec3{0.0, 5.0, 0.0})),
        rest_transform_rot90y_trans5x(SE3(Mat3::m90y(), Vec3{5.0, 0.0, 0.0})),
        revolute_joint("joint_rev", Revolute{axis_z}, rest_transform, -kPi,
                       kPi),
        prismatic_joint("joint_prim", Prismatic{axis_z},
                        rest_transform_rot90y_trans5x, -4.0, 7.0),
        fixed_joint("joint_fixed", Fixed{}, rest_transform, 0.0, 0.0),
        vec_rev_joint(
            std::vector<Joint>{revolute_joint, revolute_joint, revolute_joint}),
        vec_mixed_joints(std::vector<Joint>{revolute_joint, fixed_joint,
                                            revolute_joint, prismatic_joint,
                                            fixed_joint}),
        planar_chain(vec_rev_joint), mixed_chain(vec_mixed_joints),
        iksolver_planar_rev(planar_chain), iksolver_mixed_chain(mixed_chain),
        q_seed(std::vector<double>{kPi / 6, -kPi / 6, kPi / 6}),
        q_seed_mixed(std::vector<double>{kPi / 3 + 0.1, kPi / 7 + 0.1, 1.1}) {}
};

TEST_F(IKSolverTest, Planar_Analytical_ComputesCorrectValue) {
  double x = 5 / std::sqrt(2);
  double y = 3 * x;
  SE3 target(Mat3::identity(), Vec3{x, y, 0.0});
  auto result = iksolver_planar_rev.solve(target, q_seed);
  ASSERT_TRUE(result);
  if (result) {
    std::vector<double> q_out = *result;
    Vec3 t_out = planar_chain.forward_kinematics(q_out).translation();
    Vec3 t_in = target.translation();
    for (size_t i = 0; i < 3; i++) {
      EXPECT_NEAR(t_in[i], t_out[i], 1e-2);
    }
    Mat3 m_out = planar_chain.forward_kinematics(q_out).rotation();
    Mat3 m_in = target.rotation();
    for (std::size_t i = 0; i < 3; i++) {
      for (std::size_t j = 0; j < 3; j++) {
        EXPECT_NEAR((m_out[i, j]), (m_in[i, j]), 1e-2);
      }
    }
  }
}
TEST_F(IKSolverTest, Planar_RoundTrip_ComputesCorrectValue) {
  std::vector<double> q_in{kPi / 3, kPi / 7, -kPi / 5};
  SE3 fk_result = planar_chain.forward_kinematics(q_in);
  auto result = iksolver_planar_rev.solve(fk_result, q_seed);
  ASSERT_TRUE(result);
  if (result) {
    std::vector<double> q_out = *result;
    Vec3 t_out = planar_chain.forward_kinematics(q_out).translation();
    Vec3 t_in = fk_result.translation();
    for (size_t i = 0; i < 3; i++) {
      EXPECT_NEAR(t_out[i], t_in[i], 1e-2);
    }
    Mat3 m_out = planar_chain.forward_kinematics(q_out).rotation();
    Mat3 m_in = fk_result.rotation();
    for (std::size_t i = 0; i < 3; i++) {
      for (std::size_t j = 0; j < 3; j++) {
        EXPECT_NEAR((m_out[i, j]), (m_in[i, j]), 1e-2);
      }
    }
  }
}

TEST_F(IKSolverTest, Mixed_RoundTrip_ComputesCorrectValue) {
  std::vector<double> q_in{kPi / 3, kPi / 7, 1.0};
  SE3 fk_result = mixed_chain.forward_kinematics(q_in);
  auto result = iksolver_mixed_chain.solve(fk_result, q_seed_mixed);
  ASSERT_TRUE(result);
  if (result) {
    std::vector<double> q_out = *result;
    Vec3 t_out = mixed_chain.forward_kinematics(q_out).translation();
    Vec3 t_in = fk_result.translation();
    for (size_t i = 0; i < 3; i++) {
      EXPECT_NEAR(t_out[i], t_in[i], 1e-2);
    }
    Mat3 m_out = mixed_chain.forward_kinematics(q_out).rotation();
    Mat3 m_in = fk_result.rotation();
    for (std::size_t i = 0; i < 3; i++) {
      for (std::size_t j = 0; j < 3; j++) {
        EXPECT_NEAR((m_out[i, j]), (m_in[i, j]), 1e-2);
      }
    }
  }
}
