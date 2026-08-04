#include "kinematics/chain.hpp"
#include "kinematics/iksolver.hpp"
#include "kinematics/joint.hpp"
#include "math/se3.hpp"
#include "math/vec3.hpp"
#include <gtest/gtest.h>

using namespace kinematics::math;

class IKSolverTest : public ::testing::Test {
protected:
  IKSolver iksolver_default;
  Vec3 axis_z;
  SE3 rest_transform;
  Joint revolute_joint;
  std::vector<Joint> vec_rev_joint;
  KinematicChain planar_chain;
  IKSolver iksolver_planar_rev;
  std::vector<double> q_seed;

  IKSolverTest()
      : axis_z(Vec3{0.0, 0.0, 1.0}),
        rest_transform(SE3(Mat3::identity(), Vec3{0.0, 5.0, 0.0})),
        revolute_joint("joint_rev", Revolute{axis_z}, rest_transform, -kPi,
                       kPi),
        vec_rev_joint(
            std::vector<Joint>{revolute_joint, revolute_joint, revolute_joint}),
        planar_chain(vec_rev_joint), iksolver_planar_rev(planar_chain),
        q_seed(std::vector<double>{kPi / 6, -kPi / 6, kPi / 6}) {}
};

TEST_F(IKSolverTest, Planar_Analytical_ComputesCorrectValue) {
  double x = 5 / std::sqrt(2);
  double y = 3 * x;
  auto result =
      iksolver_planar_rev.solve(SE3(Mat3::identity(), Vec3{x, y, 0.0}), q_seed);
  if (result) {
    std::vector<double> val = *result;
    EXPECT_NEAR(val[0], kPi / 4, 1e-4);
    EXPECT_NEAR(val[1], -kPi / 4, 1e-4);
    EXPECT_NEAR(val[2], kPi / 4, 1e-4);
  }
}

TEST_F(IKSolverTest, Planar_RoundTrip_ComputesCorrectValue) {
  std::vector<double> q_in{kPi / 3, kPi / 7, -kPi / 5};
  SE3 fk_result = planar_chain.forward_kinematics(q_in);
  auto result = iksolver_planar_rev.solve(fk_result, q_seed);
  if (result) {
    std::vector<double> q_out = *result;
    for (size_t i = 0; i < 3; i++) {
      EXPECT_NEAR(q_out[i], q_in[i], 1e-4);
    }
  }
}
