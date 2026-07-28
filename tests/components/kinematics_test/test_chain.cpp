#include "kinematics/chain.hpp"
#include "kinematics/jacobian.hpp"
#include <gtest/gtest.h>

using namespace kinematics::math;

class KinematicChainTest : public ::testing::Test {
protected:
  KinematicChain default_chain;
  Vec3 axis_x;
  Vec3 axis_y;
  Vec3 axis_z;
  SE3 rest_transform_planar;
  SE3 rest_transform_rot90x_tran5y;
  Joint fixed_joint;
  Joint revolute_joint;
  Joint revo_planar_joint;
  Joint prismatic_joint;
  std::vector<Joint> vec_fix_joint;
  std::vector<Joint> vec_rev_joint;
  std::vector<Joint> vec_rev_planar;
  std::vector<Joint> vec_pris_joint;
  std::vector<Joint> vec_mixed_joint;
  KinematicChain fixed_chain;
  KinematicChain revolute_chain;
  KinematicChain revo_planar_chain;
  KinematicChain prismatic_chain;
  KinematicChain mixed_chain;

  KinematicChainTest()
      : axis_x(Vec3{1.0, 0.0, 0.0}), axis_y(Vec3{0.0, 1.0, 0.0}),
        axis_z(Vec3{0.0, 0.0, 1.0}),
        rest_transform_planar(SE3(Mat3::identity(), Vec3{0.0, 5.0, 0.0})),
        rest_transform_rot90x_tran5y(SE3(Mat3::m90x(), Vec3{0.0, 5.0, 0.0})),
        fixed_joint("joint_fixed", Fixed{}, rest_transform_rot90x_tran5y, 0.0,
                    0.0),
        revolute_joint("joint_rev", Revolute{axis_z},
                       rest_transform_rot90x_tran5y, -kPi, kPi),
        revo_planar_joint("joint_rev_planar", Revolute{axis_z},
                          rest_transform_planar, -kPi, kPi),
        prismatic_joint("joint_prismatic", Prismatic{axis_z},
                        rest_transform_rot90x_tran5y, -10.0, 10.0),
        vec_fix_joint(
            std::vector<Joint>{fixed_joint, fixed_joint, fixed_joint}),
        vec_rev_joint(
            std::vector<Joint>{revolute_joint, revolute_joint, revolute_joint}),
        vec_rev_planar(std::vector<Joint>{revo_planar_joint, revo_planar_joint,
                                          revo_planar_joint}),
        vec_pris_joint(std::vector<Joint>{prismatic_joint, prismatic_joint,
                                          prismatic_joint}),
        vec_mixed_joint(
            std::vector<Joint>{revolute_joint, fixed_joint, prismatic_joint}),
        fixed_chain(vec_fix_joint), revolute_chain(vec_rev_joint),
        revo_planar_chain(vec_rev_planar), prismatic_chain(vec_pris_joint),
        mixed_chain(vec_mixed_joint) {}
};

TEST_F(KinematicChainTest, JointCount_ReturnCorrectValues) {
  EXPECT_EQ(default_chain.joint_count(), 0);
  EXPECT_EQ(fixed_chain.joint_count(), 3);
  EXPECT_EQ(revolute_chain.joint_count(), 3);
  EXPECT_EQ(prismatic_chain.joint_count(), 3);
}

TEST_F(KinematicChainTest, DofCount_ReturnCorrectValues) {
  EXPECT_EQ(default_chain.dof_count(), 0);
  EXPECT_EQ(fixed_chain.dof_count(), 0);
  EXPECT_EQ(revolute_chain.dof_count(), 3);
  EXPECT_EQ(prismatic_chain.dof_count(), 3);
}

TEST_F(KinematicChainTest, DefaultInitialization_StoresCorrectValues) {
  EXPECT_EQ(default_chain.joint_count(), 0);
  EXPECT_EQ(default_chain.dof_count(), 0);
}

TEST_F(KinematicChainTest, ParameterizedInitialization_StoresCorrectValues) {
  EXPECT_EQ(revolute_chain.joint_count(), 3);
  EXPECT_EQ(revolute_chain.dof_count(), 3);
  EXPECT_EQ(prismatic_chain.joint_count(), 3);
  EXPECT_EQ(prismatic_chain.dof_count(), 3);
}

TEST_F(KinematicChainTest, AddJoint_StoresCorrectValues) {
  EXPECT_EQ(default_chain.joint_count(), 0);
  default_chain.add_joint(revolute_joint);
  EXPECT_EQ(default_chain.joint_count(), 1);

  // stress test
  for (int i = 2; i < 256; i++) {
    default_chain.add_joint(revolute_joint);
    EXPECT_EQ(default_chain.joint_count(), i);
  }
}

TEST_F(KinematicChainTest, ForwardKinematics_EmptyChain_AssertionAborts) {
  EXPECT_DEATH(
      default_chain.forward_kinematics(std::vector<double>{2.0, 4.0, 5.0}),
      "KinematicChain: q count must match DOF");
}

TEST_F(KinematicChainTest, ForwardKinematics_FixedChain_AssertionAborts) {
  EXPECT_DEATH(
      fixed_chain.forward_kinematics(std::vector<double>{5.0, 7.0, 1.0}),
      "KinematicChain: q count must match DOF");
}

TEST_F(KinematicChainTest, ForwardKinematics_Revolute_ComposesCorrectly) {
  Mat3 result_rot_q90 =
      (Quaternion::from_axis_angle(axis_x, -kPi / 2)).to_rotation_matrix();
  Mat3 result_rot_q0 =
      (Quaternion::from_axis_angle(axis_x, 0.0)).to_rotation_matrix();
  EXPECT_EQ(
      revolute_chain.forward_kinematics(std::vector<double>{0.0, 0.0, 0.0}),
      SE3(result_rot_q90, Vec3{0.0, 0.0, 5.0}));
  EXPECT_EQ(revolute_chain.forward_kinematics(
                std::vector<double>{kPi / 2, kPi / 2, kPi / 2}),
            SE3(result_rot_q0, Vec3{-5.0, 5.0, -5.0}));
}

TEST_F(KinematicChainTest, ForwardKinematics_Prismatic_ComposesCorrectly) {
  Mat3 result_rot_q90 =
      (Quaternion::from_axis_angle(axis_x, -kPi / 2)).to_rotation_matrix();
  EXPECT_EQ(
      prismatic_chain.forward_kinematics(std::vector<double>{-5.0, 2.0, -3.0}),
      SE3(result_rot_q90, Vec3{0.0, 2.0, 3.0}));
}

TEST_F(KinematicChainTest, ForwardKinematics_MixedJoints_ValuesCorrectlyApply) {
  Mat3 result_mixed =
      (Quaternion::from_axis_angle(axis_x, -kPi / 2).to_rotation_matrix()) *
      (Quaternion::from_axis_angle(axis_z, -kPi / 2).to_rotation_matrix());
  EXPECT_EQ(mixed_chain.forward_kinematics(std::vector<double>{kPi / 2, 2.0}),
            SE3(result_mixed, Vec3{-5.0, 2.0, 0.0}));
}

TEST_F(KinematicChainTest,
       Jacobian_RevolutePlanar_FiniteDifferenceMatchesAnalytical) {
  std::vector<double> test{1 + kEpsilonSmall, 0.0, 0.0};

  SE3 pose_base =
      revo_planar_chain.forward_kinematics(std::vector<double>{1.0, 0.0, 0.0});
  SE3 pose_test = revo_planar_chain.forward_kinematics(test);

  Vec3 linear_diff =
      (pose_test.translation() - pose_base.translation()) / kEpsilonSmall;
  Mat3 angular_diff =
      ((pose_test.rotation() * pose_base.rotation().transpose()) -
       Mat3::identity()) /
      kEpsilonSmall;

  Jacobian j = revo_planar_chain.jacobian(test);

  EXPECT_NEAR((j[0, 0]), linear_diff[0], 1e-4);
  EXPECT_NEAR((j[1, 0]), linear_diff[1], 1e-4);
  EXPECT_NEAR((j[2, 0]), linear_diff[2], 1e-4);
  EXPECT_NEAR((j[3, 0]), (angular_diff[2, 1]), 1e-4);
  EXPECT_NEAR((j[4, 0]), (angular_diff[0, 2]), 1e-4);
  EXPECT_NEAR((j[5, 0]), (angular_diff[1, 0]), 1e-4);
}

TEST_F(KinematicChainTest,
       Jacobian_Prismatic_FiniteDifferenceMatchesAnalytical) {
  std::vector<double> test{1 + kEpsilonSmall, 0.0, 0.0};
  SE3 pose_base =
      prismatic_chain.forward_kinematics(std::vector<double>{1.0, 0.0, 0.0});
  SE3 pose_test = prismatic_chain.forward_kinematics(test);
  Vec3 linear_diff =
      (pose_test.translation() - pose_base.translation()) / kEpsilonSmall;
  Mat3 angular_diff =
      ((pose_test.rotation() * pose_base.rotation().transpose()) -
       Mat3::identity()) /
      kEpsilonSmall;

  Jacobian j = prismatic_chain.jacobian(test);

  EXPECT_NEAR((j[0, 0]), linear_diff[0], 1e-4);
  EXPECT_NEAR((j[1, 0]), linear_diff[1], 1e-4);
  EXPECT_NEAR((j[2, 0]), linear_diff[2], 1e-4);
  EXPECT_NEAR((j[3, 0]), (angular_diff[2, 1]), 1e-4);
  EXPECT_NEAR((j[4, 0]), (angular_diff[0, 2]), 1e-4);
  EXPECT_NEAR((j[5, 0]), (angular_diff[1, 0]), 1e-4);
}

TEST_F(KinematicChainTest,
       Jacobian_AllColumns_FiniteDifferenceMatchesAnalytical) {
  std::vector<double> base{1.0, 1.0, 1.0};
  for (long i = 0; i < 3; i++) {
    std::vector<double> test = base;
    test[static_cast<size_t>(i)] += kEpsilonSmall;
    SE3 pose_base = revo_planar_chain.forward_kinematics(base);
    SE3 pose_test = revo_planar_chain.forward_kinematics(test);

    Vec3 linear_diff =
        (pose_test.translation() - pose_base.translation()) / kEpsilonSmall;
    Mat3 angular_diff =
        ((pose_test.rotation() * pose_base.rotation().transpose()) -
         Mat3::identity()) /
        kEpsilonSmall;

    Jacobian j = revo_planar_chain.jacobian(test);
    EXPECT_NEAR((j[0, i]), linear_diff[0], 1e-4);
    EXPECT_NEAR((j[1, i]), linear_diff[1], 1e-4);
    EXPECT_NEAR((j[2, i]), linear_diff[2], 1e-4);
    EXPECT_NEAR((j[3, i]), (angular_diff[2, 1]), 1e-4);
    EXPECT_NEAR((j[4, i]), (angular_diff[0, 2]), 1e-4);
    EXPECT_NEAR((j[5, i]), (angular_diff[1, 0]), 1e-4);
  }
}

TEST_F(KinematicChainTest,
       Jacobian_MixedChain_FiniteDifferenceMatchesAnalytical) {
  std::vector<double> base{1.0, 1.0};
  for (long i = 0; i < 2; i++) {
    std::vector<double> test = base;
    test[static_cast<size_t>(i)] += kEpsilonSmall;
    SE3 pose_base = mixed_chain.forward_kinematics(base);
    SE3 pose_test = mixed_chain.forward_kinematics(test);

    Vec3 linear_diff =
        (pose_test.translation() - pose_base.translation()) / kEpsilonSmall;
    Mat3 angular_diff =
        ((pose_test.rotation() * pose_base.rotation().transpose()) -
         Mat3::identity()) /
        kEpsilonSmall;

    Jacobian j = mixed_chain.jacobian(test);
    EXPECT_NEAR((j[0, i]), linear_diff[0], 1e-4);
    EXPECT_NEAR((j[1, i]), linear_diff[1], 1e-4);
    EXPECT_NEAR((j[2, i]), linear_diff[2], 1e-4);
    EXPECT_NEAR((j[3, i]), (angular_diff[2, 1]), 1e-4);
    EXPECT_NEAR((j[4, i]), (angular_diff[0, 2]), 1e-4);
    EXPECT_NEAR((j[5, i]), (angular_diff[1, 0]), 1e-4);
  }
}
