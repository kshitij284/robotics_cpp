#include "kinematics/chain.hpp"
#include <gtest/gtest.h>

using namespace kinematics::math;

class KinematicChainTest : public ::testing::Test {
protected:
  KinematicChain default_chain;
  Vec3 axis_x;
  Vec3 axis_y;
  Vec3 axis_z;
  SE3 rest_transform_rot90x_tran5y;
  Joint fixed_joint;
  Joint revolute_joint;
  Joint prismatic_joint;
  std::vector<Joint> vec_fix_joint;
  std::vector<Joint> vec_rev_joint;
  std::vector<Joint> vec_pris_joint;
  std::vector<Joint> vec_mixed_joint;
  KinematicChain fixed_chain;
  KinematicChain revolute_chain;
  KinematicChain prismatic_chain;
  KinematicChain mixed_chain;

  KinematicChainTest()
      : axis_x(Vec3{1.0, 0.0, 0.0}), axis_y(Vec3{0.0, 1.0, 0.0}),
        axis_z(Vec3{0.0, 0.0, 1.0}),
        rest_transform_rot90x_tran5y(SE3(Mat3::m90x(), Vec3{0.0, 5.0, 0.0})),
        fixed_joint("joint_fixed", Fixed{}, rest_transform_rot90x_tran5y, 0.0,
                    0.0),
        revolute_joint("joint_rev", Revolute{axis_z},
                       rest_transform_rot90x_tran5y, -kPi, kPi),
        prismatic_joint("joint_prismatic", Prismatic{axis_z},
                        rest_transform_rot90x_tran5y, -10.0, 10.0),
        vec_fix_joint(
            std::vector<Joint>{fixed_joint, fixed_joint, fixed_joint}),
        vec_rev_joint(
            std::vector<Joint>{revolute_joint, revolute_joint, revolute_joint}),
        vec_pris_joint(std::vector<Joint>{prismatic_joint, prismatic_joint,
                                          prismatic_joint}),
        vec_mixed_joint(
            std::vector<Joint>{revolute_joint, fixed_joint, prismatic_joint}),
        fixed_chain(vec_fix_joint), revolute_chain(vec_rev_joint),
        prismatic_chain(vec_pris_joint), mixed_chain(vec_mixed_joint) {}
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

TEST_F(KinematicChainTest, ParameteriedInitialization_StoresCorrectValues) {
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

TEST_F(KinematicChainTest, ForwardKinematics_EmptyChain_AssertionThrown) {
  EXPECT_DEATH(
      default_chain.forward_kinematics(std::vector<double>{2.0, 4.0, 5.0}),
      "KinematicChain: q count must match DOF");
}

TEST_F(KinematicChainTest, ForwardKinematics_FixedChain_AssertionThrown) {
  EXPECT_DEATH(
      fixed_chain.forward_kinematics(std::vector<double>{5.0, 7.0, 1.0}),
      "KinematicChain: q count must match DOF");
}

TEST_F(KinematicChainTest, ForwardKinematics_Revolut_ComposesCorrectly) {
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
