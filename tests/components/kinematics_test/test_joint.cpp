#include "kinematics/joint.hpp"
#include <gtest/gtest.h>

using namespace kinematics::math;

class JointTest : public ::testing::Test {
protected:
  Joint joint_default;
  Mat3 rot_identity;
  Mat3 rot90x;
  Mat3 rot90z;
  Vec3 vec_zero;
  Vec3 axis_x;
  Vec3 axis_y;
  Vec3 axis_z;
  Vec3 translate10x;
  Vec3 translate5y;
  SE3 rest_transform_non;
  SE3 rest_transform_rot;
  SE3 rest_transform_translation;
  SE3 rest_transform_rot_translation;
  Joint revolute_joint_no_rt;
  Joint revolute_joint;
  Joint prismatic_joint_no_rt;
  Joint prismatic_joint;

  JointTest()
      : rot_identity(Mat3::identity()), rot90x(Mat3::m90x()),
        rot90z(Mat3::m90z()), vec_zero{0.0, 0.0, 0.0}, axis_x{1.0, 0.0, 0.0},
        axis_y{0.0, 1.0, 0.0}, axis_z{0.0, 0.0, 1.0},
        translate10x{10.0, 0.0, 0.0}, translate5y{0.0, 5.0, 0.0},
        rest_transform_non(rot_identity, vec_zero), rest_transform_rot(rot90z),
        rest_transform_translation(translate10x),
        rest_transform_rot_translation(rot90x, translate5y),
        revolute_joint_no_rt("joint_rev", Revolute{axis_z}, rest_transform_non,
                             -kPi, kPi),
        revolute_joint("joint_rev", Revolute{axis_z},
                       rest_transform_rot_translation, -kPi, kPi),
        prismatic_joint_no_rt("prismatic_joint", Prismatic{axis_y},
                              rest_transform_non, -5.0, 5.0),
        prismatic_joint("prismatic_joint", Prismatic{axis_y},
                        rest_transform_rot_translation, -5.0, 5.0) {}
};

TEST_F(JointTest, DefaultInitialization) {
  EXPECT_EQ(joint_default.name(), "joint");
  ASSERT_TRUE(std::holds_alternative<Fixed>(joint_default.type()));
  EXPECT_EQ(joint_default.rest_transform(), SE3::identity());
  EXPECT_DOUBLE_EQ(joint_default.min_q(), 0.0);
  EXPECT_DOUBLE_EQ(joint_default.max_q(), 0.0);
}

TEST_F(JointTest, ParameterizedInitialization) {
  EXPECT_EQ(revolute_joint.name(), "joint_rev");
  ASSERT_TRUE(std::holds_alternative<Revolute>(revolute_joint.type()));
  EXPECT_EQ(revolute_joint.rest_transform(), SE3(rot90x, translate5y));
  EXPECT_DOUBLE_EQ(revolute_joint.min_q(), -kPi);
  EXPECT_DOUBLE_EQ(revolute_joint.max_q(), kPi);
}

TEST_F(JointTest, Transform_Fixed_qAnyValue_ReturnsSelf) {
  EXPECT_EQ(joint_default.transform(0.5), joint_default.rest_transform());
  EXPECT_EQ(joint_default.transform(-1235325.0),
            joint_default.rest_transform());
  EXPECT_EQ(joint_default.transform(2893759285.0),
            joint_default.rest_transform());
}

TEST_F(JointTest, Transform_Revolute_qZero_ReturnsSelf) {
  EXPECT_EQ(revolute_joint.transform(0.0), rest_transform_rot_translation);
}

TEST_F(JointTest, Transform_Revolute_ComposesBeforeMotion) {
  EXPECT_EQ(revolute_joint_no_rt.transform(kPi / 2), SE3(Mat3::m90z()));
}

TEST_F(JointTest, Transform_Revolute_ComposesAfterMotion) {
  EXPECT_EQ(revolute_joint.transform(kPi / 2),
            rest_transform_rot_translation * SE3(rot90z));
}

TEST_F(JointTest, Transform_Revolute_qBoundaries_ComposesCorrectly) {
  EXPECT_EQ(revolute_joint.transform(-kPi),
            rest_transform_rot_translation *
                SE3::from_quaternion_translation(
                    Quaternion::from_axis_angle(axis_z, -kPi), vec_zero));
  EXPECT_EQ(revolute_joint.transform(kPi),
            rest_transform_rot_translation *
                SE3::from_quaternion_translation(
                    Quaternion::from_axis_angle(axis_z, kPi), vec_zero));
}

TEST_F(JointTest, Transform_Revolute_qOutOfLimit_ClampCorrectly) {
  EXPECT_EQ(revolute_joint.transform(-kPi * 2),
            rest_transform_rot_translation *
                SE3::from_quaternion_translation(
                    Quaternion::from_axis_angle(axis_z, -kPi), vec_zero));
  EXPECT_EQ(revolute_joint.transform(kPi * 2),
            rest_transform_rot_translation *
                SE3::from_quaternion_translation(
                    Quaternion::from_axis_angle(axis_z, kPi), vec_zero));
}

TEST_F(JointTest, Transform_Prismatic_qZero_ReturnsSelf) {
  EXPECT_EQ(prismatic_joint.transform(0.0), rest_transform_rot_translation);
}

TEST_F(JointTest, Transform_Prismatic_ComposesBeforeMotions) {
  EXPECT_EQ(prismatic_joint_no_rt.transform(3.0), SE3(Vec3{0.0, 3.0, 0.0}));
}

TEST_F(JointTest, Transform_Prismatic_ComposesAfterMotion) {
  EXPECT_EQ(prismatic_joint.transform(3.0),
            rest_transform_rot_translation * SE3(Vec3{0.0, 3.0, 0.0}));
}

TEST_F(JointTest, Transform_Prismatic_qBoundaries_ComposesCorrectly) {
  EXPECT_EQ(prismatic_joint.transform(-5.0),
            rest_transform_rot_translation * SE3(Vec3{0.0, -5.0, 0.0}));
  EXPECT_EQ(prismatic_joint.transform(5.0),
            rest_transform_rot_translation * SE3(Vec3{0.0, 5.0, 0.0}));
}

TEST_F(JointTest, Transform_Prismatic_qOutOfLimits_ComposesCorrectly) {
  EXPECT_EQ(prismatic_joint.transform(-10.0),
            rest_transform_rot_translation * SE3(Vec3{0.0, -5.0, 0.0}));
  EXPECT_EQ(prismatic_joint.transform(10.0),
            rest_transform_rot_translation * SE3(Vec3{0.0, 5.0, 0.0}));
}
