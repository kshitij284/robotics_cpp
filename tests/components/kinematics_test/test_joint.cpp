#include "kinematics/joint.hpp"
#include <gtest/gtest.h>

using namespace kinematics::math;

class JointTest : public ::testing::Test {
protected:
  Joint joint_default;
  Mat3 rot_identity;
  Mat3 rot90x;
  Vec3 vec_zero;
  Vec3 translate10x;
  SE3 rest_transform_non;
  SE3 rest_transform_rot_trans;
  Joint test_joint_rev;

  JointTest()
      : rot_identity(Mat3::identity()), rot90x(Mat3::m90x()),
        vec_zero{
            0.0,
            0.0,
            0.0,
        },
        translate10x{10.0, 0.0, 0.0},
        rest_transform_non(rot_identity, vec_zero),
        rest_transform_rot_trans(rot90x, translate10x),
        test_joint_rev("rev_joint", Revolute{}, rest_transform_rot_trans, -1.0,
                       +1.0) {}
};

TEST_F(JointTest, DefaultInitialization) {
  EXPECT_EQ(joint_default.name(), "joint");
  ASSERT_TRUE(std::holds_alternative<Fixed>(joint_default.type()));
  EXPECT_EQ(joint_default.rest_transform(), SE3::identity());
  EXPECT_DOUBLE_EQ(joint_default.min_q(), -kPi);
  EXPECT_DOUBLE_EQ(joint_default.max_q(), kPi);
}

TEST_F(JointTest, ParameterizedInitialization) {
  EXPECT_EQ(test_joint_rev.name(), "rev_joint");
  ASSERT_TRUE(std::holds_alternative<Revolute>(test_joint_rev.type()));
  EXPECT_EQ(test_joint_rev.rest_transform(), SE3(rot90x, translate10x));
  EXPECT_DOUBLE_EQ(test_joint_rev.min_q(), -1.0);
  EXPECT_DOUBLE_EQ(test_joint_rev.max_q(), 1.0);
}

TEST_F(JointTest, Transform_FixedJoint_ReturnsSelf) {
  EXPECT_EQ(joint_default.transform(0.5), joint_default.rest_transform());
}
