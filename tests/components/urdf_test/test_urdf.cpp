#include "kinematics/chain.hpp"
#include "urdf/urdf_parser.hpp"
#include <gtest/gtest.h>
#include <string>

using namespace robotics::urdf;

class UrdfParserTest : public ::testing::Test {
protected:
  ParsedJoint default_joint;
  std::vector<double> axis_x{1.0, 0.0, 0.0};
  std::vector<double> axis_y{0.0, 1.0, 0.0};

  UrdfParserTest() {}
};

TEST_F(UrdfParserTest, DefaultParsedJoint_ComposesCorrectly) {
  EXPECT_EQ(default_joint.name, "default");
  EXPECT_EQ(default_joint.type, "fixed");
  EXPECT_DOUBLE_EQ(default_joint.origin_x, 0.0);
  EXPECT_DOUBLE_EQ(default_joint.origin_y, 0.0);
  EXPECT_DOUBLE_EQ(default_joint.origin_z, 0.0);
  EXPECT_DOUBLE_EQ(default_joint.origin_roll, 0.0);
  EXPECT_DOUBLE_EQ(default_joint.origin_pitch, 0.0);
  EXPECT_DOUBLE_EQ(default_joint.origin_yaw, 0.0);
  EXPECT_EQ(default_joint.axis, axis_x);
  EXPECT_DOUBLE_EQ(default_joint.limit_lower, 0.0);
  EXPECT_DOUBLE_EQ(default_joint.limit_upper, 0.0);
}

TEST_F(UrdfParserTest, ImportFromURDF_SingleJoint_CorrectInitialization) {
  std::string urdf_path =
      std::string(TEST_RESOURCES_DIR) + "test_one_joint.urdf";
  auto parsed_joints = parse_urdf_joints(urdf_path);
  EXPECT_TRUE(parsed_joints.has_value());
  if (parsed_joints) {
    auto chain = *parsed_joints;
    EXPECT_EQ(chain.size(), 1);
    ParsedJoint test = chain[0];
    EXPECT_EQ(test.name, "base_joint");
    EXPECT_EQ(test.type, "continuous");
    EXPECT_DOUBLE_EQ(test.origin_x, 0.0);
    EXPECT_DOUBLE_EQ(test.origin_y, 0.0);
    EXPECT_DOUBLE_EQ(test.origin_z, 0.5);
    EXPECT_EQ(test.axis, axis_y);
    EXPECT_DOUBLE_EQ(test.limit_lower, -1.57);
    EXPECT_DOUBLE_EQ(test.limit_upper, 1.57);
  }
}

TEST_F(UrdfParserTest, ImportFromURDF_MultipleJoint_CorrectInitialization) {
  ParsedJoint j1{.name = "link_1_joint",
                 .type = "revolute",
                 .origin_x = 0.0,
                 .origin_y = 0.0,
                 .origin_z = 0.05,
                 .axis = {0.0, 0.0, 1.0},
                 .limit_lower = -1.57,
                 .limit_upper = 1.57};
  ParsedJoint j2{.name = "link_2_joint",
                 .type = "revolute",
                 .origin_x = 0.0,
                 .origin_y = 0.0,
                 .origin_z = 0.05,
                 .axis = {0.0, 1.0, 0.0},
                 .limit_lower = -1.57,
                 .limit_upper = 1.57};
  ParsedJoint j3{.name = "link_3_joint",
                 .type = "revolute",
                 .origin_x = 0.0,
                 .origin_y = 0.0,
                 .origin_z = 0.5,
                 .axis = {0.0, 1.0, 0.0},
                 .limit_lower = -1.57,
                 .limit_upper = 1.57};

  std::vector<ParsedJoint> manually_initialized_joints{j1, j2, j3};

  std::string urdf_path =
      std::string(TEST_RESOURCES_DIR) + "test_three_joints.urdf";

  auto parsed_initialized_opt = parse_urdf_joints(urdf_path);
  EXPECT_TRUE(parsed_initialized_opt.has_value());
  if (parsed_initialized_opt) {
    auto parsed_initialized_joints = *parsed_initialized_opt;
    EXPECT_EQ(parsed_initialized_joints.size(), 3);
    EXPECT_EQ(manually_initialized_joints, parsed_initialized_joints);
  }
}
