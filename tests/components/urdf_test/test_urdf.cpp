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
  EXPECT_EQ(default_joint.parent, "");
  EXPECT_EQ(default_joint.child, "");
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

TEST_F(UrdfParserTest, ParsedFromURDF_SingleJoint_CorrectInitialization) {
  std::string urdf_path =
      std::string(TEST_RESOURCES_DIR) + "test_one_joint.urdf";
  auto parsed_joints = parse_urdf_joints(urdf_path);
  ASSERT_TRUE(parsed_joints.has_value());
  if (parsed_joints) {
    const auto &chain = *parsed_joints;
    EXPECT_EQ(chain.size(), 1);
    ParsedJoint test = chain[0];
    EXPECT_EQ(test.name, "base_joint");
    EXPECT_EQ(test.type, "continuous");
    EXPECT_EQ(test.parent, "base_link");
    EXPECT_EQ(test.child, "link_1");
    EXPECT_DOUBLE_EQ(test.origin_x, 0.0);
    EXPECT_DOUBLE_EQ(test.origin_y, 0.0);
    EXPECT_DOUBLE_EQ(test.origin_z, 0.5);
    EXPECT_EQ(test.axis, axis_y);
    EXPECT_DOUBLE_EQ(test.limit_lower, -1.57);
    EXPECT_DOUBLE_EQ(test.limit_upper, 1.57);
  }
}

TEST_F(UrdfParserTest, ParsedFromURDF_MultipleJoint_CorrectInitialization) {
  ParsedJoint j1{.name = "link_1_joint",
                 .type = "revolute",
                 .parent = "base_link",
                 .child = "link_1",
                 .origin_x = 0.0,
                 .origin_y = 0.0,
                 .origin_z = 0.05,
                 .axis = {0.0, 0.0, 1.0},
                 .limit_lower = -1.57,
                 .limit_upper = 1.57};
  ParsedJoint j2{.name = "link_2_joint",
                 .type = "revolute",
                 .parent = "link_1",
                 .child = "link_2",
                 .origin_x = 0.0,
                 .origin_y = 0.0,
                 .origin_z = 0.05,
                 .axis = {0.0, 1.0, 0.0},
                 .limit_lower = -1.57,
                 .limit_upper = 1.57};
  ParsedJoint j3{.name = "link_3_joint",
                 .type = "revolute",
                 .parent = "link_2",
                 .child = "link_3",
                 .origin_x = 0.0,
                 .origin_y = 0.0,
                 .origin_z = 0.5,
                 .axis = {0.0, 1.0, 0.0},
                 .limit_lower = -1.57,
                 .limit_upper = 1.57};

  std::vector<ParsedJoint> manually_initialized_joints{j1, j2, j3};

  std::string urdf_path =
      std::string(TEST_RESOURCES_DIR) + "test_three_joints.urdf";

  auto parsed_initialized_exp = parse_urdf_joints(urdf_path);
  ASSERT_TRUE(parsed_initialized_exp.has_value());
  if (parsed_initialized_exp) {
    const auto &parsed_initialized_joints = *parsed_initialized_exp;
    EXPECT_EQ(parsed_initialized_joints.size(), 3);
    EXPECT_EQ(manually_initialized_joints, parsed_initialized_joints);
  }
}

TEST_F(UrdfParserTest, UrdfImport_Suffled_SequencedCorrectly) {
  std::string urdf_path_baseline =
      std::string(TEST_RESOURCES_DIR) + "test_three_joints.urdf";
  std::string urdf_path_shuffled =
      std::string(TEST_RESOURCES_DIR) + "test_three_joints_shuffled.urdf";
  auto parsed_baseline_exp = parse_urdf_joints(urdf_path_baseline);
  auto parsed_shuffled_exp = parse_urdf_joints(urdf_path_shuffled);

  ASSERT_TRUE(parsed_baseline_exp.has_value());
  ASSERT_TRUE(parsed_shuffled_exp.has_value());
  const auto &baseline_joints = *parsed_baseline_exp;
  const auto &shuffled_joints = *parsed_shuffled_exp;

  EXPECT_EQ(baseline_joints, shuffled_joints);
}

TEST_F(UrdfParserTest, UrdfImport_Branched_ReturnBranchingDetectedError) {
  std::string urdf_path_branched =
      std::string(TEST_RESOURCES_DIR) + "test_branching.urdf";
  auto parsed_branched_exp = parse_urdf_joints(urdf_path_branched);
  ASSERT_FALSE(parsed_branched_exp.has_value());
  EXPECT_EQ(parsed_branched_exp.error(), UrdfError::BranchingDetected);
}

TEST_F(UrdfParserTest, UrdfImport_Forest_ReturnDisconnectedGraphError) {
  std::string urdf_path_forest =
      std::string(TEST_RESOURCES_DIR) + "test_forest.urdf";
  auto parsed_forest_exp = parse_urdf_joints(urdf_path_forest);
  ASSERT_FALSE(parsed_forest_exp.has_value());
  EXPECT_EQ(parsed_forest_exp.error(), UrdfError::DisconnectedGraph);
}

TEST_F(UrdfParserTest,
       UrdfImport_MissingParentorChild_ReturnMissingAttributeError) {
  std::string urdf_path_missing_parent =
      std::string(TEST_RESOURCES_DIR) + "test_missing_parent.urdf";
  auto parsed_missing_parent_exp = parse_urdf_joints(urdf_path_missing_parent);
  ASSERT_FALSE(parsed_missing_parent_exp.has_value());
  EXPECT_EQ(parsed_missing_parent_exp.error(), UrdfError::MissingAttribute);

  std::string urdf_path_missing_child =
      std::string(TEST_RESOURCES_DIR) + "test_missing_child.urdf";
  auto parsed_missing_child_exp = parse_urdf_joints(urdf_path_missing_child);
  ASSERT_FALSE(parsed_missing_child_exp.has_value());
  EXPECT_EQ(parsed_missing_child_exp.error(), UrdfError::MissingAttribute);
}

TEST_F(UrdfParserTest, UrdfImport_Cycle_ReturnCycleDetectedError) {
  std::string urdf_path_cycle =
      std::string(TEST_RESOURCES_DIR) + "test_cycle.urdf";
  auto parsed_cycle_exp = parse_urdf_joints(urdf_path_cycle);
  ASSERT_FALSE(parsed_cycle_exp.has_value());
  EXPECT_EQ(parsed_cycle_exp.error(), UrdfError::CycleDetected);
}

TEST_F(UrdfParserTest, UrdfImport_InvalidPath_ReturnFileNotFoundError) {
  std::string urdf_path_no_file =
      std::string(TEST_RESOURCES_DIR) + "no_file.urdf";
  auto parsed_no_file_exp = parse_urdf_joints(urdf_path_no_file);
  ASSERT_FALSE(parsed_no_file_exp.has_value());
  EXPECT_EQ(parsed_no_file_exp.error(), UrdfError::FileNotFound);
}

TEST_F(UrdfParserTest, UrdfImport_InvalidXml_ReturnInvalidXmlError) {
  std::string urdf_path_invalid =
      std::string(TEST_RESOURCES_DIR) + "invalid.urdf";
  auto parsed_invalid_exp = parse_urdf_joints(urdf_path_invalid);
  ASSERT_FALSE(parsed_invalid_exp.has_value());
  EXPECT_EQ(parsed_invalid_exp.error(), UrdfError::InvalidXml);
}
