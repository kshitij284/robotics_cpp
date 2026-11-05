#pragma once

#include "kinematics/chain.hpp"
#include <expected>
#include <string>
#include <vector>

namespace robotics::urdf {

using kinematics::math::Fixed;
using kinematics::math::Joint;
using kinematics::math::JointType;
using kinematics::math::KinematicChain;
using kinematics::math::Prismatic;
using kinematics::math::Revolute;
using kinematics::math::SE3;
using kinematics::math::Vec3;

enum class UrdfError {
  FileNotFound,
  InvalidXml,
  MissingAttribute,
  BranchingDetected,
  MergeDetected,
  CycleDetected,
  DisconnectedGraph
};

struct ParsedJoint {
  std::string name = "default";
  std::string type = "fixed";
  std::string parent = "";
  std::string child = "";
  double origin_x = 0.0, origin_y = 0.0, origin_z = 0.0;
  double origin_roll = 0.0, origin_pitch = 0.0, origin_yaw = 0.0;
  std::vector<double> axis{1.0, 0.0, 0.0};
  double limit_lower = 0.0;
  double limit_upper = 0.0;

  bool operator==(const ParsedJoint &) const = default;
};

[[nodiscard]] std::expected<std::vector<ParsedJoint>, UrdfError>
parse_urdf_joints(const std::string &urdf_file_path);

[[nodiscard]] std::expected<KinematicChain, UrdfError>
build_chain(const std::vector<ParsedJoint> &parsed_joints);

} // namespace robotics::urdf
