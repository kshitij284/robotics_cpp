#pragma once

#include <expected>
#include <string>
#include <vector>

namespace robotics::urdf {

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

} // namespace robotics::urdf
