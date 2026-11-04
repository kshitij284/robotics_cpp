#include <sstream>
#include <tinyxml2.h>
#include <unordered_map>
#include <unordered_set>
#include <urdf/urdf_parser.hpp>

namespace robotics::urdf {

std::expected<std::vector<ParsedJoint>, UrdfError>
parse_urdf_joints(const std::string &urdf_file_path) {
  tinyxml2::XMLDocument doc;

  if (doc.LoadFile(urdf_file_path.c_str()) != tinyxml2::XML_SUCCESS)
    return std::unexpected(UrdfError::FileNotFound);

  const tinyxml2::XMLElement *robot = doc.RootElement();
  if (!robot)
    return std::unexpected(UrdfError::InvalidXml);

  std::vector<ParsedJoint> joint_vector;
  std::unordered_set<std::string> child_link_set;
  std::unordered_map<std::string, ParsedJoint> parent_map;

  for (auto *j = robot->FirstChildElement("joint"); j;
       j = j->NextSiblingElement("joint")) {
    if (!j)
      return std::unexpected(UrdfError::MissingAttribute);

    const char *name = j->Attribute("name");
    const char *type = j->Attribute("type");
    if (!name || !type)
      return std::unexpected(UrdfError::MissingAttribute);

    ParsedJoint joint;
    joint.name = name;
    joint.type = type;

    if (auto *parent = j->FirstChildElement("parent")) {
      if (auto parent_attr = parent->Attribute("link")) {
        std::string parent_link{parent_attr};
        joint.parent = parent_link;
      } else
        return std::unexpected(UrdfError::MissingAttribute);
    } else
      return std::unexpected(UrdfError::MissingAttribute);

    if (auto *child = j->FirstChildElement("child")) {
      if (auto child_attr = child->Attribute("link")) {
        std::string child_link{child_attr};
        joint.child = child_link;
      } else
        return std::unexpected(UrdfError::MissingAttribute);
    } else
      return std::unexpected(UrdfError::MissingAttribute);

    if (auto *origin = j->FirstChildElement("origin")) {
      if (auto xyz_attr = origin->Attribute("xyz")) {
        if (std::istringstream xyz{xyz_attr}) {
          double x, y, z;
          if (xyz >> x >> y >> z) {
            joint.origin_x = x;
            joint.origin_y = y;
            joint.origin_z = z;
          } else
            return std::unexpected(UrdfError::MissingAttribute);
        }
      }

      if (auto rpy_attr = origin->Attribute("rpy")) {
        if (std::istringstream rpy{rpy_attr}) {
          double roll, pitch, yaw;
          if (rpy >> roll >> pitch >> yaw) {
            joint.origin_roll = roll;
            joint.origin_pitch = pitch;
            joint.origin_yaw = yaw;
          } else
            return std::unexpected(UrdfError::MissingAttribute);
        }
      }
    }

    if (auto *axis = j->FirstChildElement("axis")) {
      if (auto xyz_attr = axis->Attribute("xyz")) {
        if (std::istringstream xyz{xyz_attr}) {
          double x, y, z;
          if (xyz >> x >> y >> z) {
            joint.axis = {x, y, z};
          } else
            return std::unexpected(UrdfError::MissingAttribute);
        }
      }
    }

    if (auto *limit = j->FirstChildElement("limit")) {
      if (auto lower_attr = limit->Attribute("lower")) {
        if (std::istringstream lower{lower_attr}) {
          double l;
          if (lower >> l) {
            joint.limit_lower = l;
          } else
            return std::unexpected(UrdfError::MissingAttribute);
        }
      }
      if (auto upper_attr = limit->Attribute("upper")) {
        if (std::istringstream upper{upper_attr}) {
          double u;
          if (upper >> u) {
            joint.limit_upper = u;
          } else
            return std::unexpected(UrdfError::MissingAttribute);
        }
      }
    }

    // Reject branching
    if (parent_map.count(joint.parent))
      return std::unexpected(UrdfError::BranchingDetected);

    // Reject multi-parent
    if (child_link_set.count(joint.child))
      return std::unexpected(UrdfError::MergeDetected);

    parent_map.insert({joint.parent, joint});
    child_link_set.insert(joint.child);
  }

  if (parent_map.empty())
    return std::unexpected(UrdfError::DisconnectedGraph);

  // findin base link with no parent;
  std::string base_link;
  for (auto &[parent, _] : parent_map) {
    if (child_link_set.find(parent) == child_link_set.end()) {
      if (!base_link.empty())
        return std::unexpected(UrdfError::DisconnectedGraph);
      base_link = parent;
    }
  }
  if (base_link.empty())
    return std::unexpected(UrdfError::CycleDetected); // no parent found

  std::unordered_set<std::string> visited_link;
  std::string current_link = base_link;

  while (parent_map.count(current_link)) {
    if (visited_link.count(current_link))
      return std::unexpected(
          UrdfError::CycleDetected); // already visited, cycle exists;

    visited_link.insert(current_link);
    const auto &joint = parent_map.at(current_link);
    joint_vector.push_back(joint);
    current_link = joint.child;
  }
  // ensure all parsed joints were traversed
  if (joint_vector.size() != parent_map.size())
    return std::unexpected(UrdfError::DisconnectedGraph);

  return joint_vector;
}
} // namespace robotics::urdf
