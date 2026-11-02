#include <sstream>
#include <tinyxml2.h>
#include <urdf/urdf_parser.hpp>

namespace robotics::urdf {

std::optional<std::vector<ParsedJoint>>
parse_urdf_joints(const std::string &urdf_file_path) {
  tinyxml2::XMLDocument doc;

  if (doc.LoadFile(urdf_file_path.c_str()) != tinyxml2::XML_SUCCESS)
    return std::nullopt;

  const tinyxml2::XMLElement *robot = doc.RootElement();
  if (!robot)
    return std::nullopt;

  std::vector<ParsedJoint> joint_vector;
  for (auto *j = robot->FirstChildElement("joint"); j;
       j = j->NextSiblingElement("joint")) {
    if (!j)
      return std::nullopt;

    const char *name = j->Attribute("name");
    const char *type = j->Attribute("type");
    if (!name || !type)
      return std::nullopt;

    ParsedJoint joint;
    joint.name = name;
    joint.type = type;

    if (auto *origin = j->FirstChildElement("origin")) {
      if (auto xyz_attr = origin->Attribute("xyz")) {
        if (std::istringstream xyz{xyz_attr}) {
          double x, y, z;
          if (xyz >> x >> y >> z) {
            joint.origin_x = x;
            joint.origin_y = y;
            joint.origin_z = z;
          } else
            return std::nullopt;
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
            return std::nullopt;
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
            return std::nullopt;
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
            return std::nullopt;
        }
      }
      if (auto upper_attr = limit->Attribute("upper")) {
        if (std::istringstream upper{upper_attr}) {
          double u;
          if (upper >> u) {
            joint.limit_upper = u;
          } else
            return std::nullopt;
        }
      }
    }

    joint_vector.push_back(joint);
  }
  return joint_vector;
}
} // namespace robotics::urdf
