#include "visualization.hpp"

#include "color.hpp"
#include "math.hpp"


#include <sstream>
#include <unordered_map>

using namespace astra_ros;

namespace
{
  const static std::vector<std::pair<std::uint8_t, std::uint8_t>> JOINT_PAIRS {
    { Joint::TYPE_SHOULDER_SPINE, Joint::TYPE_HEAD },
    { Joint::TYPE_SHOULDER_SPINE, Joint::TYPE_RIGHT_SHOULDER },
    { Joint::TYPE_SHOULDER_SPINE, Joint::TYPE_LEFT_SHOULDER },
    { Joint::TYPE_LEFT_SHOULDER, Joint::TYPE_LEFT_ELBOW },
    { Joint::TYPE_LEFT_ELBOW, Joint::TYPE_LEFT_WRIST },
    { Joint::TYPE_LEFT_WRIST, Joint::TYPE_LEFT_HAND },
    { Joint::TYPE_RIGHT_SHOULDER, Joint::TYPE_RIGHT_ELBOW },
    { Joint::TYPE_RIGHT_ELBOW, Joint::TYPE_RIGHT_WRIST },
    { Joint::TYPE_RIGHT_WRIST, Joint::TYPE_RIGHT_HAND },
    { Joint::TYPE_MID_SPINE, Joint::TYPE_SHOULDER_SPINE },
    { Joint::TYPE_BASE_SPINE, Joint::TYPE_MID_SPINE },
    { Joint::TYPE_LEFT_HIP, Joint::TYPE_BASE_SPINE },
    { Joint::TYPE_LEFT_KNEE, Joint::TYPE_LEFT_HIP },
    { Joint::TYPE_LEFT_FOOT, Joint::TYPE_LEFT_KNEE },
    { Joint::TYPE_RIGHT_HIP, Joint::TYPE_BASE_SPINE },
    { Joint::TYPE_RIGHT_KNEE, Joint::TYPE_RIGHT_HIP },
    { Joint::TYPE_RIGHT_FOOT, Joint::TYPE_RIGHT_KNEE },
  };

  const static std::unordered_map<std::uint8_t, float> STATUS_OPACITIES {
    { Joint::STATUS_NOT_TRACKED, 0.2 },
    { Joint::STATUS_LOW_CONFIDENCE, 0.7 },
    { Joint::STATUS_TRACKED, 1.0 },
  };
}

namespace
{
  const static ros::Duration MARKER_LIFETIME(0.5);

  
}

visualization_msgs::MarkerArray astra_ros::toMarkerArray(const astra_ros::Body &body)
{
  using namespace visualization_msgs;

  std::size_t id_iter = 0;

  MarkerArray ret;

  std::ostringstream o;
  o << "astra_ros/" << static_cast<std::size_t>(body.id);
  const std::string ns = o.str();

  // Helper function that creates a marker
  const auto create_marker = [&](const std::int32_t type) -> Marker {
    Marker ret;
    ret.header = body.header;
    ret.ns = ns;
    ret.id = id_iter++;
    ret.type = type;
    ret.action = Marker::MODIFY;
    ret.frame_locked = false;
    ret.lifetime = MARKER_LIFETIME;
    return ret;
  };
  
  std::unordered_map<std::uint8_t, const Joint *> lookup_map;

  // Helper function to look up a joint with caching (to avoid O(n^2)),
  // though with hashmap allocation the performance might not be
  // better for n = number of joints.
  const auto lookup = [&](std::uint8_t type) -> const Joint * {
    const auto it = lookup_map.find(type);
    if (it != lookup_map.cend()) return it->second;

    for (const auto &joint : body.joints)
    {
      if (joint.type != type) continue;

      lookup_map[type] = &joint;
      return &joint;
    }

    return nullptr;
  };

  const Rgb color = indexColor(body.id).toRgb();

  // Center of mass
  Marker center_of_mass = create_marker(Marker::SPHERE);
  center_of_mass.scale.x = center_of_mass.scale.y = center_of_mass.scale.z = 0.1;
  center_of_mass.pose.position.x = body.center_of_mass.x;
  center_of_mass.pose.position.y = body.center_of_mass.y;
  center_of_mass.pose.position.z = body.center_of_mass.z;
  center_of_mass.pose.orientation.w = 1.0;
  center_of_mass.color = color;
  ret.markers.push_back(center_of_mass);

  // Joints
  for (const auto &joint : body.joints)
  {
    Marker marker = create_marker(Marker::CUBE);
    marker.scale.x = marker.scale.y = marker.scale.z = 0.05;
    const auto it = STATUS_OPACITIES.find(joint.status);
    marker.color = color.toRgba(it == STATUS_OPACITIES.cend() ? 0.0 : it->second);
    marker.pose = joint.pose;
    ret.markers.push_back(marker);
  }


  // Draw cylinders between all joint pairs
  for (const auto &joint_pair : JOINT_PAIRS)
  {
    const Joint *const left = lookup(joint_pair.first);
    const Joint *const right = lookup(joint_pair.second);
    
    if (left == nullptr || right == nullptr) continue;

    Marker link = create_marker(Marker::CYLINDER);

    const auto &left_position = left->pose.position;
    const auto &right_position = right->pose.position;
    
    link.scale.z = link.scale.y = 0.01;

    // Scale the cylinder along the X axis to be able to touch both joints
    link.scale.x = distance(left_position, right_position);

    // Place the origin of the cyliner directly in the middle of the two joints
    link.pose.position = (left_position + right_position) / 2.0;
    
    link.pose.orientation = fromEigen(lookAt(
      toEigen(link.pose.position),
      toEigen(left_position),
      Eigen::Vector3d::UnitX(),
      Eigen::Vector3d::UnitZ()
    ));

    const auto left_it = STATUS_OPACITIES.find(left->status);
    const auto right_it = STATUS_OPACITIES.find(right->status);
    link.color = color.toRgba(std::min(
      left_it == STATUS_OPACITIES.cend() ? 0.0f : left_it->second,
      right_it == STATUS_OPACITIES.cend() ? 0.0f : right_it->second
    ));
    ret.markers.push_back(link);
  }

  return ret;
}

visualization_msgs::MarkerArray astra_ros::toMarkerArray(const std::vector<astra_ros::Body> &bodies)
{
  visualization_msgs::MarkerArray ret;

  for (const auto &body : bodies)
  {
    const auto body_markers = toMarkerArray(body);
    ret.markers.insert(ret.markers.end(), body_markers.markers.cbegin(), body_markers.markers.cend());
  }

  return ret;
}