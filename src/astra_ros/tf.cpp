// This file implements Astra Body -> ROS tf2 conversion

#include "tf.hpp"
#include "math.hpp"

#include <eigen3/Eigen/Dense>

#include <unordered_map>
#include <unordered_set>
#include <deque>

#include <tf2/buffer_core.h>

using namespace astra_ros;

namespace
{
  // Derived from the graphic on https://shop.orbbec3d.com/Orbbec-Body-Tracking-License_p_55.html
  // A mapping of frames (parent -> children)
  const static std::unordered_map<std::uint8_t, std::unordered_set<std::uint8_t>> TF_LINKS {
    // Torso (BASE_SPINE is the root)
    {
      Joint::TYPE_BASE_SPINE, {
        Joint::TYPE_LEFT_HIP,  
        Joint::TYPE_RIGHT_HIP,  
        Joint::TYPE_MID_SPINE
      }
    },
    { Joint::TYPE_MID_SPINE, { Joint::TYPE_SHOULDER_SPINE } },
    {
      Joint::TYPE_SHOULDER_SPINE, {
        Joint::TYPE_HEAD,
        Joint::TYPE_RIGHT_SHOULDER,
        Joint::TYPE_LEFT_SHOULDER
      }
    },
    // Left Arm
    { Joint::TYPE_LEFT_SHOULDER, { Joint::TYPE_LEFT_ELBOW } },
    { Joint::TYPE_LEFT_ELBOW, { Joint::TYPE_LEFT_WRIST } },
    { Joint::TYPE_LEFT_WRIST, { Joint::TYPE_LEFT_HAND } },
    // Right Arm
    { Joint::TYPE_RIGHT_SHOULDER, { Joint::TYPE_RIGHT_ELBOW } },
    { Joint::TYPE_RIGHT_ELBOW, { Joint::TYPE_RIGHT_WRIST } },
    { Joint::TYPE_RIGHT_WRIST, { Joint::TYPE_RIGHT_HAND } },
    // Left Leg
    { Joint::TYPE_LEFT_HIP, { Joint::TYPE_LEFT_FOOT } },
    { Joint::TYPE_LEFT_KNEE, { Joint::TYPE_LEFT_FOOT } },
    // Right Leg
    { Joint::TYPE_RIGHT_HIP, { Joint::TYPE_RIGHT_KNEE } },
    { Joint::TYPE_RIGHT_KNEE, { Joint::TYPE_RIGHT_FOOT } }
  };

  const static std::unordered_map<std::uint8_t, std::string> TF_FRAME_NAMES {
    // Head
    { Joint::TYPE_NECK, "neck" },
    { Joint::TYPE_HEAD, "head" },
    // Torso
    { Joint::TYPE_BASE_SPINE, "base_spine" },
    { Joint::TYPE_MID_SPINE, "mid_spine" },
    { Joint::TYPE_SHOULDER_SPINE, "shoulder_spine" },
    // Left Arm
    { Joint::TYPE_LEFT_SHOULDER, "left_shoulder" },
    { Joint::TYPE_LEFT_ELBOW, "left_elbow" },
    { Joint::TYPE_LEFT_WRIST, "left_wrist" },
    { Joint::TYPE_LEFT_HAND, "left_hand" },
    // Right Arm
    { Joint::TYPE_RIGHT_SHOULDER, "right_shoulder" },
    { Joint::TYPE_RIGHT_ELBOW, "right_elbow" },
    { Joint::TYPE_RIGHT_WRIST, "right_wrist" },
    { Joint::TYPE_RIGHT_HAND, "right_hand" },
    // Left Leg
    { Joint::TYPE_LEFT_HIP, "left_hip" },
    { Joint::TYPE_LEFT_KNEE, "left_knee" },
    { Joint::TYPE_LEFT_FOOT, "left_foot" },
    // Right Leg
    { Joint::TYPE_RIGHT_HIP, "right_hip" },
    { Joint::TYPE_RIGHT_KNEE, "right_knee" },
    { Joint::TYPE_RIGHT_FOOT, "right_foot" }
  };

  // Used for some record-keeping during skeleton
  // construction
  struct Node
  {
    std::uint8_t type;
    std::uint8_t parent_type;
  };
}

std::vector<geometry_msgs::TransformStamped> astra_ros::bodyTransforms(const Body &body)
{
  const auto frame_id = [&](const std::uint8_t type) -> std::string {
    const auto it = TF_FRAME_NAMES.find(type);
    if (it == TF_FRAME_NAMES.cend())
    {
      std::ostringstream o;
      o << "Unknown frame name for joint type " << static_cast<std::size_t>(type);
      throw std::runtime_error(o.str());
    }

    std::ostringstream o;
    o << "humanoid/" << static_cast<std::size_t>(body.id) << "/" << it->second;
    return o.str();
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

  tf2::BufferCore buffer;

  for (const auto &joint : body.joints)
  {
    geometry_msgs::TransformStamped transform;
    transform.header = body.header;
    transform.child_frame_id = frame_id(joint.type);

    auto &translation = transform.transform.translation;
    translation.x = joint.pose.position.x;
    translation.y = joint.pose.position.y;
    translation.z = joint.pose.position.z;

    transform.transform.rotation = joint.pose.orientation;
    buffer.setTransform(transform, std::string());
  }


  std::vector<geometry_msgs::TransformStamped> ret;

  Node root;
  root.type = Joint::TYPE_BASE_SPINE;
  root.parent_type = Joint::TYPE_UNKNOWN;

  std::deque<Node> queue { root };

  while (!queue.empty())
  {
    // Get current
    const Node current = queue.front();
    queue.pop_front();

    ret.push_back(buffer.lookupTransform(
      current.parent_type != Joint::TYPE_UNKNOWN ? frame_id(current.parent_type) : body.header.frame_id,
      frame_id(current.type),
      ros::Time(0)
    ));

    // Push current's children for processing
    const auto links_it = TF_LINKS.find(current.type);
    if (links_it == TF_LINKS.cend()) continue;
  
    for (auto it = links_it->second.cbegin(); it != links_it->second.cend(); ++it)
    {
      Node child;
      child.type = *it;
      child.parent_type = current.type;
      queue.push_back(child);
    }
  }

  return ret;
}
 