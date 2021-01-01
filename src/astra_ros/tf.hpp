#ifndef _ASTRA_ROS_TF_HPP_
#define _ASTRA_ROS_TF_HPP_

#include <unordered_map>
#include <geometry_msgs/TransformStamped.h>
#include <astra_ros/Body.h>

namespace astra_ros
{
  std::vector<geometry_msgs::TransformStamped> bodyTransforms(const Body &body);
}

#endif