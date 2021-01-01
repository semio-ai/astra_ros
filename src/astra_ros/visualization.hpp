#ifndef _ASTRA_ROS_VISUALIZATION_HPP_
#define _ASTRA_ROS_VISUALIZATION_HPP_

#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>
#include "astra_ros/Body.h"

namespace astra_ros
{
  
  // visualization_msgs::MarkerArray

  visualization_msgs::MarkerArray toMarkerArray(const Body &body);
  visualization_msgs::MarkerArray toMarkerArray(const std::vector<Body> &bodies);
}

#endif