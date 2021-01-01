#ifndef _ASTRA_ROS_POINT_CLOUD_HPP_
#define _ASTRA_ROS_POINT_CLOUD_HPP_

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud.h>

namespace astra_ros
{
  bool defaultMask(const std::size_t x, const std::size_t y);
  sensor_msgs::PointCloud toPointCloud(const sensor_msgs::Image &rgb, const sensor_msgs::Image &registered_depth, const sensor_msgs::CameraInfo &camera_info, const std::function<bool (const std::size_t x, const std::size_t y)> &mask = defaultMask);
}

#endif