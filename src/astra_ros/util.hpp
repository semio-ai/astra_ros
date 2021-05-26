#ifndef _ASTRA_ROS_UTIL_HPP_
#define _ASTRA_ROS_UTIL_HPP_

#include <vector>

#include <sensor_msgs/Image.h>
#include <visualization_msgs/MarkerArray.h>
#include "astra_ros/Device.hpp"
#include "astra_ros/ImageStreamMode.h"
#include "astra_ros/Body.h"
#include "astra_ros/Plane.h"
#include <astra/capi/astra.h>

namespace astra_ros
{
  const std::string &statusToString(const astra_status_t status);
  sensor_msgs::Image toRos(const std::uint8_t *const data, const std::size_t length, const astra_image_metadata_t &metadata);
  sensor_msgs::Image toRos(const std::int16_t *const data, const std::size_t length, const astra_image_metadata_t &metadata);
  sensor_msgs::Image toRos(const astra_rgba_pixel_t *const data, const std::size_t length, const astra_image_metadata_t &metadata);
  sensor_msgs::Image toRos(const astra_floormask_t &floor_mask);
  Body toRos(const astra_body_t &body, const std_msgs::Header &header);
  std::vector<Body> toRos(const astra_body_list_t &body_list, const std_msgs::Header &header);
  Plane toRos(const astra_plane_t &plane);
  ImageStreamMode toRos(const Device::ImageStreamMode &image_stream_mode);
  Device::ImageStreamMode fromRos(const ImageStreamMode &image_stream_mode);

  template<typename T>
  T clamp(const T min, const T value, const T max)
  {
    if (min > value) return min;
    if (max < value) return max;
    return value;
  }
} 

#endif