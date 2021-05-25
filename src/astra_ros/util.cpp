#include "util.hpp"

#include <astra/capi/astra.h>

// #include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <stdexcept>
#include <unordered_map>
#include <sstream>

#include <eigen3/Eigen/Dense>

#include <optional>

#include "visualization.hpp"

namespace std
{
  template<>
  struct hash<astra_status_t>
  {
    std::size_t operator() (astra_status_t status) const
    {
      return static_cast<std::size_t>(status);
    }
  };
}

namespace
{
  __attribute__((destructor))
  static void terminate()
  {
    astra_terminate();
  }


  struct Encoding
  {
    Encoding(const std::string &type, const std::uint32_t byte_length)
      : type(type)
      , byte_length(byte_length)
    {
    }

    std::string type;
    std::uint32_t byte_length;
  };

  const static std::unordered_map<astra_pixel_format_t, Encoding> ENCODING_MAPPING {
    { ASTRA_PIXEL_FORMAT_GRAY8, Encoding(sensor_msgs::image_encodings::MONO8, 1) },
    { ASTRA_PIXEL_FORMAT_GRAY16, Encoding(sensor_msgs::image_encodings::MONO16, 2) },
    { ASTRA_PIXEL_FORMAT_RGB888, Encoding(sensor_msgs::image_encodings::RGB8, 3) },
    { ASTRA_PIXEL_FORMAT_RGBA, Encoding(sensor_msgs::image_encodings::RGBA8, 4) },
  };

  const static std::unordered_map<astra_status_t, std::string> STATUS_STRINGS {
    { ASTRA_STATUS_SUCCESS, "Success" },
    { ASTRA_STATUS_INVALID_PARAMETER, "Invalid Parameter" },
    { ASTRA_STATUS_DEVICE_ERROR, "Device Error" },
    { ASTRA_STATUS_TIMEOUT, "Timeout" },
    { ASTRA_STATUS_INVALID_PARAMETER_TOKEN, "Invalid Parameter Token" },
    { ASTRA_STATUS_INVALID_OPERATION, "Invalid Operation" },
    { ASTRA_STATUS_INTERNAL_ERROR, "Internal Error" },
    { ASTRA_STATUS_UNINITIALIZED, "Uninitialized" }
  };
}

const std::string &astra_ros::statusToString(const astra_status_t status)
{
  const auto it = STATUS_STRINGS.find(status);
  if (it == STATUS_STRINGS.cend()) throw std::runtime_error("Unknown astra_status_t value");
  return it->second;
}

sensor_msgs::Image astra_ros::toRos(const std::uint8_t *const data, const std::size_t length, const astra_image_metadata_t &metadata)
{
  sensor_msgs::Image ret;
  ret.height = metadata.height;
  ret.width = metadata.width;
  const auto it = ENCODING_MAPPING.find(metadata.pixelFormat);
  if (it == ENCODING_MAPPING.cend()) throw std::runtime_error("Can't convert pixel format to sensor_msgs::Image");
  ret.encoding = it->second.type;
  ret.is_bigendian = false;
  ret.step = ret.width * it->second.byte_length;
  ret.data.insert(ret.data.end(), data, data + length);
  return ret;
}

sensor_msgs::Image astra_ros::toRos(const std::int16_t *const data, const std::size_t length, const astra_image_metadata_t &metadata)
{
  sensor_msgs::Image ret;
  ret.height = metadata.height;
  ret.width = metadata.width;
  ret.encoding = sensor_msgs::image_encodings::MONO16;
  ret.is_bigendian = false;
  ret.step = ret.width * 2;

  // TODO: Verify endianess?
  ret.data.insert(ret.data.end(), reinterpret_cast<const std::uint8_t *>(data), reinterpret_cast<const std::uint8_t *>(data) + length);
  return ret;
}

sensor_msgs::Image astra_ros::toRos(const astra_rgba_pixel_t *const data, const std::size_t length, const astra_image_metadata_t &metadata)
{
  sensor_msgs::Image ret;
  ret.height = metadata.height;
  ret.width = metadata.width;
  
  ret.encoding = sensor_msgs::image_encodings::RGBA8;
  ret.is_bigendian = false;
  ret.step = ret.width * 4;

  // astra_rgba_pixel_t isn't packed, so we have to manually copy
  ret.data.reserve(length * 4);
  for (std::size_t i = 0; i < length; ++i)
  {
    ret.data.push_back(data[i].r);
    ret.data.push_back(data[i].g);
    ret.data.push_back(data[i].b);
    ret.data.push_back(data[i].alpha);
  }

  return ret;
}

sensor_msgs::Image astra_ros::toRos(const astra_floormask_t &floor_mask)
{
  sensor_msgs::Image ret;
  ret.height = floor_mask.height;
  ret.width = floor_mask.width;
  ret.encoding = sensor_msgs::image_encodings::MONO8;
  ret.is_bigendian = false;
  ret.step = ret.width;

  const std::size_t size = floor_mask.height * floor_mask.width;
  ret.data.reserve(size);
  for (std::size_t i = 0; i < size; ++i)
  {
    ret.data.push_back(floor_mask.data[i] ? 0xFF : 0x00);
  }
  
  return ret;
}

astra_ros::Body astra_ros::toRos(const astra_body_t &body, const std_msgs::Header &header)
{
  Body ret;
  ret.header = header;
  ret.id = body.id;
  ret.status = static_cast<std::uint8_t>(body.status);
  ret.center_of_mass.x = body.centerOfMass.x / 1000.0;
  ret.center_of_mass.y = -body.centerOfMass.y / 1000.0;
  ret.center_of_mass.z = body.centerOfMass.z / 1000.0;
  ret.is_tracking_joints = body.features & ASTRA_BODY_TRACKING_JOINTS;
  ret.is_tracking_hand_poses = body.features & ASTRA_BODY_TRACKING_HAND_POSES;

  for (std::size_t i = 0; i < ASTRA_MAX_JOINTS; ++i)
  {
    if (body.joints[i].type == ASTRA_JOINT_UNKNOWN) continue;

    Joint joint;
    joint.type = static_cast<std::uint8_t>(body.joints[i].type);
    joint.status = static_cast<std::uint8_t>(body.joints[i].status);
    joint.depth_position.x = body.joints[i].depthPosition.x;
    joint.depth_position.y = body.joints[i].depthPosition.y;
    joint.pose.position.x = body.joints[i].worldPosition.x / 1000.0;
    joint.pose.position.y = -body.joints[i].worldPosition.y / 1000.0;
    joint.pose.position.z = body.joints[i].worldPosition.z / 1000.0;

    const auto &orientation = body.joints[i].orientation;

    Eigen::Matrix3f mat;
    mat << orientation.m00, orientation.m01, orientation.m02,
           orientation.m10, orientation.m11, orientation.m12,
           orientation.m20, orientation.m21, orientation.m22;

    // We convert from the left-handed Z-up coordinate system of the Astra SDK
    // to the right-handed Z-up coordinate system of ROS
    Eigen::Matrix3f P;
    P << 1, 0 , 0,
         0, -1, 0,
         0, 0 , 1;
    
    Eigen::Quaternionf q(mat * P);
    q.normalize();
    joint.pose.orientation.x = q.x();
    joint.pose.orientation.y = q.y();
    joint.pose.orientation.z = q.z();
    joint.pose.orientation.w = q.w();
    

    ret.joints.emplace_back(std::move(joint));
    const auto &back = ret.joints.back();
  }

  

  ret.left_hand_pose = static_cast<std::uint8_t>(body.handPoses.leftHand);
  ret.right_hand_pose = static_cast<std::uint8_t>(body.handPoses.rightHand);
  return ret;
}


std::vector<astra_ros::Body> astra_ros::toRos(const astra_body_list_t &body_list, const std_msgs::Header &header)
{
  std::vector<Body> ret;
  ret.reserve(body_list.count);
  for (std::int32_t i = 0; i < body_list.count; ++i)
  {
    ret.push_back(toRos(body_list.bodies[i], header));
  }

  return ret;
}

astra_ros::Plane astra_ros::toRos(const astra_plane_t &plane)
{
  Plane ret;
  ret.a = plane.a;
  ret.b = plane.b;
  ret.c = plane.c;
  ret.d = plane.d;
  return ret;
}


astra_ros::ImageStreamMode astra_ros::toRos(const astra_ros::Device::ImageStreamMode &image_stream_mode)
{
  ImageStreamMode ret;
  ret.width = image_stream_mode.width.get_value_or(0);
  ret.height = image_stream_mode.height.get_value_or(0);
  ret.fps = image_stream_mode.fps.get_value_or(0);
  ret.pixel_format = static_cast<std::uint32_t>(image_stream_mode.pixel_format.get_value_or(ASTRA_PIXEL_FORMAT_UNKNOWN));
  return ret;
}

astra_ros::Device::ImageStreamMode astra_ros::fromRos(const astra_ros::ImageStreamMode &image_stream_mode)
{
  Device::ImageStreamMode ret;
  if (image_stream_mode.width != 0) ret.width = image_stream_mode.width;
  if (image_stream_mode.height != 0) ret.height = image_stream_mode.height;
  if (image_stream_mode.fps != 0) ret.fps = image_stream_mode.fps;
  if (image_stream_mode.pixel_format != ImageStreamMode::PIXEL_FORMAT_UNKNOWN) ret.pixel_format = static_cast<astra_pixel_format_t>(image_stream_mode.pixel_format);
  return ret;
}
