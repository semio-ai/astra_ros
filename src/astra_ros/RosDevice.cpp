#include "astra_ros/RosDevice.hpp"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/CameraInfo.h>

#include <string>
#include <iostream>

#include <eigen3/Eigen/Dense>

#include "util.hpp"
#include "tf.hpp"
#include "visualization.hpp"
#include "point_cloud.hpp"

using namespace astra_ros;

namespace
{
  // For some reason ros::Publisher and image_transport::Publisher can cast to void * for determining
  // validity. (void *)1 is valid, (void *)0 is invalid.
  // T must be a Publisher type
  template<typename T>
  bool isPublisherValid(const T &publisher)
  {
    return (void *)publisher == (void *)1;
  }

  // Wraps a ROS param for a boost::optional
  template<typename T>
  bool optionalParam(const ros::NodeHandle &nh, const std::string &key, boost::optional<T> &value)
  {
    T raw;
    if (!nh.getParam(key, raw))
    {
      value = boost::none;
      return false;
    }

    value = raw;
    return true;
  }

  // Wraps a ros param for a boost::optional<Parameter>
  template<typename T>
  bool liveParam(const ros::NodeHandle &nh, const std::string &key, Parameter<T> &value, const T &default_value)
  {
    T raw;
    if (!nh.getParam(key, raw))
    {
      value.set(default_value);
      return false;
    }

    value.set(raw);
    return true;
  }

  // Wraps a ros param for a boost::optional<Parameter>
  template<typename T>
  bool optionalLiveParam(const ros::NodeHandle &nh, const std::string &key, boost::optional<Parameter<T>> &value, const T &default_value)
  {
    T raw;
    if (!nh.getParam(key, raw))
    {
      value = Parameter<T>(default_value);
      if (value) value->set(default_value);
      else value = Parameter<T>(default_value);
      return false;
    }

    if (value) value->set(raw);
    else value = Parameter<T>(raw);

    return true;
  }

  template<typename Iter>
  Joint *findJoint(const Iter begin, const Iter end, const Joint::_type_type type)
  {
    for (auto it = begin; it != end; ++it)
    {
      if (it->type != type) continue;
      return &*it;
    }

    return nullptr;
  }
}

const std::string astra_ros::DEVICE_NAMESPACE("devices");

RosDevice::RosDevice(const std::string &name, ros::NodeHandle &nh, ros::NodeHandle &pnh)
  : name_(name)
  , nh_(nh)
  , device_nh_(ros::NodeHandle(pnh, DEVICE_NAMESPACE + "/" + name))
  , pnh_(pnh)
  , image_transport_(device_nh_)
  , mut_(std::make_unique<boost::recursive_mutex>())
  , dynamic_reconfigure_server_(std::make_unique<DeviceConfigServer>(*mut_, device_nh_))
  , publish_body_markers(false)
  , publish_body_mask(true)
  , publish_floor_mask(true)
  , body_frame_id("/camera")
{
  device_nh_.getParam("body/publish_body_markers", publish_body_markers);
  device_nh_.getParam("body/publish_body_mask", publish_body_mask);
  device_nh_.getParam("body/publish_floor_mask", publish_floor_mask);
  device_nh_.getParam("body/frame_id", body_frame_id);

  Device::Configuration config = getConfiguration(device_nh_);
  config.on_frame = std::bind(&RosDevice::onFrame, this, std::placeholders::_1);
  device_ = Device::open(config);

  camera_parameters_ = device_->getCameraParameters();
  
  auto &color_stream = config.color_stream;
  auto &ir_stream = config.ir_stream;
  auto &depth_stream = config.depth_stream;

  // Make sure the dynamic reconfigure configuration is synchronized with
  // the user-provided static configuration
  DeviceConfig dyn_config;
  dyn_config.color_stream_running = color_stream && color_stream->running.get();
  dyn_config.color_stream_mirrored = color_stream && color_stream->mirrored && color_stream->mirrored->get();

  dyn_config.ir_stream_running = ir_stream && ir_stream->running.get();
  dyn_config.ir_stream_mirrored = ir_stream && ir_stream->mirrored && ir_stream->mirrored->get();

  dyn_config.depth_stream_running = depth_stream && depth_stream->running.get();
  dyn_config.depth_stream_mirrored = depth_stream && depth_stream->mirrored && depth_stream->mirrored->get();

  dynamic_reconfigure_server_->updateConfig(dyn_config);

  dynamic_reconfigure_server_->setCallback(std::bind(&RosDevice::onDynamicReconfigure, this, std::placeholders::_1, std::placeholders::_2));

  // Configure services
  if (color_stream)
  {
    get_color_usb_info_svc_ = device_nh_.advertiseService("color/get_usb_info", &RosDevice::onGetColorUsbInfo, this);
    get_color_image_stream_modes_svc_ = device_nh_.advertiseService("color/get_image_stream_modes", &RosDevice::onGetColorImageStreamModes, this);
    get_color_image_stream_mode_svc_ = device_nh_.advertiseService("color/get_image_stream_mode", &RosDevice::onGetColorImageStreamMode, this);
    set_color_image_stream_mode_svc_ = device_nh_.advertiseService("color/set_image_stream_modes", &RosDevice::onSetColorImageStreamMode, this);
    get_color_running_svc_ = device_nh_.advertiseService("color/get_running", &RosDevice::onGetColorRunning, this);
    set_color_running_svc_ = device_nh_.advertiseService("color/set_running", &RosDevice::onSetColorRunning, this);

    if (color_stream->mirrored)
    {
      get_color_mirrored_svc_ = device_nh_.advertiseService("color/get_mirrored", &RosDevice::onGetColorMirrored, this);
      set_color_mirrored_svc_ = device_nh_.advertiseService("color/set_mirrored", &RosDevice::onSetColorMirrored, this);
    }
  }

  if (ir_stream)
  {
    get_ir_usb_info_svc_ = device_nh_.advertiseService("ir/get_usb_info", &RosDevice::onGetIrUsbInfo, this);
    get_ir_image_stream_mode_svc_ = device_nh_.advertiseService("ir/get_image_stream_mode", &RosDevice::onGetIrImageStreamMode, this);
    get_ir_image_stream_modes_svc_ = device_nh_.advertiseService("ir/get_image_stream_modes", &RosDevice::onGetIrImageStreamModes, this);
    set_ir_image_stream_mode_svc_ = device_nh_.advertiseService("ir/set_image_stream_modes", &RosDevice::onSetIrImageStreamMode, this);

    get_ir_running_svc_ = device_nh_.advertiseService("ir/get_running", &RosDevice::onGetIrRunning, this);
    set_ir_running_svc_ = device_nh_.advertiseService("ir/set_running", &RosDevice::onSetIrRunning, this);

    if (ir_stream->mirrored)
    {
      get_ir_mirrored_svc_ = device_nh_.advertiseService("ir/get_mirrored", &RosDevice::onGetIrMirrored, this);
      set_ir_mirrored_svc_ = device_nh_.advertiseService("ir/set_mirrored", &RosDevice::onSetIrMirrored, this);
    }

    if (ir_stream->exposure)
    {
      get_ir_exposure_svc_ = device_nh_.advertiseService("ir/get_exposure", &RosDevice::onGetIrExposure, this);
      set_ir_exposure_svc_ = device_nh_.advertiseService("ir/set_exposure", &RosDevice::onSetIrExposure, this);
    }

    if (ir_stream->gain)
    {
      get_ir_gain_svc_ = device_nh_.advertiseService("ir/get_gain", &RosDevice::onGetIrGain, this);
      set_ir_gain_svc_ = device_nh_.advertiseService("ir/set_gain", &RosDevice::onSetIrGain, this);
    }
  }

  if (depth_stream)
  {
    // For some reason these are tied to the depth sensor
    get_chip_id_svc_ = device_nh_.advertiseService("depth/get_chip_id", &RosDevice::onGetChipId, this);
    
    get_depth_registration_svc_ = device_nh_.advertiseService("depth/get_registration", &RosDevice::onGetDepthRegistration, this);
    set_depth_registration_svc_ = device_nh_.advertiseService("depth/set_registration", &RosDevice::onSetDepthRegistration, this);
    
    get_serial_svc_ = device_nh_.advertiseService("depth/get_serial", &RosDevice::onGetSerial, this);
    get_depth_image_stream_mode_svc_ = device_nh_.advertiseService("depth/get_image_stream_mode", &RosDevice::onGetDepthImageStreamMode, this);
    get_depth_image_stream_modes_svc_ = device_nh_.advertiseService("depth/get_image_stream_modes", &RosDevice::onGetDepthImageStreamModes, this);
    set_depth_image_stream_mode_svc_ = device_nh_.advertiseService("depth/set_image_stream_mode", &RosDevice::onSetDepthImageStreamMode, this);
    
    get_depth_usb_info_svc_ = device_nh_.advertiseService("depth/get_usb_info", &RosDevice::onGetDepthUsbInfo, this);
    get_depth_running_svc_ = device_nh_.advertiseService("depth/get_running", &RosDevice::onGetDepthRunning, this);
    set_depth_running_svc_ = device_nh_.advertiseService("depth/set_running", &RosDevice::onSetDepthRunning, this);

    if (depth_stream->mirrored)
    {
      get_depth_mirrored_svc_ = device_nh_.advertiseService("depth/get_mirrored", &RosDevice::onGetDepthMirrored, this);
      set_depth_mirrored_svc_ = device_nh_.advertiseService("depth/set_mirrored", &RosDevice::onSetDepthMirrored, this);
    }
  }


}

const std::string &RosDevice::getName() const noexcept
{
  return name_;
}

void RosDevice::update()
{
  device_->update();
}

Device::Configuration RosDevice::getConfiguration(ros::NodeHandle &nh)
{
  using namespace std::string_literals;

  Device::Configuration config;
  
  std::string uri;
  optionalParam(nh, "uri", config.uri);

  if (nh.hasParam("color"))
  {
    ros::NodeHandle color_nh(nh, "color");

    Device::Configuration::ColorStream color_stream;

    liveParam(color_nh, "running", color_stream.running, true);
    optionalLiveParam(color_nh, "mirrored", color_stream.mirrored, false);
  
    config.color_stream = color_stream;
  }

  if (nh.hasParam("ir"))
  {
    ros::NodeHandle ir_nh(nh, "ir");

    Device::Configuration::IrStream ir_stream;
    
    liveParam(ir_nh, "running", ir_stream.running, true);
    optionalLiveParam(ir_nh, "mirrored", ir_stream.mirrored, false);
    
    config.ir_stream = ir_stream;
  }

  if (nh.hasParam("depth"))
  {
    ros::NodeHandle depth_nh(nh, "depth");

    Device::Configuration::DepthStream depth_stream;

    liveParam(depth_nh, "running", depth_stream.running, true);
    optionalLiveParam(depth_nh, "mirrored", depth_stream.mirrored, false);

    optionalLiveParam(depth_nh, "d2c_mode", depth_stream.d2c_mode, 0);
    optionalLiveParam(depth_nh, "registration", depth_stream.registration, true);

    config.depth_stream = depth_stream;
  }

  if (nh.hasParam("body"))
  {
    ros::NodeHandle body_nh(nh, "body");

    Device::Configuration::BodyStream body_stream;

    liveParam(body_nh, "running", body_stream.running, true);
    optionalParam(body_nh, "license", body_stream.license);

    config.body_stream = body_stream;
  }

  if (nh.hasParam("colorized_body"))
  {
    ros::NodeHandle colorized_body_nh(nh, "colorized_body");

    Device::Configuration::ColorizedBodyStream colorized_body_stream;

    liveParam(colorized_body_nh, "running", colorized_body_stream.running, true);

    config.colorized_body_stream = colorized_body_stream;
  }

  if (nh.hasParam("hand"))
  {
    ros::NodeHandle hand_nh(nh, "hand");

    Device::Configuration::HandStream hand_stream;

    liveParam(hand_nh, "running", hand_stream.running, true);

    config.hand_stream = hand_stream;
  }

  if (nh.hasParam("masked_color"))
  {
    ros::NodeHandle masked_color_nh(nh, "masked_color");

    Device::Configuration::MaskedColorStream masked_color_stream;

    liveParam(masked_color_nh, "running", masked_color_stream.running, true);

    config.masked_color_stream = masked_color_stream;
  }

  if (nh.hasParam("point"))
  {
    ros::NodeHandle point_nh(nh, "point");

    Device::Configuration::PointStream point_stream;

    liveParam(point_nh, "running", point_stream.running, true);

    config.point_stream = point_stream;
  }
  
  return config;
}

void RosDevice::onFrame(const Device::Frame &frame)
{
  const ros::Time now = ros::Time::now();

  // If all of these are present we attempt to generate point clouds
  boost::optional<sensor_msgs::Image> color_image;
  boost::optional<sensor_msgs::Image> depth_image;
  boost::optional<sensor_msgs::CameraInfo> color_camera_info;

  if (frame.color)
  {
    if (!isPublisherValid(color_image_pub_))
    {
      color_image_pub_ = image_transport_.advertise("color/image_color", 1);
    }

    if (!isPublisherValid(color_camera_info_pub_))
    {
      color_camera_info_pub_ = device_nh_.advertise<sensor_msgs::CameraInfo>("color/camera_info", 1, true);
    }

    color_image = toRos(frame.color->data, frame.color->data_length, frame.color->metadata);  
    color_image_pub_.publish(*color_image);
  
    // Camera Info
    // TODO: orbbec_camera_params is undocumented,
    // so we need to verify this is correct.
    sensor_msgs::CameraInfo camera_info;
    camera_info.header.stamp = now;
    camera_info.header.frame_id = body_frame_id;
    camera_info.height = frame.color->metadata.height;
    camera_info.width = frame.color->metadata.width;
    camera_info.distortion_model = "plumb_bob";
    
    camera_info.D.resize(5, 0.0);
    camera_info.D[0] = camera_parameters_.l_k[0];
    camera_info.D[1] = camera_parameters_.l_k[1];
    camera_info.D[2] = camera_parameters_.l_k[2];
    camera_info.D[3] = camera_parameters_.l_k[3];
    camera_info.D[4] = camera_parameters_.l_k[4];
  
    // camera_info.K.resize(9, 0.0);
    camera_info.K[0] = camera_parameters_.l_intr_p[0];
    camera_info.K[2] = camera_parameters_.l_intr_p[2];
    camera_info.K[4] = camera_parameters_.l_intr_p[1];
    camera_info.K[5] = camera_parameters_.l_intr_p[3];
    camera_info.K[8] = 1.0;

    color_camera_info = camera_info;
    color_camera_info_pub_.publish(camera_info);
  }
  else
  {
    color_image_pub_ = image_transport::Publisher();
    color_camera_info_pub_ = ros::Publisher();
  }

  if (frame.ir)
  {
    if (!isPublisherValid(ir_image_pub_))
    {
      ir_image_pub_ = image_transport_.advertise("ir/image", 1);
    }

    ir_image_pub_.publish(toRos(frame.ir->data, frame.ir->data_length, frame.ir->metadata));
  }
  else
  {
    ir_image_pub_ = image_transport::Publisher();
  }

  if (frame.depth)
  {
    if (!isPublisherValid(depth_image_pub_))
    {
      depth_image_pub_ = image_transport_.advertise("depth/image", 1);
    }

    if (!isPublisherValid(depth_camera_info_pub_))
    {
      depth_camera_info_pub_ = device_nh_.advertise<sensor_msgs::CameraInfo>("depth/camera_info", 1, true);
    }


    depth_image = toRos(frame.depth->data, frame.depth->data_length, frame.depth->metadata);
    depth_image_pub_.publish(*depth_image);

    // Camera Info
    // TODO: orbbec_camera_params is undocumented,
    // so we need to verify this is correct.
    sensor_msgs::CameraInfo camera_info;
    camera_info.header.stamp = now;
    camera_info.header.frame_id = body_frame_id;
    camera_info.height = frame.depth->metadata.height;
    camera_info.width = frame.depth->metadata.width;
    camera_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    camera_info.D.resize(5, 0.0);
    camera_info.D[0] = camera_parameters_.r_k[0];
    camera_info.D[1] = camera_parameters_.r_k[1];
    camera_info.D[2] = camera_parameters_.r_k[2];
    camera_info.D[3] = camera_parameters_.r_k[3];
    camera_info.D[4] = camera_parameters_.r_k[4];
  
    camera_info.K[0] = camera_parameters_.r_intr_p[0];
    camera_info.K[2] = camera_parameters_.r_intr_p[2];
    camera_info.K[4] = camera_parameters_.r_intr_p[1];
    camera_info.K[5] = camera_parameters_.r_intr_p[3];
    camera_info.K[8] = 1.0;

    depth_camera_info_pub_.publish(camera_info);
  }
  else
  {
    depth_image_pub_ = image_transport::Publisher();
    depth_camera_info_pub_ = ros::Publisher();
  }

  if (frame.body)
  {
    if (!isPublisherValid(body_frame_pub_))
    {
      body_frame_pub_ = device_nh_.advertise<BodyFrame>("body/frame", 1);
    }

    if (publish_body_markers)
    {
      if (!isPublisherValid(body_markers_pub_))
      {
        body_markers_pub_ = device_nh_.advertise<visualization_msgs::MarkerArray>("body/markers", 1);
      }
    }
    else
    {
      body_markers_pub_ = ros::Publisher();
    }
    

    if (publish_body_mask)
    {
      if (!isPublisherValid(body_mask_image_pub_))
      {
        body_mask_image_pub_ = image_transport_.advertise("body/mask", 1);
      }
    }
    else
    {
      body_mask_image_pub_ = image_transport::Publisher();
    }
    

    if (publish_floor_mask)
    {
      if (!isPublisherValid(floor_mask_image_pub_))
      {
        floor_mask_image_pub_ = image_transport_.advertise("body/floor_mask", 1);
      }
    }
    else
    {
      floor_mask_image_pub_ = image_transport::Publisher();
    }

    std_msgs::Header header;
    header.stamp = now;
    header.frame_id = body_frame_id;

    BodyFrame body_frame;
    body_frame.bodies = toRos(frame.body->body_list, header);
    body_frame.floor_detected = frame.body->floor_info->floorDetected;
    body_frame.floor_plane = toRos(frame.body->floor_info->floorPlane);

    const double fx = color_camera_info->K[0];
    const double fy = color_camera_info->K[4];
    const double cx = color_camera_info->K[2];
    const double cy = color_camera_info->K[5];

    Eigen::Matrix3d intrinsics;
    intrinsics <<
      fx , 0.0, cx,
      0.0, fy , cy,
      0.0, 0.0, 1.0;

    if (color_camera_info)
    {
      for (auto &body : body_frame.bodies)
      {
        const Joint *const head = findJoint(body.joints.begin(), body.joints.end(), Joint::TYPE_HEAD);
        if (!head) continue;

        // ~17 cm width
        const static double AVERAGE_HUMAN_HEAD_Y = 0.01778;

        // ~22 cm height
        const static double AVERAGE_HUMAN_HEAD_Z = 0.02286;
        
        const auto &position = head->pose.position;

        Eigen::Vector3d top_left(
          position.x,
          position.y + AVERAGE_HUMAN_HEAD_Y / 2.0,
          position.z + AVERAGE_HUMAN_HEAD_Z / 2.0
        );

        Eigen::Vector3d bottom_right(
          position.x,
          position.y - AVERAGE_HUMAN_HEAD_Y / 2.0,
          position.z - AVERAGE_HUMAN_HEAD_Z / 2.0
        );

        const auto transformed_top_left = intrinsics * top_left;
        const auto transformed_bottom_right = intrinsics * bottom_right;

        auto &bounding_box = body.face_bounding_box;

        bounding_box.top_left.x = transformed_top_left.x();
        bounding_box.top_left.y = transformed_top_left.y();

        bounding_box.bottom_right.x = transformed_bottom_right.x();
        bounding_box.bottom_right.y = transformed_bottom_right.y();
      }
    }

   

    if (depth_image && color_image && color_camera_info)
    {
      const auto &body_mask = frame.body->body_mask;

      for (auto &body : body_frame.bodies)
      {
        const std::uint8_t id = body.id;
        body.point_cloud = toPointCloud(*color_image, *depth_image, *color_camera_info, [&] (const std::size_t x, const std::size_t y) -> bool {
          return x < body_mask.width && y < body_mask.height && body_mask.data[y * body_mask.width + x] == id;
        });
      }
    }

    body_frame_pub_.publish(body_frame);

    if (isPublisherValid(body_mask_image_pub_)) body_mask_image_pub_.publish(toRos(frame.body->body_mask));
    if (isPublisherValid(floor_mask_image_pub_)) floor_mask_image_pub_.publish(toRos(frame.body->floor_info->floorMask));

    if (isPublisherValid(body_markers_pub_)) body_markers_pub_.publish(toMarkerArray(body_frame.bodies));

    // TF2
    if (!tf_broadcaster_)
    {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();
    }

    std::vector<geometry_msgs::TransformStamped> transforms;
    for (const auto &body : body_frame.bodies)
    {
      tf_broadcaster_->sendTransform(bodyTransforms(body));
    }
  }
  else
  {
    body_frame_pub_ = ros::Publisher();
    body_mask_image_pub_ = image_transport::Publisher();
    floor_mask_image_pub_ = image_transport::Publisher();
    body_markers_pub_ = ros::Publisher();
  }

  if (frame.colorized_body)
  {
    if (!isPublisherValid(colorized_body_image_pub_))
    {
      colorized_body_image_pub_ = image_transport_.advertise("colorized_body/image", 1);
    }

    colorized_body_image_pub_.publish(toRos(frame.colorized_body->data, frame.colorized_body->data_length, frame.colorized_body->metadata));
  }
  else
  {
    colorized_body_image_pub_ = image_transport::Publisher();
  }

  if (frame.masked_color)
  {
    if (!isPublisherValid(colorized_body_image_pub_))
    {
      masked_color_image_pub_ = image_transport_.advertise("masked_color/image", 1);
    }

    masked_color_image_pub_.publish(toRos(frame.masked_color->data, frame.masked_color->data_length, frame.masked_color->metadata));
  }
  else
  {
    masked_color_image_pub_ = image_transport::Publisher();
  }

  if (depth_image && color_image && color_camera_info)
  {
    if (!isPublisherValid(point_cloud_pub_))
    {
      point_cloud_pub_ = device_nh_.advertise<sensor_msgs::PointCloud>("point_cloud", 1);
    }

    // Generate a point cloud
    auto point_cloud = toPointCloud(*color_image, *depth_image, *color_camera_info);
    point_cloud.header.stamp = now;
    point_cloud.header.frame_id = body_frame_id;

    // std::cout << body_frame_id << std::endl;
    point_cloud_pub_.publish(point_cloud);
  }
  else
  {
    point_cloud_pub_ = ros::Publisher();
  }
}

void RosDevice::onDynamicReconfigure(DeviceConfig &config, uint32_t level)
{
  auto &color_stream = device_->getConfiguration().color_stream;
  if (color_stream)
  {
    color_stream->running.set(config.color_stream_running);

    if (color_stream->mirrored)
    {
      color_stream->mirrored->set(config.color_stream_mirrored);
    }
  }

  auto &ir_stream = device_->getConfiguration().ir_stream;
  if (ir_stream)
  {
    ir_stream->running.set(config.ir_stream_running);

    if (ir_stream->mirrored)
    {
      ir_stream->mirrored->set(config.ir_stream_mirrored);
    }

    if (ir_stream->exposure)
    {
      ir_stream->exposure->set(config.ir_exposure);
    }
  }

  auto &depth_stream = device_->getConfiguration().depth_stream;
  if (depth_stream)
  {
    depth_stream->running.set(config.depth_stream_running);

    if (depth_stream->mirrored)
    {
      depth_stream->mirrored->set(config.depth_stream_mirrored);
    }
  }
}

bool RosDevice::onGetChipId(GetChipId::Request &req, GetChipId::Response &res)
{
  const auto chip_id = device_->getChipId();
  if (!chip_id) return false;
  res.chip_id = *chip_id;
  return true;
}

bool RosDevice::onGetDepthRegistration(GetDepthRegistration::Request &req, GetDepthRegistration::Response &res)
{
  const auto &depth_stream = device_->getConfiguration().depth_stream;
  if (!depth_stream) return false;

  const auto &registration = depth_stream->registration;
  if (!registration) return false;

  res.registration = **registration;

  return true;
}

bool RosDevice::onGetColorImageStreamMode(GetImageStreamMode::Request &req, GetImageStreamMode::Response &res)
{
  const auto &color_stream = device_->getConfiguration().color_stream;
  if (!color_stream) return false;

  const auto &mode = color_stream->mode;
  if (!mode) return false;

  res.image_stream_mode = toRos(**mode);

  return true;
}

bool RosDevice::onGetDepthImageStreamMode(GetImageStreamMode::Request &req, GetImageStreamMode::Response &res)
{
  const auto &depth_stream = device_->getConfiguration().depth_stream;
  if (!depth_stream) return false;

  const auto &mode = depth_stream->mode;
  if (!mode) return false;

  res.image_stream_mode = toRos(**mode);

  return true;
}

bool RosDevice::onGetIrImageStreamMode(GetImageStreamMode::Request &req, GetImageStreamMode::Response &res)
{
  const auto &ir_stream = device_->getConfiguration().ir_stream;
  if (!ir_stream) return false;

  const auto &mode = ir_stream->mode;
  if (!mode) return false;

  res.image_stream_mode = toRos(**mode);

  return true;
}


bool RosDevice::onGetColorImageStreamModes(GetImageStreamModes::Request &req, GetImageStreamModes::Response &res)
{
  const auto image_stream_modes = device_->getColorImageStreamModes();
  if (!image_stream_modes) return false;
  
  res.image_stream_modes.reserve(image_stream_modes->size());
  for (const auto &image_stream_mode : *image_stream_modes)
  {
    res.image_stream_modes.push_back(toRos(image_stream_mode));
  }

  return true;
}

bool RosDevice::onGetDepthImageStreamModes(GetImageStreamModes::Request &req, GetImageStreamModes::Response &res)
{
  const auto image_stream_modes = device_->getDepthImageStreamModes();
  if (!image_stream_modes) return false;
  
  res.image_stream_modes.reserve(image_stream_modes->size());
  for (const auto &image_stream_mode : *image_stream_modes)
  {
    res.image_stream_modes.push_back(toRos(image_stream_mode));
  }

  return true;
}

bool RosDevice::onGetIrImageStreamModes(GetImageStreamModes::Request &req, GetImageStreamModes::Response &res)
{
  const auto image_stream_modes = device_->getIrImageStreamModes();
  if (!image_stream_modes) return false;

  res.image_stream_modes.reserve(image_stream_modes->size());
  for (const auto &image_stream_mode : *image_stream_modes)
  {
    res.image_stream_modes.push_back(toRos(image_stream_mode));
  }

  return true;
}

bool RosDevice::onGetIrExposure(GetIrExposure::Request &req, GetIrExposure::Response &res)
{
  const auto &ir_stream = device_->getConfiguration().ir_stream;
  if (!ir_stream) return false;
  
  const auto &exposure = ir_stream->exposure;
  if (!exposure) return false;
  
  res.exposure = **exposure;
  
  return true;
}

bool RosDevice::onGetIrGain(GetIrGain::Request &req, GetIrGain::Response &res)
{
  const auto &ir_stream = device_->getConfiguration().ir_stream;
  if (!ir_stream) return false;
  
  const auto &gain = ir_stream->gain;
  if (!gain) return false;
  
  res.gain = **gain;
  
  return true;
}

bool RosDevice::onGetColorMirrored(GetMirrored::Request &req, GetMirrored::Response &res)
{
  const auto &color_stream = device_->getConfiguration().color_stream;
  if (!color_stream) return false;
  
  const auto &mirrored = color_stream->mirrored;
  if (!mirrored) return false;

  res.mirrored = **mirrored;

  return true;
}

bool RosDevice::onGetDepthMirrored(GetMirrored::Request &req, GetMirrored::Response &res)
{
  const auto &depth_stream = device_->getConfiguration().depth_stream;
  if (!depth_stream) return false;
  
  const auto &mirrored = depth_stream->mirrored;
  if (!mirrored) return false;

  res.mirrored = **mirrored;

  return true;
}

bool RosDevice::onGetIrMirrored(GetMirrored::Request &req, GetMirrored::Response &res)
{
  const auto &ir_stream = device_->getConfiguration().ir_stream;
  if (!ir_stream) return false;
  
  const auto &mirrored = ir_stream->mirrored;
  if (!mirrored) return false;

  res.mirrored = **mirrored;

  return true;
}

bool RosDevice::onGetColorRunning(GetRunning::Request &req, GetRunning::Response &res)
{
  const auto &color_stream = device_->getConfiguration().color_stream;
  if (!color_stream) return false;

  res.running = *color_stream->running;
  return true;
}

bool RosDevice::onGetDepthRunning(GetRunning::Request &req, GetRunning::Response &res)
{
  const auto &depth_stream = device_->getConfiguration().depth_stream;
  if (!depth_stream) return false;

  res.running = *depth_stream->running;
  return true;
}

bool RosDevice::onGetIrRunning(GetRunning::Request &req, GetRunning::Response &res)
{
  const auto &ir_stream = device_->getConfiguration().ir_stream;
  if (!ir_stream) return false;

  res.running = *ir_stream->running;
  return true;
}

bool RosDevice::onGetSerial(GetSerial::Request &req, GetSerial::Response &res)
{
  const auto serial_number = device_->getSerialNumber();
  if (!serial_number) return false;
  res.serial = *serial_number;
  return true;
}

bool RosDevice::onGetColorUsbInfo(GetUsbInfo::Request &req, GetUsbInfo::Response &res)
{
  const auto usb_info = device_->getColorUsbInfo();
  if (!usb_info) return false;
  res.usb_info.pid = usb_info->pid;
  res.usb_info.vid = usb_info->vid;
  return true;
}

bool RosDevice::onGetDepthUsbInfo(GetUsbInfo::Request &req, GetUsbInfo::Response &res)
{
  const auto usb_info = device_->getDepthUsbInfo();
  if (!usb_info) return false;
  res.usb_info.pid = usb_info->pid;
  res.usb_info.vid = usb_info->vid;
  return true;
}

bool RosDevice::onGetIrUsbInfo(GetUsbInfo::Request &req, GetUsbInfo::Response &res)
{
  const auto usb_info = device_->getIrUsbInfo();
  if (!usb_info) return false;
  res.usb_info.pid = usb_info->pid;
  res.usb_info.vid = usb_info->vid;
  return true;
}

bool RosDevice::onSetDepthRegistration(SetDepthRegistration::Request &req, SetDepthRegistration::Response &res)
{
  auto &depth_stream = device_->getConfiguration().depth_stream;
  if (!depth_stream) return false;

  auto &registration = depth_stream->registration;
  if (!registration) return false;

  registration->set(req.registration);

  return true;
}


bool RosDevice::onSetColorImageStreamMode(SetImageStreamMode::Request &req, SetImageStreamMode::Response &res)
{
  auto &color_stream = device_->getConfiguration().color_stream;
  if (!color_stream) return false;

  auto &mode = color_stream->mode;
  if (!mode) return false;

  mode->set(fromRos(req.image_stream_mode));

  return true;
}

bool RosDevice::onSetDepthImageStreamMode(SetImageStreamMode::Request &req, SetImageStreamMode::Response &res)
{
  auto &depth_stream = device_->getConfiguration().depth_stream;
  if (!depth_stream) return false;

  auto &mode = depth_stream->mode;
  if (!mode) return false;

  mode->set(fromRos(req.image_stream_mode));

  return true;
}

bool RosDevice::onSetIrImageStreamMode(SetImageStreamMode::Request &req, SetImageStreamMode::Response &res)
{
  auto &ir_stream = device_->getConfiguration().ir_stream;
  if (!ir_stream) return false;

  auto &mode = ir_stream->mode;
  if (!mode) return false;

  mode->set(fromRos(req.image_stream_mode));

  return true;
}

bool RosDevice::onSetIrExposure(SetIrExposure::Request &req, SetIrExposure::Response &res)
{
  auto &ir_stream = device_->getConfiguration().ir_stream;
  if (!ir_stream) return false;
  
  auto &exposure = ir_stream->exposure;
  if (!exposure) return false;
  
  return exposure->set(req.exposure);
}

bool RosDevice::onSetIrGain(SetIrGain::Request &req, SetIrGain::Response &res)
{
  auto &ir_stream = device_->getConfiguration().ir_stream;
  if (!ir_stream) return false;
  
  auto &gain = ir_stream->gain;
  if (!gain) return false;
  
  return gain->set(req.gain);
}

bool RosDevice::onSetColorMirrored(SetMirrored::Request &req, SetMirrored::Response &res)
{
  auto &color_stream = device_->getConfiguration().color_stream;
  if (!color_stream) return false;
  
  auto &mirrored = color_stream->mirrored;
  if (!mirrored) return false;

  return mirrored->set(req.mirrored);
}

bool RosDevice::onSetDepthMirrored(SetMirrored::Request &req, SetMirrored::Response &res)
{
  auto &depth_stream = device_->getConfiguration().depth_stream;
  if (!depth_stream) return false;
  
  auto &mirrored = depth_stream->mirrored;
  if (!mirrored) return false;

  return mirrored->set(req.mirrored);
}

bool RosDevice::onSetIrMirrored(SetMirrored::Request &req, SetMirrored::Response &res)
{
  auto &ir_stream = device_->getConfiguration().ir_stream;
  if (!ir_stream) return false;
  
  auto &mirrored = ir_stream->mirrored;
  if (!mirrored) return false;

  return mirrored->set(req.mirrored);
}

bool RosDevice::onSetColorRunning(SetRunning::Request &req, SetRunning::Response &res)
{
  auto &color_stream = device_->getConfiguration().color_stream;
  if (!color_stream) return false;
  
  return color_stream->running.set(req.running);
}

bool RosDevice::onSetDepthRunning(SetRunning::Request &req, SetRunning::Response &res)
{
  auto &depth_stream = device_->getConfiguration().depth_stream;
  if (!depth_stream) return false;
  
  return depth_stream->running.set(req.running);
}

bool RosDevice::onSetIrRunning(SetRunning::Request &req, SetRunning::Response &res)
{
  auto &ir_stream = device_->getConfiguration().ir_stream;
  if (!ir_stream) return false;
  
  return ir_stream->running.set(req.running);
}