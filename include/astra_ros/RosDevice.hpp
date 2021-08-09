#ifndef _ASTRA_ROS_ROS_DEVICE_HPP_
#define _ASTRA_ROS_ROS_DEVICE_HPP_

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <tf2_ros/transform_broadcaster.h>

#include "astra_ros/DeviceConfig.h"

#include "astra_ros/BodyFrame.h"

#include "astra_ros/GetChipId.h"
#include "astra_ros/GetDepthRegistration.h"
#include "astra_ros/GetImageStreamMode.h"
#include "astra_ros/GetImageStreamModes.h"
#include "astra_ros/GetIrExposure.h"
#include "astra_ros/GetIrGain.h"
#include "astra_ros/GetMirrored.h"
#include "astra_ros/GetRunning.h"
#include "astra_ros/GetSerial.h"
#include "astra_ros/GetUsbInfo.h"
#include "astra_ros/SetDepthRegistration.h"
#include "astra_ros/SetImageStreamMode.h"
#include "astra_ros/SetIrExposure.h"
#include "astra_ros/SetIrGain.h"
#include "astra_ros/SetMirrored.h"
#include "astra_ros/SetRunning.h"

#include "Device.hpp"

namespace astra_ros
{
  class RosDevice
  {
  public:
    /**
     * Construct a Device that interacts with ROS.
     * 
     * \param name The name of the device. This is used as a namespace for the ROS parameters under the private `NodeHandle`.
     * \param nh The public ROS `NodeHandle`
     * \param pnh The private ROS `NodeHandle`
     */
    RosDevice(const std::string &name, ros::NodeHandle &nh, ros::NodeHandle &pnh);

    /**
     * \return The name of the RosDevice (as configured in the constructor)
     */
    const std::string &getName() const noexcept;

    /**
     * Check if a new frame is available, and if so, publish all configured
     * data to ROS.
     */
    void update();

  private:
    static Device::Configuration getConfiguration(ros::NodeHandle &nh);

    void onFrame(const Device::Frame &frame);
    void onDynamicReconfigure(DeviceConfig &config, uint32_t level);

    bool onGetChipId(GetChipId::Request &req, GetChipId::Response &res);
    bool onGetDepthRegistration(GetDepthRegistration::Request &req, GetDepthRegistration::Response &res);
    bool onGetColorImageStreamMode(GetImageStreamMode::Request &req, GetImageStreamMode::Response &res);
    bool onGetDepthImageStreamMode(GetImageStreamMode::Request &req, GetImageStreamMode::Response &res);
    bool onGetIrImageStreamMode(GetImageStreamMode::Request &req, GetImageStreamMode::Response &res);
    bool onGetColorImageStreamModes(GetImageStreamModes::Request &req, GetImageStreamModes::Response &res);
    bool onGetDepthImageStreamModes(GetImageStreamModes::Request &req, GetImageStreamModes::Response &res);
    bool onGetIrImageStreamModes(GetImageStreamModes::Request &req, GetImageStreamModes::Response &res);
    bool onGetIrExposure(GetIrExposure::Request &req, GetIrExposure::Response &res);
    bool onGetIrGain(GetIrGain::Request &req, GetIrGain::Response &res);
    bool onGetColorMirrored(GetMirrored::Request &req, GetMirrored::Response &res);
    bool onGetDepthMirrored(GetMirrored::Request &req, GetMirrored::Response &res);
    bool onGetIrMirrored(GetMirrored::Request &req, GetMirrored::Response &res);
    bool onGetColorRunning(GetRunning::Request &req, GetRunning::Response &res);
    bool onGetDepthRunning(GetRunning::Request &req, GetRunning::Response &res);
    bool onGetIrRunning(GetRunning::Request &req, GetRunning::Response &res);
    bool onGetSerial(GetSerial::Request &req, GetSerial::Response &res);
    bool onGetColorUsbInfo(GetUsbInfo::Request &req, GetUsbInfo::Response &res);
    bool onGetDepthUsbInfo(GetUsbInfo::Request &req, GetUsbInfo::Response &res);
    bool onGetIrUsbInfo(GetUsbInfo::Request &req, GetUsbInfo::Response &res);
    bool onSetDepthRegistration(SetDepthRegistration::Request &req, SetDepthRegistration::Response &res);
    bool onSetColorImageStreamMode(SetImageStreamMode::Request &req, SetImageStreamMode::Response &res);
    bool onSetDepthImageStreamMode(SetImageStreamMode::Request &req, SetImageStreamMode::Response &res);
    bool onSetIrImageStreamMode(SetImageStreamMode::Request &req, SetImageStreamMode::Response &res);
    bool onSetIrExposure(SetIrExposure::Request &req, SetIrExposure::Response &res);
    bool onSetIrGain(SetIrGain::Request &req, SetIrGain::Response &res);
    bool onSetColorMirrored(SetMirrored::Request &req, SetMirrored::Response &res);
    bool onSetDepthMirrored(SetMirrored::Request &req, SetMirrored::Response &res);
    bool onSetIrMirrored(SetMirrored::Request &req, SetMirrored::Response &res);
    bool onSetColorRunning(SetRunning::Request &req, SetRunning::Response &res);
    bool onSetDepthRunning(SetRunning::Request &req, SetRunning::Response &res);
    bool onSetIrRunning(SetRunning::Request &req, SetRunning::Response &res);

    std::string name_;
    ros::NodeHandle &nh_;
    ros::NodeHandle device_nh_;
    ros::NodeHandle &pnh_;

    image_transport::ImageTransport image_transport_;

    typedef dynamic_reconfigure::Server<DeviceConfig> DeviceConfigServer;

    // DeviceConfigServer isn't movable (w.r.t. move semantics), so we wrap it in a unique_ptr
    std::unique_ptr<boost::recursive_mutex> mut_;
    std::unique_ptr<DeviceConfigServer> dynamic_reconfigure_server_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    bool publish_body_markers;
    bool publish_body_mask;
    bool publish_floor_mask;
    bool publish_tf;
    std::string body_frame_id;

    Device::Ptr device_;

    orbbec_camera_params camera_parameters_;

    ros::Publisher color_camera_info_pub_;
    ros::Publisher depth_camera_info_pub_;

    image_transport::Publisher color_image_pub_;
    image_transport::Publisher ir_image_pub_;
    image_transport::Publisher depth_image_pub_;
    image_transport::Publisher body_mask_image_pub_;
    image_transport::Publisher floor_mask_image_pub_;
    image_transport::Publisher colorized_body_image_pub_;
    image_transport::Publisher masked_color_image_pub_;

    ros::Publisher body_frame_pub_;
    ros::Publisher body_markers_pub_;
    ros::Publisher point_cloud_pub_;


    ros::ServiceServer get_chip_id_svc_;
    ros::ServiceServer get_depth_registration_svc_;
    ros::ServiceServer get_color_image_stream_mode_svc_;
    ros::ServiceServer get_depth_image_stream_mode_svc_;
    ros::ServiceServer get_ir_image_stream_mode_svc_;
    ros::ServiceServer get_color_image_stream_modes_svc_;
    ros::ServiceServer get_depth_image_stream_modes_svc_;
    ros::ServiceServer get_ir_image_stream_modes_svc_;
    ros::ServiceServer get_ir_exposure_svc_;
    ros::ServiceServer get_ir_gain_svc_;
    ros::ServiceServer get_color_mirrored_svc_;
    ros::ServiceServer get_depth_mirrored_svc_;
    ros::ServiceServer get_ir_mirrored_svc_;
    ros::ServiceServer get_color_running_svc_;
    ros::ServiceServer get_depth_running_svc_;
    ros::ServiceServer get_ir_running_svc_;
    ros::ServiceServer get_serial_svc_;
    ros::ServiceServer get_color_usb_info_svc_;
    ros::ServiceServer get_depth_usb_info_svc_;
    ros::ServiceServer get_ir_usb_info_svc_;
    ros::ServiceServer set_depth_registration_svc_;
    ros::ServiceServer set_color_image_stream_mode_svc_;
    ros::ServiceServer set_depth_image_stream_mode_svc_;
    ros::ServiceServer set_ir_image_stream_mode_svc_;
    ros::ServiceServer set_ir_exposure_svc_;
    ros::ServiceServer set_ir_gain_svc_;
    ros::ServiceServer set_color_mirrored_svc_;
    ros::ServiceServer set_depth_mirrored_svc_;
    ros::ServiceServer set_ir_mirrored_svc_;
    ros::ServiceServer set_color_running_svc_;
    ros::ServiceServer set_depth_running_svc_;
    ros::ServiceServer set_ir_running_svc_;
  };

  extern const std::string DEVICE_NAMESPACE;
}

#endif