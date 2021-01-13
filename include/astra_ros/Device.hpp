#ifndef _ASTRA_ROS_DEVICE_HPP_
#define _ASTRA_ROS_DEVICE_HPP_

#include <string>
#include <memory>
#include <iostream>
#include <vector>
#include <boost/optional.hpp>

#include "Parameter.hpp"

#include <astra/capi/astra.h>

namespace astra_ros
{
  /**
   * \class Device
   * \brief An agnostic C++ wrapper for the C Astra SDK. This object
   * is wrapped by `RosDevice`.
   */
  class Device
  {
  public:
    typedef std::shared_ptr<Device> Ptr;
    typedef std::shared_ptr<const Device> ConstPtr;

    /**
     * \struct ImageStreamMode
     * \brief Represents a potential stream configuration
     */
    struct ImageStreamMode
    {
      /**
       * The width, in pixels, of the image mode. `boost::none` means any width.
       */
      boost::optional<std::uint32_t> width;

      /**
       * The height, in pixels, of the image mode. `boost::none` means any height.
       */
      boost::optional<std::uint32_t> height;

      /**
       * The pixel format of the image mode. `boost::none` means any pixel format.
       */
      boost::optional<astra_pixel_format_t> pixel_format;

      /**
       * The refresh rate, in Hz, of the image mode. `boost::none` means any refresh rate.
       */
      boost::optional<std::uint8_t> fps;
    };

    struct Frame
    {
      struct Color
      {
        std::uint32_t data_length;

        // The image data for the color stream (use metadata to understand the content).
        // This pointer is only valid until the next update().
        std::uint8_t *data;

        astra_image_metadata_t metadata;
      };

      struct Ir
      {
        float horizontal_fov;
        float vertical_fov;

        std::uint32_t data_length;

        // The image data for the IR stream (use metadata to understand the content).
        // This pointer is only valid until the next update().
        std::uint8_t *data;

        astra_image_metadata_t metadata;
      };

      struct Depth
      {
        float horizontal_fov;
        float vertical_fov;

        bool registered;

        std::uint32_t data_length;

        // The image data for the depth stream (use metadata to understand the content).
        // This pointer is only valid until the next update().
        std::int16_t *data;

        astra_image_metadata_t metadata;
      };

      struct Body
      {
        astra_floor_info_t *floor_info;
        astra_bodymask_t body_mask;
        astra_body_list_t body_list;
      };

      struct ColorizedBody
      {
        std::uint32_t data_length;

        // The pixel data for the colorized body stream.
        // This pointer is only valid until the next update().
        astra_rgba_pixel_t *data;

        astra_image_metadata_t metadata;
      };

      struct Hand
      {

      };

      struct MaskedColor
      {
        std::uint32_t data_length;

        // The pixel data for the masked color stream.
        // This pointer is only valid until the next update().
        astra_rgba_pixel_t *data;

        astra_image_metadata_t metadata;
      };

      struct Point
      {
        std::uint32_t data_length;
        astra_vector3f_t *data;
      };

      /**
       * Color data. `boost::none` if `color_stream` was `boost::none` in the `Configuration`.
       */
      boost::optional<Color> color;
      
      /**
       * IR data. `boost::none` if `ir_stream` was `boost::none` in the `Configuration`.
       */
      boost::optional<Ir> ir;

      /**
       * Depth data. `boost::none` if `depth_stream` was `boost::none` in the `Configuration`.
       */
      boost::optional<Depth> depth;

      /**
       * Body data. `boost::none` if `body_stream` was `boost::none` in the `Configuration`.
       */
      boost::optional<Body> body;

      /**
       * Colorized body data. `boost::none` if `colorized_body_stream` was `boost::none` in the `Configuration`.
       */
      boost::optional<ColorizedBody> colorized_body;

      /**
       * Hand data. `boost::none` if `hand_stream` was `boost::none` in the `Configuration`.
       */
      boost::optional<Hand> hand;

      /**
       * Masked color data. `boost::none` if `masked_color_stream` was `boost::none` in the `Configuration`.
       */
      boost::optional<MaskedColor> masked_color;

      /**
       * Point cloud data. `boost::none` if `point_stream` was `boost::none` in the `Configuration`.
       */
      boost::optional<Point> point;
    };

    struct Configuration
    {
      typedef std::function<void (const Frame &frame)> OnFrame;

      /**
       * \struct ColorStream
       * 
       * Holds configuration options for the Astra color stream
       */
      struct ColorStream
      {
        typedef Parameter<bool> Running;
        typedef Parameter<bool> Mirrored;

        typedef Parameter<ImageStreamMode> Mode;

        Running running;

        /**
         * Whether the color image should be horizontally mirrored. If `boost::none`, the image will not be mirrored.
         */
        boost::optional<Mirrored> mirrored;

        /**
         * The `ImageStreamMode` of the color stream. If `boost::none`, the default mode will be used.
         */
        boost::optional<Mode> mode;
      };

      /**
       * \struct IrStream
       * 
       * Holds configuration options for the Astra IR stream
       */
      struct IrStream
      {
        typedef Parameter<bool> Running;
        typedef Parameter<bool> Mirrored;

        typedef Parameter<ImageStreamMode> Mode;

        typedef Parameter<std::int32_t> Gain;
        typedef Parameter<std::int32_t> Exposure;

        Running running;

        /**
         * Whether the IR image should be horizontally mirrored. If `boost::none`, the image will not be mirrored.
         */
        boost::optional<Mirrored> mirrored;

        /**
         * The `ImageStreamMode` of the IR stream. If `boost::none`, the default mode will be used.
         */
        boost::optional<Mode> mode;

        boost::optional<Gain> gain;
        boost::optional<Exposure> exposure;
        
      };

      struct DepthStream
      {
        typedef Parameter<bool> Running;
        typedef Parameter<bool> Mirrored;

        typedef Parameter<ImageStreamMode> Mode;

        typedef Parameter<bool> Registration;
        typedef Parameter<int> D2CMode;

        Running running;

        /**
         * Whether the depth image should be horizontally mirrored. If `boost::none`, the image will not be mirrored.
         */
        boost::optional<Mirrored> mirrored;

        /**
         * The `ImageStreamMode` of the depth stream. If `boost::none`, the default mode will be used.
         */
        boost::optional<Mode> mode;

        /**
         * If true, the depth image will be registered. If false or `boost::none`, the depth image will not be registered.
         */
        boost::optional<Registration> registration;

        
        boost::optional<D2CMode> d2c_mode;
        
      };

      struct BodyStream
      {
        typedef Parameter<bool> Running;
        typedef Parameter<astra_body_orientation_t> BodyOrientation;
        typedef Parameter<astra_skeleton_optimization_t> SkeletonOptimization;
        typedef Parameter<astra_skeleton_profile_t> SkeletonProfile;
        typedef Parameter<astra_body_tracking_feature_flags_t> BodyFeatures;

        /**
         * \property license
         * 
         * The Orbbec Body Tracking SDK license key. Depending on the Astra SDK used, this may be required.
         */
        boost::optional<std::string> license;

        Running running;

        boost::optional<BodyOrientation> body_orientation;
        boost::optional<SkeletonOptimization> skeleton_optimization;
        boost::optional<SkeletonProfile> skeleton_profile;
        boost::optional<BodyFeatures> default_body_features;
      };

      struct ColorizedBodyStream
      {
        typedef Parameter<bool> Running;
        typedef Parameter<bool> Mirrored;

        Running running;
      };

      struct HandStream
      {
        typedef Parameter<bool> Running;
        Running running;
      };

      struct MaskedColorStream
      {
        typedef Parameter<bool> Running;
        typedef Parameter<bool> Mirrored;

        Running running;
      };

      struct PointStream
      {
        typedef Parameter<bool> Running;
        Running running;
      };

      boost::optional<std::string> uri;
      OnFrame on_frame;


      /**
       * Configure the `Device`'s color stream. `boost::none` means no color stream will be
       * provided in the `Frame`.
       */
      boost::optional<ColorStream> color_stream;
      
      /**
       * Configure the `Device`'s IR stream. `boost::none` means no IR stream will be
       * provided in the `Frame`.
       */
      boost::optional<IrStream> ir_stream;

      /**
       * Configure the `Device`'s depth stream. `boost::none` means no depth stream will be
       * provided in the `Frame`.
       */
      boost::optional<DepthStream> depth_stream;
      
      /**
       * Configure the `Device`'s body stream. `boost::none` means no body stream will be
       * provided in the `Frame`.
       */
      boost::optional<BodyStream> body_stream;
      
      /**
       * \property colorized_body_stream
       * 
       * Configure the `Device`'s colorized body stream. `boost::none` means no colorized body stream will be
       * provided in the `Frame`.
       */
      boost::optional<ColorizedBodyStream> colorized_body_stream;
      
      /**
       * \property hand_stream
       * 
       * Configure the `Device`'s hand stream. `boost::none` means no hand stream will be
       * provided in the `Frame`.
       */
      boost::optional<HandStream> hand_stream;
      
      /**
       * \property masked_color_stream
       * 
       * Configure the `Device`'s masked color stream. `boost::none` means no masked color stream will be
       * provided in the `Frame`.
       */
      boost::optional<MaskedColorStream> masked_color_stream;
      
      /**
       * \property point_stream
       * 
       * Configure the `Device`'s point cloud stream. `boost::none` means no masked point cloud stream will be
       * provided in the `Frame`.
       */
      boost::optional<PointStream> point_stream;
    };

    /**
     * \fn open
     * Open an Orbbec device
     * 
     * \param configuration The `Configuration` to use for this device.
     */
    static Ptr open(const Configuration &configuration);

    /**
     * \fn update
     * If a new frame is available, process it and call the `OnFrame` handler as specified in the `Configuration`.
     */
    void update();

    /**
     * \fn getConfiguration
     * \return A constant reference to the configuration.
     */
    inline const Configuration &getConfiguration() const noexcept
    {
      return configuration_;
    }

    /**
     * \fn getConfiguration
     * \return A mutable reference to the configuration. Can be used to update parameters.
     */
    inline Configuration &getConfiguration() noexcept
    {
      return configuration_;
    }

    /**
     * \fn getCameraParameters
     * \return The color and depth camera intrinsics
     */
    orbbec_camera_params getCameraParameters() const;

    /**
     * \fn getSerialNumber 
     * \return The Orbbec device's serial number, or `boost::none` if the `Device` has no depth stream.
     */
    boost::optional<std::string> getSerialNumber() const;

    /**
     * \fn getChipId
     * \return The Orbbec device's Chip ID, or `boost::none` if the `Device` has no depth stream.
     */
    boost::optional<std::uint32_t> getChipId() const;

    /**
     * \fn getColorUsbInfo
     * \return The USB information for the depth camera, or `boost::none` if the `Device` has no color stream.
     */
    boost::optional<astra_usb_info_t> getColorUsbInfo() const;
    
    /**
     * \fn getDepthUsbInfo
     * \return The USB information for the depth camera, or `boost::none` if the `Device` has no depth stream.
     */
    boost::optional<astra_usb_info_t> getDepthUsbInfo() const;

    /**
     * \fn getIrUsbInfo
     * \return The USB information for the IR camera, or `boost::none` if the `Device` has no IR stream.
     */
    boost::optional<astra_usb_info_t> getIrUsbInfo() const;

    /**
     * \fn getColorImageStream
     * Get the image modes supported by the color stream.
     * 
     * \return boost::none if the device wasn't configured with a color stream, a list of image modes otherwise.
     */
    boost::optional<std::vector<ImageStreamMode>> getColorImageStreamModes() const;
    
    /**
     * Get the image modes supported by the depth stream.
     * 
     * \return boost::none if the device wasn't configured with a depth stream, a list of image modes otherwise.
     */
    boost::optional<std::vector<ImageStreamMode>> getDepthImageStreamModes() const;

    /**
     * Get the image modes supported by the IR stream.
     * 
     * \return boost::none if the device isn't configured with a IR stream, a list of image modes otherwise.
     */
    boost::optional<std::vector<ImageStreamMode>> getIrImageStreamModes() const;

  private:
    Device(const Configuration &configuration);

    bool onStreamStartedChange(astra_streamconnection_t stream, const bool &started, const bool &started_prev);
    bool onDepthRegistrationChange(astra_depthstream_t stream, const bool &registration, const bool &prev_registration);
    bool onDepthD2CModeChange(astra_depthstream_t stream, const int &d2c_mode, const bool &prev_d2c_mode);
    bool onBodyOrientationChange(astra_bodystream_t body_stream, const astra_body_orientation_t &orientation, const astra_skeleton_optimization_t &prev_orientation);
    bool onBodySkeletonOptimizationChange(astra_bodystream_t body_stream, const astra_skeleton_optimization_t &optimization, const astra_skeleton_optimization_t &prev_optimization);
    bool onBodySkeletonProfileChange(astra_bodystream_t body_stream, const astra_skeleton_profile_t &profile, const astra_skeleton_profile_t &prev_profile);
    bool onModeChange(astra_streamconnection_t stream, const ImageStreamMode &mode, const ImageStreamMode &prev_mode);
    bool onMirroredChange(astra_streamconnection_t stream, const bool &mirrored, const bool &prev_mirrored);

    std::vector<ImageStreamMode> getImageStreamModes(astra_streamconnection_t stream) const;

    Configuration configuration_;

    astra_streamsetconnection_t stream_set_;
    astra_reader_t reader_;

    boost::optional<astra_colorstream_t> color_stream_;
    boost::optional<astra_infraredstream_t> ir_stream_;
    boost::optional<astra_depthstream_t> depth_stream_;
    boost::optional<astra_bodystream_t> body_stream_;
    boost::optional<astra_colorizedbodystream_t> colorized_body_stream_;
    boost::optional<astra_handstream_t> hand_stream_;
    boost::optional<astra_maskedcolorstream_t> masked_color_stream_;
    boost::optional<astra_pointstream_t> point_stream_;
  };
}

std::ostream &operator <<(std::ostream &o, const astra_ros::Device::Configuration::ColorStream &value);
std::ostream &operator <<(std::ostream &o, const astra_ros::Device::Configuration::IrStream &value);
std::ostream &operator <<(std::ostream &o, const astra_ros::Device::Configuration::DepthStream &value);
std::ostream &operator <<(std::ostream &o, const astra_ros::Device::Configuration::BodyStream &value);
std::ostream &operator <<(std::ostream &o, const astra_ros::Device::Configuration::ColorizedBodyStream &value);
std::ostream &operator <<(std::ostream &o, const astra_ros::Device::Configuration::MaskedColorStream &value);
std::ostream &operator <<(std::ostream &o, const astra_ros::Device::Configuration &value);

#endif