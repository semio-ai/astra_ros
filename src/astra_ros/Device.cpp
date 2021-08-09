#include "astra_ros/Device.hpp"
#include "astra_ros/Exception.hpp"

#include "util.hpp"


#include <iostream>
#include <sstream>

namespace
{
  const static char *const DEFAULT_URI = "device/default";

  // Wrapper for astra_*_is_available functions
  // to make it slightly more convenient to use.
  template<typename F, typename T>
  bool is_available(F &&f, const T &stream)
  {
    bool is_available = false;
    return f(stream, &is_available) == ASTRA_STATUS_SUCCESS && is_available;
  }

  bool initialized = false;

  static void initialize()
  {
    if (initialized) return;
    astra_initialize();
    initialized = false;
  }
}

using namespace astra_ros;

Device::Ptr Device::open(const Configuration &configuration)
{
  return std::shared_ptr<Device>(new Device(configuration));
}

void Device::update()
{
  // Try to fetch a new frame (this invalidates all existing frame pointers)
  astra_status_t status = astra_update();
  if (status != ASTRA_STATUS_SUCCESS) throw Exception(status);

  bool new_frame = false;
  astra_reader_has_new_frame(reader_, &new_frame);
  if (status != ASTRA_STATUS_SUCCESS) throw Exception(status);

  // If we don't have a new frame to process, we're done
  if (!new_frame) return;


  
  astra_reader_frame_t astra_frame;
  
  status = astra_reader_open_frame(reader_, 3, &astra_frame);
  if (status != ASTRA_STATUS_SUCCESS) throw Exception(status);

  Frame frame;

  // Color Stream
  if (color_stream_ && is_available(&astra_colorstream_is_available, *color_stream_))
  {
    astra_colorframe_t color_frame;
    status = astra_frame_get_colorframe(astra_frame, &color_frame);
    if (status != ASTRA_STATUS_SUCCESS) throw Exception(status);

    Frame::Color color;
    status = astra_colorframe_get_data_ptr(color_frame, &color.data, &color.data_length);
    if (status != ASTRA_STATUS_SUCCESS) throw Exception(status);

    astra_image_metadata_t metadata;
    status = astra_colorframe_get_metadata(color_frame, &metadata);
    if (status != ASTRA_STATUS_SUCCESS) throw Exception(status);

    color.metadata = metadata;

    frame.color = color;
  }

  // IR Stream
  if (ir_stream_ && is_available(&astra_infraredstream_is_available, *ir_stream_))
  {
    astra_colorframe_t ir_frame;
    status = astra_frame_get_infraredframe(astra_frame, &ir_frame);
    if (status != ASTRA_STATUS_SUCCESS) throw Exception(status);

    Frame::Ir ir;
    status = astra_infraredframe_get_data_ptr(ir_frame, &ir.data, &ir.data_length);
    if (status != ASTRA_STATUS_SUCCESS) throw Exception(status);

    astra_image_metadata_t metadata;
    status = astra_infraredframe_get_metadata(ir_frame, &metadata);
    if (status != ASTRA_STATUS_SUCCESS) throw Exception(status);

    ir.metadata = metadata;

    frame.ir = ir;
  }

  // Depth Stream
  if (depth_stream_ && is_available(&astra_depthstream_is_available, *depth_stream_))
  {
    astra_depthframe_t depth_frame;
    status = astra_frame_get_depthframe(astra_frame, &depth_frame);
    if (status != ASTRA_STATUS_SUCCESS) throw Exception(status);
  
    Frame::Depth depth;
    status = astra_depthframe_get_data_ptr(depth_frame, &depth.data, &depth.data_length);
    if (status != ASTRA_STATUS_SUCCESS) throw Exception(status);

    astra_image_metadata_t metadata;
    status = astra_depthframe_get_metadata(depth_frame, &metadata);
    if (status != ASTRA_STATUS_SUCCESS) throw Exception(status);

    depth.metadata = metadata;
    
    status = astra_depthstream_get_hfov(*depth_stream_, &depth.horizontal_fov);
    if (status != ASTRA_STATUS_SUCCESS) throw Exception(status);

    status = astra_depthstream_get_vfov(*depth_stream_, &depth.vertical_fov);
    if (status != ASTRA_STATUS_SUCCESS) throw Exception(status);

    frame.depth = depth;
  }

  // Body Stream
  if (body_stream_)
  {
    astra_bodyframe_t body_frame;
    status = astra_frame_get_bodyframe(astra_frame, &body_frame);
    if (status != ASTRA_STATUS_SUCCESS) throw Exception(status);

    Frame::Body body;
    status = astra_bodyframe_body_list(body_frame, &body.body_list);
    if (status != ASTRA_STATUS_SUCCESS) throw Exception(status);

    status = astra_bodyframe_bodymask(body_frame, &body.body_mask);
    if (status != ASTRA_STATUS_SUCCESS) throw Exception(status);

    status = astra_bodyframe_floor_info_ptr(body_frame, &body.floor_info);
    if (status != ASTRA_STATUS_SUCCESS) throw Exception(status);
    
    frame.body = body;
  }

  // Colorized Body Stream
  if (colorized_body_stream_ && is_available(&astra_colorizedbodystream_is_available, *colorized_body_stream_))
  {
    astra_colorizedbodyframe_t colorized_body_frame;
    status = astra_frame_get_colorizedbodyframe(astra_frame, &colorized_body_frame);
    if (status != ASTRA_STATUS_SUCCESS) throw Exception(status);

    Frame::ColorizedBody colorized_body;
    status = astra_colorizedbodyframe_get_data_ptr(colorized_body_frame, &colorized_body.data, &colorized_body.data_length);
    if (status != ASTRA_STATUS_SUCCESS) throw Exception(status);

    astra_image_metadata_t metadata;
    status = astra_colorizedbodyframe_get_metadata(colorized_body_frame, &metadata);
    if (status != ASTRA_STATUS_SUCCESS) throw Exception(status);

    colorized_body.metadata = metadata;

    frame.colorized_body = colorized_body;
  }

  // Hand Stream
  if (hand_stream_ && is_available(&astra_handstream_is_available, *hand_stream_))
  {
    astra_handframe_t hand_frame;
    status = astra_frame_get_handframe(astra_frame, &hand_frame);
    if (status != ASTRA_STATUS_SUCCESS) throw Exception(status);

    Frame::Hand hand;
    frame.hand = hand;
  }

  // Masked Color Stream
  if (masked_color_stream_ && is_available(&astra_maskedcolorstream_is_available, *masked_color_stream_))
  {
    astra_maskedcolorframe_t masked_color_frame;
    status = astra_frame_get_maskedcolorframe(astra_frame, &masked_color_frame);
    if (status != ASTRA_STATUS_SUCCESS) throw Exception(status);

    Frame::MaskedColor masked_color;
    status = astra_maskedcolorframe_get_data_ptr(masked_color_frame, &masked_color.data, &masked_color.data_length);
    if (status != ASTRA_STATUS_SUCCESS) throw Exception(status);

    astra_image_metadata_t metadata;
    status = astra_maskedcolorframe_get_metadata(masked_color_frame, &metadata);
    if (status != ASTRA_STATUS_SUCCESS) throw Exception(status);
    masked_color.metadata = metadata;

    frame.masked_color = masked_color;
  }

  // Point Stream
  if (point_stream_ && is_available(&astra_pointstream_is_available, *point_stream_))
  {
    astra_pointframe_t point_frame;
    status = astra_frame_get_pointframe(astra_frame, &point_frame);
    if (status != ASTRA_STATUS_SUCCESS) throw Exception(status);

    Frame::Point point;
    status = astra_pointframe_get_data_ptr(point_frame, &point.data, &point.data_length);
    if (status != ASTRA_STATUS_SUCCESS) throw Exception(status);
    frame.point = point;
  }

  configuration_.on_frame(frame);

  status = astra_reader_close_frame(&astra_frame);
  if (status != ASTRA_STATUS_SUCCESS) throw Exception(status);
}

Device::Device(const Configuration &configuration)
  : configuration_(configuration)
{
  const char *const uri = configuration_.uri ? configuration_.uri->c_str() : DEFAULT_URI;

  
  

  initialize();

  // The license must be registered after initialize()
  if (configuration_.body_stream)
  {
    const auto &license = configuration_.body_stream->license;
    if (license)
    {
      std::cout << "Configured license" << std::endl;
      orbbec_body_tracking_set_license(license->c_str());
    }
  }
  
  astra_status_t status = astra_streamset_open(uri, &stream_set_);
  if (status != ASTRA_STATUS_SUCCESS) throw Exception(status);

  status = astra_reader_create(stream_set_, &reader_);
  if (status != ASTRA_STATUS_SUCCESS) throw Exception(status);

  // This function is used to check the return values of all astra functions
  // below. If there is an error, we perform appropriate cleanup before throwing.
  const auto check_status = [this](const astra_status_t status) {
    if (status == ASTRA_STATUS_SUCCESS) return;

    color_stream_ = boost::none;
    ir_stream_ = boost::none;
    depth_stream_ = boost::none;
    body_stream_ = boost::none;
    colorized_body_stream_ = boost::none;

    astra_reader_destroy(&reader_);
    astra_streamset_close(&stream_set_);

    throw Exception(status);
  };

  // Color Stream
  if (configuration_.color_stream)
  {
    astra_colorstream_t stream;
    check_status(astra_reader_get_colorstream(reader_, &stream));
    color_stream_ = stream;

    // Configure the mode parameter
    auto &mode = configuration_.color_stream->mode;
    if (mode)
    {
      mode->bindOnChangeHandler(&Device::onModeChange, this, stream);
      Device::onModeChange(stream, **mode, ImageStreamMode());
    }

    // Configure the mirrored parameter
    auto &mirrored = configuration_.color_stream->mirrored;
    if (mirrored)
    {
      mirrored->bindOnChangeHandler(&Device::onMirroredChange, this, stream);
      astra_imagestream_set_mirroring(stream, **mirrored);
    }

    // Configure the running parameter
    auto &running = configuration_.color_stream->running;
    running.bindOnChangeHandler(&Device::onStreamStartedChange, this, stream);
    if (*running)
    {
      check_status(astra_stream_start(stream));
    }
  }

  // IR Stream
  if (configuration_.ir_stream)
  {
    astra_infraredstream_t stream;
    check_status(astra_reader_get_infraredstream(reader_, &stream));
    ir_stream_ = stream;

    // Configure the mode parameter
    auto &mode = configuration_.ir_stream->mode;
    if (mode)
    {
      mode->bindOnChangeHandler(&Device::onModeChange, this, stream);
      Device::onModeChange(stream, **mode, ImageStreamMode());
    }

    // Configure the mirrored parameter
    auto &mirrored = configuration_.ir_stream->mirrored;
    if (mirrored)
    {
      mirrored->bindOnChangeHandler(&Device::onMirroredChange, this, stream);
      astra_imagestream_set_mirroring(stream, **mirrored);
    }

    // Configure the running parameter
    auto &running = configuration_.ir_stream->running;
    running.bindOnChangeHandler(&Device::onStreamStartedChange, this, stream);
    if (*running)
    {
      check_status(astra_stream_start(stream));
    }
  }

  // Depth Stream
  if (configuration_.depth_stream)
  {
    astra_depthstream_t stream;
    check_status(astra_reader_get_depthstream(reader_, &stream));
    depth_stream_ = stream;

    // Configure the mode parameter
    auto &mode = configuration_.depth_stream->mode;
    if (mode)
    {
      mode->bindOnChangeHandler(&Device::onModeChange, this, stream);
      Device::onModeChange(stream, **mode, ImageStreamMode());
    }

    // Configure the mirrored parameter
    auto &mirrored = configuration_.depth_stream->mirrored;
    if (mirrored)
    {
      mirrored->bindOnChangeHandler(&Device::onMirroredChange, this, stream);
      astra_imagestream_set_mirroring(stream, **mirrored);
    }

    // Configure the d2c mode parameter
    auto &d2c_mode = configuration_.depth_stream->d2c_mode;
    if (d2c_mode)
    {
      d2c_mode->bindOnChangeHandler(&Device::onDepthD2CModeChange, this, stream);
      astra_depthstream_set_d2c_resolution(stream, **d2c_mode);
    }

    // Configure the registration parameter
    auto &registration = configuration_.depth_stream->registration;
    if (registration)
    {
      registration->bindOnChangeHandler(&Device::onDepthRegistrationChange, this, stream);
      astra_depthstream_set_registration(stream, **registration);
    }

    // Configure the running parameter
    auto &running = configuration_.depth_stream->running;
    running.bindOnChangeHandler(&Device::onStreamStartedChange, this, stream);
    if (*running)
    {
      check_status(astra_stream_start(stream));
    }
  }

  // Body Stream
  if (configuration_.body_stream)
  {
    // Open the stream
    astra_bodystream_t stream;
    check_status(astra_reader_get_bodystream(reader_, &stream));
    body_stream_ = stream;

    // Configure body orientation parameter
    /*auto &body_orientation = configuration_.body_stream->body_orientation;
    if (body_orientation)
    {
      body_orientation->bindOnChangeHandler(&Device::onBodyOrientationChange, this, stream);
      check_status(astra_bodystream_set_body_orientation(stream, **body_orientation));
    }

    // Configure skeleton optimization parameter
    auto &skeleton_optimization = configuration_.body_stream->skeleton_optimization;
    if (skeleton_optimization)
    {
      skeleton_optimization->bindOnChangeHandler(&Device::onBodySkeletonOptimizationChange, this, stream);
      check_status(astra_bodystream_set_skeleton_optimization(stream, **skeleton_optimization));
    }

    // Configure skeleton profile parameter
    auto &skeleton_profile = configuration_.body_stream->skeleton_profile;
    if (skeleton_profile)
    {
      skeleton_profile->bindOnChangeHandler(&Device::onBodySkeletonOptimizationChange, this, stream);
      check_status(astra_bodystream_set_skeleton_optimization(stream, **skeleton_profile));
    }*/

    

    astra_bodystream_set_skeleton_profile(stream, ASTRA_SKELETON_PROFILE_FULL);
    astra_bodystream_set_default_body_features(stream, ASTRA_BODY_TRACKING_SEGMENTATION | ASTRA_BODY_TRACKING_JOINTS | ASTRA_BODY_TRACKING_HAND_POSES);

    // Configure the running parameter
    auto &running = configuration_.body_stream->running;
    running.bindOnChangeHandler(&Device::onStreamStartedChange, this, stream);
    if (*running)
    {
      check_status(astra_stream_start(stream));
    }
  }

  // Colorized Body Stream
  if (configuration_.colorized_body_stream)
  {
    astra_colorizedbodystream_t stream;
    check_status(astra_reader_get_colorizedbodystream(reader_, &stream));
    colorized_body_stream_ = stream;

    // Configure the running parameter
    auto &running = configuration_.colorized_body_stream->running;
    running.bindOnChangeHandler(&Device::onStreamStartedChange, this, stream);
    if (*running)
    {
      check_status(astra_stream_start(stream));
    }
  }

  // Hand Stream
  if (configuration_.hand_stream)
  {
    astra_handstream_t stream;
    check_status(astra_reader_get_handstream(reader_, &stream));
    hand_stream_ = stream;
    
    // Configure the running parameter
    auto &running = configuration_.hand_stream->running;
    running.bindOnChangeHandler(&Device::onStreamStartedChange, this, stream);
    if (*running)
    {
      check_status(astra_stream_start(stream));
    }

    
  }

  // Point Stream
  if (configuration_.point_stream)
  {
    astra_pointstream_t stream;
    check_status(astra_reader_get_pointstream(reader_, &stream));
    point_stream_ = stream;

    // Configure the running parameter
    auto &running = configuration_.point_stream->running;
    running.bindOnChangeHandler(&Device::onStreamStartedChange, this, stream);
    if (*running)
    {
      check_status(astra_stream_start(stream));
    }
  }
}

bool Device::onStreamStartedChange(astra_streamconnection_t stream, const bool &started, const bool &started_prev)
{
  // Detect how the state changed
  const bool should_start = started && !started_prev;
  const bool should_stop = !started && started_prev;

  if (should_start)
  {
    const astra_status_t status = astra_stream_start(stream);
    if (status != ASTRA_STATUS_SUCCESS)
    {
      std::cerr << "astra_stream_start failed (status: " << statusToString(status) << ")" << std::endl;
    }
    return status == ASTRA_STATUS_SUCCESS;
  }

  if (should_stop)
  {
    const astra_status_t status = astra_stream_stop(stream);
    if (status != ASTRA_STATUS_SUCCESS)
    {
      std::cerr << "astra_stream_stop failed (status: " << statusToString(status) << ")" << std::endl;
    }
    return status == ASTRA_STATUS_SUCCESS;
  }

  // We should never reach this point
  return false;
}

bool Device::onDepthRegistrationChange(astra_depthstream_t stream, const bool &registration, const bool &prev_registration)
{
  const astra_status_t status = astra_depthstream_set_registration(stream, registration);
  if (status != ASTRA_STATUS_SUCCESS)
  {
    std::cerr << "astra_depthstream_set_registration failed (status: " << statusToString(status) << ")" << std::endl;
  }

  return status == ASTRA_STATUS_SUCCESS;
}

bool Device::onDepthD2CModeChange(astra_depthstream_t stream, const int &d2c_mode, const bool &prev_d2c_mode)
{
  const astra_status_t status = astra_depthstream_set_d2c_resolution(stream, d2c_mode);
  if (status != ASTRA_STATUS_SUCCESS)
  {
    std::cerr << "astra_depthstream_set_d2c_resolution failed (status: " << statusToString(status) << ")" << std::endl;
  }

  return status == ASTRA_STATUS_SUCCESS;
}


bool Device::onBodySkeletonOptimizationChange(astra_bodystream_t body_stream, const astra_skeleton_optimization_t &optimization, const astra_skeleton_optimization_t &prev_optimization)
{
  const astra_status_t status = astra_bodystream_set_skeleton_optimization(body_stream, optimization);
  if (status != ASTRA_STATUS_SUCCESS)
  {
    std::cerr << "astra_bodystream_set_skeleton_optimization failed (status: " << statusToString(status) << ")" << std::endl;
  }

  return status == ASTRA_STATUS_SUCCESS;
}

bool Device::onBodyOrientationChange(astra_bodystream_t body_stream, const astra_body_orientation_t &orientation, const astra_body_orientation_t &prev_orientation)
{
  const astra_status_t status = astra_bodystream_set_body_orientation(body_stream, orientation);
  if (status != ASTRA_STATUS_SUCCESS)
  {
    std::cerr << "astra_bodystream_set_body_orientation failed (status: " << statusToString(status) << ")" << std::endl;
  }

  return status == ASTRA_STATUS_SUCCESS;
}

bool Device::onBodySkeletonProfileChange(astra_bodystream_t body_stream, const astra_skeleton_profile_t &profile, const astra_skeleton_profile_t &prev_profile)
{
  const astra_status_t status = astra_bodystream_set_skeleton_profile(body_stream, profile);
  if (status != ASTRA_STATUS_SUCCESS)
  {
    std::cerr << "astra_bodystream_set_skeleton_profile failed (status: " << statusToString(status) << ")" << std::endl;
  }

  return status == ASTRA_STATUS_SUCCESS;
}

bool Device::onMirroredChange(astra_streamconnection_t stream, const bool &mirrored, const bool &prev_mirrored)
{
  const astra_status_t status = astra_imagestream_set_mirroring(stream, mirrored);
  if (status != ASTRA_STATUS_SUCCESS)
  {
    std::cerr << "astra_imagestream_set_mirroring failed (status: " << statusToString(status) << ")" << std::endl;
  }

  return status == ASTRA_STATUS_SUCCESS;
}


bool Device::onModeChange(astra_streamconnection_t stream, const ImageStreamMode &mode, const ImageStreamMode &prev_mode)
{
  astra_result_token_t token;
  std::uint32_t count = 0;
  astra_status_t status = astra_imagestream_request_modes(stream, &token, &count);
  if (status != ASTRA_STATUS_SUCCESS)
  {
    std::cerr << "astra_imagestream_request_modes failed (status: " << statusToString(status) << ")" << std::endl;
    return false;
  }

  astra_imagestream_mode_t *const modes = new astra_imagestream_mode_t[count];
  status = astra_imagestream_get_modes_result(stream, token, modes, count);
  if (status != ASTRA_STATUS_SUCCESS)
  {
    std::cerr << "astra_imagestream_get_modes_result failed (status: " << statusToString(status) << ")" << std::endl;
    delete[] modes;
    return false;
  }

  const astra_imagestream_mode_t *match = nullptr;
  for (std::uint32_t i = 0; i < count; ++i)
  {
    if (
      (!mode.fps || modes[i].fps == *mode.fps) &&
      (!mode.width || modes[i].width == *mode.width) &&
      (!mode.height || modes[i].height == *mode.height) &&
      (!mode.pixel_format || modes[i].pixelFormat == *mode.pixel_format)
    )
    {
      match = &modes[i];
      break;
    }
  }

  if (!match)
  {
    // TODO: Maybe list compatible modes here?
    std::cerr << "Failed to find compatible mode for stream" << std::endl;
    delete[] modes;
    return false;
  }

  astra_imagestream_set_mode(stream, match);

  delete[] modes;
  return true;
}

orbbec_camera_params Device::getCameraParameters() const
{
  orbbec_camera_params ret;
  const astra_status_t status = astra_get_orbbec_camera_params(stream_set_, &ret);
  if (status != ASTRA_STATUS_SUCCESS) throw Exception(status);
  return ret;
}


boost::optional<std::string> Device::getSerialNumber() const
{
  if (!depth_stream_) return boost::none;

  // FIXME: What is the correct length here?
  char buffer[32];
  const astra_status_t status = astra_depthstream_get_serialnumber(*depth_stream_, buffer, sizeof(buffer));
  if (status != ASTRA_STATUS_SUCCESS) throw Exception(status);
  
  return std::string(buffer, buffer + sizeof(buffer));
}

boost::optional<std::uint32_t> Device::getChipId() const
{
  if (!depth_stream_) return boost::none;

  std::uint32_t chip_id;
  const astra_status_t status = astra_depthstream_get_chip_id(*depth_stream_, &chip_id);
  if (status != ASTRA_STATUS_SUCCESS) throw Exception(status);
  
  return chip_id;
}

boost::optional<astra_usb_info_t> Device::getColorUsbInfo() const
{
  if (!color_stream_) return boost::none;

  astra_usb_info_t ret;
  const astra_status_t status = astra_colorstream_get_usb_info(*color_stream_, &ret);
  if (status != ASTRA_STATUS_SUCCESS) throw Exception(status);
  return ret;
}

boost::optional<astra_usb_info_t> Device::getDepthUsbInfo() const
{
  if (!depth_stream_) return boost::none;

  astra_usb_info_t ret;
  const astra_status_t status = astra_depthstream_get_usb_info(*depth_stream_, &ret);
  if (status != ASTRA_STATUS_SUCCESS) throw Exception(status);
  return ret;
}

boost::optional<astra_usb_info_t> Device::getIrUsbInfo() const
{
  if (!ir_stream_) return boost::none;

  astra_usb_info_t ret;
  const astra_status_t status = astra_infraredstream_get_usb_info(*ir_stream_, &ret);
  if (status != ASTRA_STATUS_SUCCESS) throw Exception(status);
  return ret;
}

boost::optional<std::vector<Device::ImageStreamMode>> Device::getColorImageStreamModes() const
{
  if (!color_stream_) return boost::none;
  return getImageStreamModes(*color_stream_);
}

boost::optional<std::vector<Device::ImageStreamMode>> Device::getDepthImageStreamModes() const
{
  if (!depth_stream_) return boost::none;
  return getImageStreamModes(*depth_stream_);
}

boost::optional<std::vector<Device::ImageStreamMode>> Device::getIrImageStreamModes() const
{
  if (!ir_stream_) return boost::none;
  return getImageStreamModes(*ir_stream_);
}

std::vector<Device::ImageStreamMode> Device::getImageStreamModes(astra_streamconnection_t stream) const
{
  astra_result_token_t token;
  std::uint32_t count = 0;
  astra_status_t status = astra_imagestream_request_modes(stream, &token, &count);
  if (status != ASTRA_STATUS_SUCCESS) throw Exception(status);

  astra_imagestream_mode_t *const modes = new astra_imagestream_mode_t[count];
  status = astra_imagestream_get_modes_result(stream, token, modes, count);
  if (status != ASTRA_STATUS_SUCCESS)
  {
    delete[] modes;
    throw Exception(status);
  }

  std::vector<ImageStreamMode> ret;
  ret.reserve(count);
  for (std::uint32_t i = 0; i < count; ++i)
  {
    const auto &mode = modes[i];
    ret.push_back({
      .width = mode.width,
      .height = mode.height,
      .pixel_format = mode.pixelFormat,
      .fps = mode.fps
    });
  }

  return ret;
}


// Debugging helpers
namespace
{
  template<typename T>
  std::ostream &operator <<(std::ostream &o, const boost::optional<T> &value)
  {
    if (value)
    {
      return o << value.get();
    }

    return o << "Not Present / Default";
  }

  template<typename T>
  std::string to_string(const T &t)
  {
    std::ostringstream o;
    o << t;
    return o.str();
  }

  std::string indent(const std::string &str, const std::size_t spaces)
  {
    std::ostringstream o;
    for (std::size_t i = 0; i < spaces; ++i) o << " ";
    const std::string indention = o.str();

    std::ostringstream ret;
    std::string::size_type last = 0;
    for (std::string::size_type i = 0; i != std::string::npos; i = str.find("\n", i + 1))
    {
      ret << indention << str.substr(last, i - last);
      last = i;
    }

    return ret.str();
  }
}

std::ostream &operator <<(std::ostream &o, const Device::Configuration::ColorStream &value)
{
  return o;
}

std::ostream &operator <<(std::ostream &o, const Device::Configuration::IrStream &value)
{
  return o;
}

std::ostream &operator <<(std::ostream &o, const Device::Configuration::DepthStream &value)
{
  return o;
}

std::ostream &operator <<(std::ostream &o, const Device::Configuration::BodyStream &value)
{
  return o;
}

std::ostream &operator <<(std::ostream &o, const Device::Configuration::ColorizedBodyStream &value)
{
  return o;
}

std::ostream &operator <<(std::ostream &o, const Device::Configuration::MaskedColorStream &value)
{
  return o;
}

std::ostream &operator <<(std::ostream &o, const Device::Configuration &value)
{
  o << "Device::Configuration {" << std::endl
    << "  URI: " << value.uri << std::endl
    << "  Color Stream: " << std::endl;
    // << indent(to_string(value.color_stream), 4) << std::endl;
  return o;
}