#include "point_cloud.hpp"

#include <sstream>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc.hpp>

using namespace astra_ros;

namespace
{
  
  struct __attribute__((packed)) Rgb
  {
    std::uint8_t r;
    std::uint8_t g;
    std::uint8_t b;
  };

  template<typename T>
  struct RowView
  {
    RowView(const std::uint8_t *const data)
      : data(reinterpret_cast<const T *>(data))
    {
    }

    const T &getColumn(const std::size_t column) const noexcept
    {
      return data[column];
    }

    inline const T &operator[] (const std::size_t column) const noexcept
    {
      return getColumn(column);
    }

    const T *data;
  };

  template<typename T>
  struct ImageView
  {
    ImageView(const std::uint8_t *const data, const std::size_t step)
      : data(data)
      , step(step)
    {
    }

    inline RowView<T> getRow(const std::size_t row) const noexcept
    {
      return RowView<T>(data + row * step);
    }

    inline RowView<T> operator[] (const std::size_t row) const noexcept
    {
      return getRow(row);
    }

    const std::uint8_t *data;
    std::size_t step;
  };


}

bool astra_ros::defaultMask(const std::size_t x, const std::size_t y)
{
  return true;
}

sensor_msgs::PointCloud astra_ros::toPointCloud(const sensor_msgs::Image &rgb, const sensor_msgs::Image &registered_depth, const sensor_msgs::CameraInfo &camera_info, const std::function<bool (const std::size_t x, const std::size_t y)> &mask)
{
  using namespace sensor_msgs::image_encodings;

  // K = [fx 0  cx
  //      0  fy cy
  //      0  0  1 ]
  const double fx = camera_info.K[0];
  const double fy = camera_info.K[4];
  const double cx = camera_info.K[2];
  const double cy = camera_info.K[5];

  if (rgb.encoding != RGB8)
  {
    throw std::runtime_error("Expected RGB image to be RGB8 encoded");
  }

  if (registered_depth.encoding != MONO16)
  {
    throw std::runtime_error("Expected registered depth image to be MONO16 encoded");
  }

  if (rgb.height != registered_depth.height || rgb.width != registered_depth.width)
  {
    std::ostringstream o;
    o << "Size mismatch. RGB image was (" << rgb.width << ", " << rgb.height << ")"
      << ", but registered depth image was (" << registered_depth.width << ", " << registered_depth.height << ")";

    throw std::runtime_error(o.str());
  }

  if (rgb.height * rgb.step != rgb.data.size())
  {
    throw std::runtime_error("RGB image is malformed. height * step != size");
  }

  if (registered_depth.height * registered_depth.step != registered_depth.data.size())
  {
    throw std::runtime_error("Registered depth image is malformed. height * step != size");
  }

  const std::size_t rgb_pixel_size   = bitDepth(rgb.encoding) * numChannels(rgb.encoding);
  const std::size_t depth_pixel_size = bitDepth(rgb.encoding) * numChannels(rgb.encoding);

  // cv::Mat undistorted;
  // cv::undistort(cv::Mat(rgb.height, rgb.width, CV_8UC3, rgb.data.data()), undistorted, )

  sensor_msgs::ChannelFloat32 rgb_channel;
  rgb_channel.name = "rgb";

  sensor_msgs::PointCloud ret;

  const ImageView<Rgb> rgb_view(rgb.data.data(), rgb.step);
  const ImageView<std::uint16_t> depth_view(registered_depth.data.data(), registered_depth.step);

  for (std::size_t y = 0; y < rgb.height; ++y)
  {
    const RowView<Rgb> rgb_row_view = rgb_view[y];
    const RowView<std::uint16_t> depth_row_view = depth_view[y];

    for (std::size_t x = 0; x < rgb.width; ++x)
    {
      // Should we process this pixel?
      if (!mask(x, y)) continue;

      std::uint16_t depth_pixel = depth_row_view[x];

      // Invalid depth pixel
      if (!depth_pixel) continue;

      // Detect if endianness differs from architecture. If so, we need to swap
      // the high and low byte.
      #ifdef ASTRA_ROS_BIG_ENDIAN
      const bool swap_bytes = !registered_depth.is_bigendian;
      #else
      const bool swap_bytes = registered_depth.is_bigendian;
      #endif

      if (swap_bytes)
      {
        depth_pixel = ((depth_pixel & 0x00FF) << 8) | ((depth_pixel & 0xFF00) << 0); 
      }

      const double depth_meters = depth_pixel / 1000.0;

      const double dx = x - cx;

      // Y axis is flipped (pinhole camera model)
      const double dy = y - cy;

      const double dx_over_z = dx / fx;
      const double dy_over_z = dy / fy;

      const double dx_over_z2 = dx_over_z * dx_over_z;
      const double dy_over_z2 = dy_over_z * dy_over_z;

      geometry_msgs::Point32 point;
      point.z = depth_meters / sqrt(1.0 + dx_over_z2 + dy_over_z2);
      point.x = dx_over_z * point.z;
      point.y = dy_over_z * point.z;

      // std::cout << "px: " << x << ", py: "<< y << ", point = " << point << std::endl;

      ret.points.push_back(point);

      const Rgb &pixel = rgb_row_view[x];

      // The channel requires the RGB data to be packed in the bottom 24 bits
      // of a float. Bleh.
      const std::uint32_t rgb_data = (pixel.r << 16) | (pixel.g << 8) | (pixel.b << 0);
      rgb_channel.values.push_back(*reinterpret_cast<const float *>(&rgb_data));
    }
  }

  ret.channels.push_back(rgb_channel);
  return ret;

}