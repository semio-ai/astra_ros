# astra_ros

astra_ros wraps the Orbbec Astra SDK for ROS. It can be used as a library (with or without ROS), a nodelet, or a node.

## Installation

```
cd /path/to/your_catkin_workspace/src
git clone https://github.com/semio-ai/astra_ros.git
```

## Compilation

The Astra SDK must be installed on the system. Set the environment variable `ASTRA_ROOT` to the root of the Astra SDK if it cannot be found.

```
cd /path/to/your_catkin_workspace
catkin_make
```

## Documentation

### API Documentation

View the C++ API documentation at <https://semio-ai.github.io/astra_ros/annotated.html>.

## Node

### Parameters

  - `~/devices` - A map of devices to open and publish
  - `~/devices/$DEVICE_NAME` - Parameters for the device with the name `$DEVICE_NAME`. `$DEVICE_NAME` will determine topic namespacing.
  - `~/devices/$DEVICE_NAME/color` - Must be present for the color stream to be enabled. Can either contain sub-keys or be set to `true`.
  - `~/devices/$DEVICE_NAME/color/running` - Set whether the color stream is running. Defaults to `true`.
  - `~/devices/$DEVICE_NAME/color/mirrored` - Set whether the color frames are mirrored horizontally. Defaults to `true`.
  - `~/devices/$DEVICE_NAME/ir` - Must be present for the IR stream to be enabled. Can either contain sub-keys or be set to `true`.
  - `~/devices/$DEVICE_NAME/ir/running` - Set whether the IR stream is running. Defaults to `true`.
  - `~/devices/$DEVICE_NAME/depth` - Must be present for the depth stream to be enabled. Can either contain sub-keys or be set to `true`.
  - `~/devices/$DEVICE_NAME/depth/running` - Set whether the depth stream is running. Defaults to `true`.
  - `~/devices/$DEVICE_NAME/depth/mirrored` - Set whether the depth frames are mirrored horizontally. Defaults to `true`.
  - `~/devices/$DEVICE_NAME/body` - Must be present for the body stream to be enabled. Can either contain sub-keys or be set to `true`.
  - `~/devices/$DEVICE_NAME/body` - Must be present for the body stream to be enabled. Can either contain sub-keys or be set to `true`.
  - `~/devices/$DEVICE_NAME/body/running` - Set whether the body stream is running. Defaults to `true`.
  - `~/devices/$DEVICE_NAME/body/publish_body_markers` - Set whether visualization markers are published for tracked bodies. Defaults to `false`.
  - `~/devices/$DEVICE_NAME/body/publish_body_mask` - Set whether the body mask image is published. Defaults to `true`.
  - `~/devices/$DEVICE_NAME/body/publish_floor_mask` - Set whether the floor mask image is published. Defaults to `true`.
  - `~/devices/$DEVICE_NAME/body/license` - The license string for the Orbbec Body Tracking SDK. Depending on the SDK version used, this value must be set.
  - `~/devices/$DEVICE_NAME/body/frame_id` - Must be set to the orbbec camera's optical frame.
  - `~/devices/$DEVICE_NAME/hand` - Must be present for the hand stream to be enabled. Can either contain sub-keys or be set to `true`.
  - `~/devices/$DEVICE_NAME/hand/running` - Set whether the hand stream is running. Defaults to `true`.
  - `~/devices/$DEVICE_NAME/masked_color` - Must be present for the masked color stream to be enabled. Can either contain sub-keys or be set to `true`.
  - `~/devices/$DEVICE_NAME/masked_color/running` - Set whether the masked color stream is running. Defaults to `true`.
  - `~/devices/$DEVICE_NAME/colorized_body` - Must be present for the colorized body stream to be enabled. Can either contain sub-keys or be set to `true`.
  - `~/devices/$DEVICE_NAME/colorized_body/running` - Set whether the colorized body stream is running. Defaults to `true`.
  - `~/devices/$DEVICE_NAME/point` - Must be present for the point stream to be enabled. Can either contain sub-keys or be set to `true`.
  - `~/devices/$DEVICE_NAME/point/running` - Set whether the point stream is running. Defaults to `true`.

#### Example ROS Parameter file

```
devices:
  default:
    color: true
    depth: true
    body:
      publish_body_markers: true
      frame_id: "quori/head_camera_optical"
      license: "$YOUR_LICENSE"
```

### Topics

  - `color/image_color: sensor_msgs/Image` - The color image
  - `color/camera_info: sensors_msgs/CameraInfo` - The color camera's intrinsics
  - `ir/image: sensor_msgs/Image` - The IR image
  - `depth/image: sensor_msgs/Image` - The depth image
  - `depth/camera_info: sensors_msgs/CameraInfo` - The depth camera's intrinsics
  - `body/frame: astra_ros/BodyFrame` - The body frames
  - `body/markers: visualization_msgs/MarkerArray` - The body markers
  - `body/mask: sensor_msgs/Image` - The body mask
  - `body/floor_mask: sensor_msgs/Image` - The floor mask
  - `colorized_body/image: sensor_msgs/Image` - The colorized body image
  - `masked_color/image: sensor_msgs/Image` - The masked color image
  - `point_cloud: sensor_msgs/PointCloud` - The RGBD point cloud


### Services

If the color stream is enabled:
  - `color/get_usb_info: astra_ros/UsbInfo` - Get the USB information for the color camera.
  - `color/get_mirrored: astra_ros/GetMirrored` - Get whether the color frame is mirrored horizontally.
  - `color/set_mirrored: astra_ros/SetMirrored` - Set whether the color frame is mirrored horizontally.
  - `color/get_image_stream_modes: astra_ros/GetImageStreamModes` - Get the image stream modes supported by the color camera.
  - `color/get_image_stream_mode: astra_ros/GetImageStreamMode` - Get the current image stream mode used by the color camera.
  - `color/set_image_stream_mode: astra_ros/SetImageStreamMode` - Configure the image stream mode for the color camera.
  - `color/get_running: astra_ros/GetRunning` - Get the running status of the color camera.
  - `color/set_running: astra_ros/SetRunning` - Set the running status of the color camera. False means that no color frames will be published.


If the depth stream is enabled:
  - `depth/get_chip_id: astra_ros/GetChipId` - Get the Orbbec chip ID.
  - `depth/get_mirrored: astra_ros/GetMirrored` - Get whether the depth frame is mirrored horizontally.
  - `depth/set_mirrored: astra_ros/SetMirrored` - Set whether the depth frame is mirrored horizontally.
  - `depth/get_registration: astra_ros/GetRegistration` - Get whether the depth frame is registered.
  - `depth/set_registration: astra_ros/SetRegistration` - Set whether the depth frame is registered.
  - `depth/get_usb_info: astra_ros/UsbInfo` - Get the USB information for the depth camera.
  - `depth/get_image_stream_modes: astra_ros/GetImageStreamModes` - Get the image stream modes supported by the depth camera.
  - `depth/get_image_stream_mode: astra_ros/GetImageStreamMode` - Get the current image stream mode used by the depth camera.
  - `depth/set_image_stream_mode: astra_ros/SetImageStreamMode` - Configure the image stream mode for the depth camera.
  - `depth/get_running: astra_ros/GetRunning` - Get the running status of the depth camera.
  - `depth/set_running: astra_ros/SetRunning` - Set the running status of the depth camera. False means that no depth frames will be published.

If the IR stream is enabled:
  - `ir/get_usb_info: astra_ros/UsbInfo` - Get the USB information for the IR camera
  - `ir/get_image_stream_modes: astra_ros/GetImageStreamModes` - Get the image stream modes supported by the IR camera
  - `ir/get_image_stream_mode: astra_ros/GetImageStreamMode` - Get the current image stream mode used by the IR camera
  - `ir/set_image_stream_mode: astra_ros/SetImageStreamMode` - Configure the image stream mode for the IR camera
  - `ir/get_running: astra_ros/GetRunning` - Get the running status of the IR camera
  - `ir/set_running: astra_ros/SetRunning` - Set the running status of the IR camera. False means that no IR frames will be published.

If the colorized body stream is enabled:
  - `colorized_body/get_running: astra_ros/GetRunning` - Get the running status of the colorized body stream
  - `colorized_body/set_running: astra_ros/SetRunning` - Set the running status of the colorized body stream. False means that no colorized body frames will be published.

If the hand stream is enabled:
  - `hand/get_running: astra_ros/GetRunning` - Get the running status of the hand stream
  - `hand/set_running: astra_ros/SetRunning` - Set the running status of the hand stream. False means that no hand frames will be published.

If the point stream is enabled:
  - `point/get_running: astra_ros/GetRunning` - Get the running status of the point stream
  - `point/set_running: astra_ros/SetRunning` - Set the running status of the point stream. False means that no point frames will be published.

## License

`astra_ros` is released under the terms of the 3-Clause BSD License. See `LICENSE` for details.
