#include "astra_ros/RosDevice.hpp"
#include "astra_ros/Exception.hpp"

#include <ros/ros.h>
#include <string>
#include <unistd.h>

using namespace std::string_literals;
using namespace astra_ros;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "astra_ros_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // Get all parameter keys from the parameter server (I didn't see a way to only get a namespace)
  std::vector<std::string> param_names;
  if (!pnh.getParamNames(param_names))
  {
    ROS_ERROR("NodeHandle::getParamNames() failed. Aborting.");
    return EXIT_FAILURE;
  }

  // Extract the device names from the parameter key list
  std::set<std::string> devices;

  const std::string prefix = pnh.getNamespace() + "/" + DEVICE_NAMESPACE;
  for (const auto &param : param_names)
  {
    if (param.find(prefix) != 0) continue;

    std::string::size_type next_slash = param.find("/", prefix.size() + 1);
    if (next_slash == std::string::npos) next_slash = param.size();
    const std::string device = param.substr(prefix.size() + 1, next_slash - prefix.size() - 1);
    devices.insert(device);
  }

  // Construct the ROS representations of the devices
  std::vector<RosDevice> ros_devices;
  for (const auto &device : devices)
  {
    ros_devices.emplace_back(device, nh, pnh);
  }

  // Finally, start streaming
  ros::Rate rate(35.0);
  while (ros::ok())
  {
    for (auto &ros_device : ros_devices)
    {
      try
      {
        ros_device.update();
      }
      catch (const Exception &exception)
      {
        ROS_ERROR_STREAM("Failed to update device \"" << ros_device.getName() << "\": " << exception.what());
        continue;
      }
      catch(const std::exception &exception)
      {
        ROS_FATAL_STREAM("Unknown exception occurred while updating device \"" << ros_device.getName() << "\". Aborting: " << exception.what());
        return EXIT_FAILURE;
      }
    }

    ros::spinOnce();
    rate.sleep();
  }

  return EXIT_SUCCESS;
}