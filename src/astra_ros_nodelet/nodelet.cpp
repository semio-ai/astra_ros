// tf
// nodelets

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <astra_ros/Exception.hpp>
#include <astra_ros/RosDevice.hpp>

#include <thread>
#include <atomic>
#include <vector>
#include <set>
#include <memory>
#include <functional>

namespace astra_ros
{
  class Nodelet : public nodelet::Nodelet
  {
  public:
    void onInit()
    {
      ros::NodeHandle &nh = getNodeHandle();
      ros::NodeHandle &pnh = getPrivateNodeHandle();

      // Get all parameter keys from the parameter server (I didn't see a way to only get a namespace)
      std::vector<std::string> param_names;
      if (!pnh.getParamNames(param_names))
      {
        NODELET_ERROR("NodeHandle::getParamNames() failed. Aborting.");
        return;
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
      for (const auto &device : devices)
      {
        ros_devices_.emplace_back(device, nh, pnh);
      }

      running_ = true;
      thread_ = std::make_unique<std::thread>(std::bind(&Nodelet::run, this));
    }

    ~Nodelet()
    {
      running_ = false;
      thread_->join();
    }

  private:
    void run()
    {
      // Finally, start streaming
      ros::Rate rate(35.0);
      while (running_)
      {
        for (auto &ros_device : ros_devices_)
        {
          try
          {
            ros_device.update();
          }
          catch (const Exception &exception)
          {
            NODELET_ERROR_STREAM("Failed to update device \"" << ros_device.getName() << "\": " << exception.what());
            continue;
          }
          catch(const std::exception &exception)
          {
            NODELET_FATAL_STREAM("Unknown exception occurred while updating device \"" << ros_device.getName() << "\". Aborting: " << exception.what());
            return;
          }
        }

        ros::spinOnce();
        rate.sleep();
      }
    }

    std::atomic<bool> running_;
    std::unique_ptr<std::thread> thread_;
    std::vector<RosDevice> ros_devices_;
  };
}

PLUGINLIB_EXPORT_CLASS(astra_ros::Nodelet, nodelet::Nodelet);
