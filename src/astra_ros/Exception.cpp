#include "astra_ros/Exception.hpp"

#include <unordered_map>

#include "util.hpp"

using namespace astra_ros;

Exception::Exception(const astra_status_t status)
  : status_(status)
{
}

const char *Exception::what() const noexcept
{
  try
  {
    // statusToString can throw if status_ is of an Unknown value
    return statusToString(status_).c_str();
  }
  catch(...)
  {
    return "Unknown";
  }
}