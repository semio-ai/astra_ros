#ifndef _ASTRA_ROS_EXCEPTION_HPP_
#define _ASTRA_ROS_EXCEPTION_HPP_

#include <exception>

#include <astra/capi/astra.h>

namespace astra_ros
{
  /**
   * \class Exception
   * \brief Wraps a astra_status_t when an Astra SDK call fails.
   */
  class Exception : public std::exception
  {
  public:
    /**
     * \param status The astra_status_t value to wrap. ASTRA_STATUS_SUCCESS should not be thrown.
     */
    Exception(const astra_status_t status);

    /**
     * \return A string representing the astra_status_t
     */
    virtual const char *what() const noexcept;

  private:
    astra_status_t status_;
  };
}

#endif