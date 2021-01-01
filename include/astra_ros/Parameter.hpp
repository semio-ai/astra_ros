#ifndef _ASTRA_ROS_PARAMETER_HPP_
#define _ASTRA_ROS_PARAMETER_HPP_

#include <functional>
#include <boost/optional.hpp>

#include <mutex>

namespace astra_ros
{
  /**
   * \class ParameterImpl
   * \brief Internal implementation of a Parameter
   */
  template<typename T>
  class ParameterImpl
  {
  public:
    typedef T ValueType;
    typedef std::function<bool (const T &value, const T &previous)> OnChange;

    ParameterImpl()
      : value_(T())
    {
    }

    ParameterImpl(const T &value)
      : value_(value)
    {
    }
    
    bool set(const T &value) noexcept
    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (on_change_)
      {
        if ((*on_change_)(value, value_))
        {
          // Accept the changes
          value_ = value;
          return true;
        }

        return false;
      }

      // No handler, always accept the change
      value_ = value;
      return true;
    }

    const T &get() const noexcept
    {
      std::lock_guard<std::mutex> lock(mutex_);
      return value_;
    }

    void setOnChangeHandler(const boost::optional<OnChange> &on_change)
    {
      on_change_ = on_change;
    }

    const boost::optional<OnChange> &getOnChangeHandler() const
    {
      return on_change_;
    }

  private:
    mutable std::mutex mutex_;
    
    T value_;
    boost::optional<OnChange> on_change_;
  };

  // Parameter wraps a value to make it "live"
  // A listener can register a callback when the value changes
  /**
   * \class Parameter
   * \brief A "live" value that can be changed at runtime. Changes are broadcast to a listener.
   * 
   * Parameters can be copied cheaply. They are backed by `ParameterImpl`, which implements the actual logic.
   */
  template<typename T>
  class Parameter
  {
  public:
    typedef typename ParameterImpl<T>::ValueType ValueType;
    typedef typename ParameterImpl<T>::OnChange OnChange;

    /**
     * Create a `Parameter` with the value `T()` (the default value for the type `T`) 
     */
    Parameter()
      : impl_(std::make_shared<ParameterImpl<T>>())
    {
    }

    /**
     * Create a `Parameter` with a given value
     */
    Parameter(const T &value)
      : impl_(std::make_shared<ParameterImpl<T>>(value))
    {
    }

    /**
     * Update the value of the `Parameter`. Changes are broadcast to the `OnChange` listener.
     */
    bool set(const T &value)
    {
      return impl_->set(value);
    }

    /**
     * Retrieve the value of the `Parameter`.
     */
    const T &get() const
    {
      return impl_->get();
    }

    /**
     * Convenience method. Equivalent to `get()`.
     */
    const T &operator *() const
    {
      return get();
    }

    /**
     * Set the `OnChange` handler that is called when values change. This shouldn't be used by the
     * Parameter "client".
     */
    void setOnChangeHandler(const boost::optional<OnChange> &on_change)
    {
      impl_->setOnChangeHandler(on_change);
    }

    /**
     * Convience method for setting the `OnChange` handler with semantics equivalent to `std::bind`. 
     */
    template<typename F, typename... Args>
    void bindOnChangeHandler(F &&f, Args &&...args)
    {
      setOnChangeHandler(bind(f, args...));
    }

    /**
     * Retrieve the current onChange handler
     * \return boost::none if not set, otherwise the onChange handler
     */
    const boost::optional<OnChange> &getOnChangeHandler() const
    {
      return impl_->getOnChangeHandler();
    }

    /**
     * Helper method used by bindOnChangeHandler. Wraps std::bind in an OnChange.
     */
    template<typename F, typename... Args>
    static OnChange bind(F &&f, Args &&...args)
    {
      return OnChange(std::bind(f, args..., std::placeholders::_1, std::placeholders::_2));
    }

  private:
    std::shared_ptr<ParameterImpl<T>> impl_;
  };

}



#endif