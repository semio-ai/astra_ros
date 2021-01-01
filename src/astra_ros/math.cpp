#include "math.hpp"



using namespace astra_ros;

double astra_ros::sum(const geometry_msgs::Point &v)
{
  return v.x + v.y + v.z;
}

double astra_ros::distance(const geometry_msgs::Point &lhs, const geometry_msgs::Point &rhs)
{
  const auto d = lhs - rhs;
  return sqrt(sum(d * d));
}

geometry_msgs::Point astra_ros::operator +(const geometry_msgs::Point &lhs, const geometry_msgs::Point &rhs)
{
  geometry_msgs::Point ret;
  ret.x = lhs.x + rhs.x;
  ret.y = lhs.y + rhs.y;
  ret.z = lhs.z + rhs.z;
  return ret;
}

geometry_msgs::Point astra_ros::operator -(const geometry_msgs::Point &lhs, const geometry_msgs::Point &rhs)
{
  geometry_msgs::Point ret;
  ret.x = lhs.x - rhs.x;
  ret.y = lhs.y - rhs.y;
  ret.z = lhs.z - rhs.z;
  return ret;
}

geometry_msgs::Point astra_ros::operator *(const geometry_msgs::Point &lhs, const geometry_msgs::Point &rhs)
{
  geometry_msgs::Point ret;
  ret.x = lhs.x * rhs.x;
  ret.y = lhs.y * rhs.y;
  ret.z = lhs.z * rhs.z;
  return ret;
}

geometry_msgs::Point astra_ros::operator /(const geometry_msgs::Point &lhs, const double rhs)
{
  geometry_msgs::Point ret;
  ret.x = lhs.x / rhs;
  ret.y = lhs.y / rhs;
  ret.z = lhs.z / rhs;
  return ret;
}

Eigen::Vector3d astra_ros::toEigen(const geometry_msgs::Point &point)
{
  return Eigen::Vector3d(point.x, point.y, point.z);
}

Eigen::Vector3d astra_ros::toEigen(const geometry_msgs::Vector3 &vector)
{
  return Eigen::Vector3d(vector.x, vector.y, vector.z);
}

Eigen::Quaterniond astra_ros::toEigen(const geometry_msgs::Quaternion &quaternion)
{
  return Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
}

Eigen::Affine3d astra_ros::toEigen(const geometry_msgs::Transform &transform)
{
  Eigen::Affine3d ret = Eigen::Translation3d(toEigen(transform.translation)) * toEigen(transform.rotation);
  return ret;
}

geometry_msgs::Point astra_ros::fromEigen(const Eigen::Vector3d &point)
{
  geometry_msgs::Point ret;
  ret.x = point.x();
  ret.y = point.y();
  ret.z = point.z();
  return ret;
}

geometry_msgs::Quaternion astra_ros::fromEigen(const Eigen::Quaterniond &quaternion)
{
  geometry_msgs::Quaternion ret;
  ret.x = quaternion.x();
  ret.y = quaternion.y();
  ret.z = quaternion.z();
  ret.w = quaternion.w();
  return ret;
}

geometry_msgs::Transform astra_ros::fromEigen(const Eigen::Affine3d &transform)
{
  geometry_msgs::Transform ret;
  Eigen::Quaterniond rotation(transform.rotation());
  Eigen::Vector3d translation = transform.translation();

  ret.translation.x = translation.x();
  ret.translation.y = translation.y();
  ret.translation.z = translation.z();

  ret.rotation = fromEigen(rotation);

  return ret;
}

namespace
{
  const static double EPSILON = 0.000001;
}

// Adapted from https://stackoverflow.com/questions/12435671/quaternion-lookat-function
Eigen::Quaterniond astra_ros::lookAt(const Eigen::Vector3d &source_point, const Eigen::Vector3d &dest_point, const Eigen::Vector3d &forward, const Eigen::Vector3d &up)
{
  using namespace Eigen;

  const Vector3d forward_vec = (dest_point - source_point).normalized();
  const double dot = forward.dot(forward_vec);

  if (std::abs(dot + 1.0f) < EPSILON)
  {
    return Quaterniond(M_PI, up.x(), up.y(), up.z());
  }

  if (std::abs(dot - 1.0f) < EPSILON)
  {
    return Quaterniond::Identity();
  }

  const double angle = acos(dot);
  const Vector3d axis = forward.cross(forward_vec).normalized();

  Quaterniond ret;
  ret = AngleAxisd(angle, axis);
  return ret;
}
