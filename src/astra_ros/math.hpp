#ifndef _ROS_ASTRA_MATH_HPP_
#define _ROS_ASTRA_MATH_HPP_

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Transform.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>


namespace astra_ros
{
  double sum(const geometry_msgs::Point &v);
  double distance(const geometry_msgs::Point &lhs, const geometry_msgs::Point &rhs);

  geometry_msgs::Point operator +(const geometry_msgs::Point &lhs, const geometry_msgs::Point &rhs);
  geometry_msgs::Point operator -(const geometry_msgs::Point &lhs, const geometry_msgs::Point &rhs);
  geometry_msgs::Point operator *(const geometry_msgs::Point &lhs, const geometry_msgs::Point &rhs);
  geometry_msgs::Point operator /(const geometry_msgs::Point &lhs, const double rhs);

  Eigen::Vector3d toEigen(const geometry_msgs::Point &point);
  Eigen::Vector3d toEigen(const geometry_msgs::Vector3 &vector);
  Eigen::Quaterniond toEigen(const geometry_msgs::Quaternion &quaternion);
  Eigen::Affine3d toEigen(const geometry_msgs::Transform &transform);
  geometry_msgs::Point fromEigen(const Eigen::Vector3d &point);
  geometry_msgs::Quaternion fromEigen(const Eigen::Quaterniond &quaternion);
  geometry_msgs::Transform fromEigen(const Eigen::Affine3d &transform);

  Eigen::Quaterniond lookAt(const Eigen::Vector3d &source_point, const Eigen::Vector3d &dest_point, const Eigen::Vector3d &forward, const Eigen::Vector3d &up);
}



#endif

  