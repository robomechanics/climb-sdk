#ifndef CLIMB_UTIL__ROS_UTILS_HPP_
#define CLIMB_UTIL__ROS_UTILS_HPP_

#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/wrench.hpp>

/**
 * @brief Utility functions for converting between ROS messages and Eigen types
 */
namespace RosUtils
{

/**
 * @brief Convert a std::array to an Eigen::Vector
 */
template<typename T, std::size_t N>
inline Eigen::Vector<T, N> arrayToEigen(const std::array<T, N> & array)
{
  return Eigen::Map<const Eigen::Vector<T, N>>(array.data(), N).eval();
}

/**
 * @brief Convert a fixed size Eigen::Vector or Eigen::Matrix to a std::array
 */
template<typename Derived>
inline std::array<
  typename Derived::Scalar,
  static_cast<std::size_t>(Derived::SizeAtCompileTime)>
eigenToArray(const Eigen::DenseBase<Derived> & vector)
{
  static_assert(
    Derived::SizeAtCompileTime != Eigen::Dynamic,
    "Vector must have a fixed size.");
  std::array<typename Derived::Scalar,
    static_cast<std::size_t>(Derived::SizeAtCompileTime)> array;
  std::copy(vector.begin(), vector.end(), array.data());
  return array;
}

/**
 * @brief Convert a std::Vector to an Eigen::Vector
 */
template<typename T>
inline Eigen::Vector<T, Eigen::Dynamic> vectorToEigen(
  const std::vector<T> & array)
{
  return Eigen::Map<const Eigen::VectorXd>(array.data(), array.size()).eval();
}

/**
 * @brief Convert a dynamic Eigen::Vector or Eigen::Matrix to a std::Vector
 */
template<typename Derived>
inline std::vector<typename Derived::Scalar> eigenToVector(
  const Eigen::DenseBase<Derived> & vector)
{
  return std::vector<typename Derived::Scalar>(vector.begin(), vector.end());
}

/**
 * @brief Convert a geometry_msgs::msg::Point to an Eigen::Vector3d
 */
inline Eigen::Vector3d pointToEigen(const geometry_msgs::msg::Point & point)
{
  return Eigen::Vector3d(point.x, point.y, point.z);
}

/**
 * @brief Convert an Eigen::Vector3d to a geometry_msgs::msg::Point
 */
inline geometry_msgs::msg::Point eigenToPoint(
  const Eigen::Ref<const Eigen::Vector3d> & point)
{
  geometry_msgs::msg::Point msg;
  msg.x = point.x();
  msg.y = point.y();
  msg.z = point.z();
  return msg;
}

/**
 * @brief Convert a geometry_msgs::msg::Vector3 to an Eigen::Vector3d
 */
inline Eigen::Vector3d vector3ToEigen(
  const geometry_msgs::msg::Vector3 & vector3)
{
  return Eigen::Vector3d(vector3.x, vector3.y, vector3.z);
}

/**
 * @brief Convert an Eigen::Vector3d to a geometry_msgs::msg::Vector3
 */
inline geometry_msgs::msg::Vector3 eigenToVector3(
  const Eigen::Ref<const Eigen::Vector3d> & vector3)
{
  geometry_msgs::msg::Vector3 msg;
  msg.x = vector3.x();
  msg.y = vector3.y();
  msg.z = vector3.z();
  return msg;
}

/**
 * @brief Convert a geometry_msgs::msg::Quaternion to an Eigen::Quaterniond
 */
inline Eigen::Quaterniond quaternionToEigen(
  const geometry_msgs::msg::Quaternion & quaternion)
{
  return Eigen::Quaterniond(
    quaternion.w, quaternion.x, quaternion.y, quaternion.z);
}

/**
 * @brief Convert an Eigen::Quaterniond to a geometry_msgs::msg::Quaternion
 */
inline geometry_msgs::msg::Quaternion eigenToQuaternion(
  const Eigen::Quaterniond & quaternion)
{
  geometry_msgs::msg::Quaternion msg;
  msg.w = quaternion.w();
  msg.x = quaternion.x();
  msg.y = quaternion.y();
  msg.z = quaternion.z();
  return msg;
}

/**
 * @brief Convert a geometry_msgs::msg::Twist to an Eigen::Vector<double, 6>
 */
inline Eigen::Vector<double, 6> twistToEigen(
  const geometry_msgs::msg::Twist & twist)
{
  return Eigen::Vector<double, 6>(
    twist.linear.x, twist.linear.y, twist.linear.z,
    twist.angular.x, twist.angular.y, twist.angular.z);
}

/**
 * @brief Convert an Eigen::Vector<double, 6> to a geometry_msgs::msg::Twist
 */
inline geometry_msgs::msg::Twist eigenToTwist(
  const Eigen::Ref<const Eigen::Vector<double, 6>> & twist)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = twist(0);
  msg.linear.y = twist(1);
  msg.linear.z = twist(2);
  msg.angular.x = twist(3);
  msg.angular.y = twist(4);
  msg.angular.z = twist(5);
  return msg;
}

/**
 * @brief Convert a geometry_msgs::msg::Wrench to an Eigen::Vector<double, 6>
 */
inline Eigen::Vector<double, 6> wrenchToEigen(
  const geometry_msgs::msg::Wrench & wrench)
{
  return Eigen::Vector<double, 6>(
    wrench.force.x, wrench.force.y, wrench.force.z,
    wrench.torque.x, wrench.torque.y, wrench.torque.z);
}

/**
 * @brief Convert an Eigen::Vector<double, 6> to a geometry_msgs::msg::Wrench
 */
inline geometry_msgs::msg::Wrench eigenToWrench(
  const Eigen::Ref<const Eigen::Vector<double, 6>> & wrench)
{
  geometry_msgs::msg::Wrench msg;
  msg.force.x = wrench(0);
  msg.force.y = wrench(1);
  msg.force.z = wrench(2);
  msg.torque.x = wrench(3);
  msg.torque.y = wrench(4);
  msg.torque.z = wrench(5);
  return msg;
}

/**
 * @brief Convert a geometry_msgs::msg::Pose to an Eigen::Isometry3d
 */
inline Eigen::Isometry3d poseToEigen(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = pointToEigen(pose.position);
  result.linear() = quaternionToEigen(pose.orientation).toRotationMatrix();
  return result;
}

/**
 * @brief Convert an Eigen::Isometry3d to a geometry_msgs::msg::Pose
 */
inline geometry_msgs::msg::Pose eigenToPose(const Eigen::Isometry3d & pose)
{
  geometry_msgs::msg::Pose msg;
  msg.position = eigenToPoint(pose.translation());
  msg.orientation = eigenToQuaternion(Eigen::Quaterniond(pose.rotation()));
  return msg;
}

/**
 * @brief Convert a geometry_msgs::msg::Transform to an Eigen::Isometry3d
 */
inline Eigen::Isometry3d transformToEigen(
  const geometry_msgs::msg::Transform & transform)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = vector3ToEigen(transform.translation);
  result.linear() = quaternionToEigen(transform.rotation).toRotationMatrix();
  return result;
}

/**
 * @brief Convert an Eigen::Isometry3d to a geometry_msgs::msg::Transform
 */
inline geometry_msgs::msg::Transform eigenToTransform(
  const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Transform msg;
  msg.translation = eigenToVector3(transform.translation());
  msg.rotation = eigenToQuaternion(Eigen::Quaterniond(transform.rotation()));
  return msg;
}

/**
 * @brief Negate a geometry_msgs::msg::Twist
 */
inline geometry_msgs::msg::Twist operator-(
  const geometry_msgs::msg::Twist & twist)
{
  geometry_msgs::msg::Twist result;
  result.linear.x = -twist.linear.x;
  result.linear.y = -twist.linear.y;
  result.linear.z = -twist.linear.z;
  result.angular.x = -twist.angular.x;
  result.angular.y = -twist.angular.y;
  result.angular.z = -twist.angular.z;
  return result;
}

/**
 * @brief Negate a geometry_msgs::msg::Wrench
 */
inline geometry_msgs::msg::Wrench operator-(
  const geometry_msgs::msg::Wrench & wrench)
{
  geometry_msgs::msg::Wrench result;
  result.force.x = -wrench.force.x;
  result.force.y = -wrench.force.y;
  result.force.z = -wrench.force.z;
  result.torque.x = -wrench.torque.x;
  result.torque.y = -wrench.torque.y;
  result.torque.z = -wrench.torque.z;
  return result;
}

/**
 * @brief Negate a geometry_msgs::msg::Pose
 */
inline geometry_msgs::msg::Pose operator-(
  const geometry_msgs::msg::Pose & pose)
{
  geometry_msgs::msg::Pose result;
  result.position.x = -pose.position.x;
  result.position.y = -pose.position.y;
  result.position.z = -pose.position.z;
  result.orientation.w = pose.orientation.w;
  result.orientation.x = -pose.orientation.x;
  result.orientation.y = -pose.orientation.y;
  result.orientation.z = -pose.orientation.z;
  return result;
}

/**
 * @brief Negate a geometry_msgs::msg::Transform
 */
inline geometry_msgs::msg::Transform operator-(
  const geometry_msgs::msg::Transform & transform)
{
  geometry_msgs::msg::Transform result;
  result.translation.x = -transform.translation.x;
  result.translation.y = -transform.translation.y;
  result.translation.z = -transform.translation.z;
  result.rotation.w = transform.rotation.w;
  result.rotation.x = -transform.rotation.x;
  result.rotation.y = -transform.rotation.y;
  result.rotation.z = -transform.rotation.z;
  return result;
}
}  // namespace RosUtils

#endif  // CLIMB_UTIL__ROS_UTILS_HPP_
