#ifndef EIGEN_UTILS_HPP
#define EIGEN_UTILS_HPP

#include <Eigen/Dense>

/**
 * @brief Utility functions for working with Eigen types
 */
namespace EigenUtils
{
/**
 * @brief Apply a twist to a transform
 * @param[in] transform Original transform
 * @param[in] twist Twist in the transform base frame
 * @return Transform after twist is applied
 */
Eigen::Isometry3d applyTwist(
  const Eigen::Isometry3d & transform, const Eigen::Vector<double, 6> & twist);

/**
 * @brief Apply a twist to a transform
 * @param[in] transform Original transform
 * @param[in] twist Twist in the transform child frame
 * @return Transform after twist is applied
 */
Eigen::Isometry3d applyChildTwist(
  const Eigen::Isometry3d & transform, const Eigen::Vector<double, 6> & twist)
{
  Eigen::Vector<double, 6> twist_base;
  twist_base.head<3>() = transform.rotation() * twist.head<3>();
  twist_base.tail<3>() = transform.rotation() * twist.tail<3>();
  return applyTwist(transform, twist_base);
}

/**
 * @brief Apply a twist to a transform in place
 * @param[in,out] transform Transform to modify
 * @param[in] twist Twist in the transform base frame
 */
void applyTwistInPlace(
  Eigen::Isometry3d & transform, const Eigen::Vector<double, 6> & twist);

/**
 * @brief Apply a twist to a transform in place
 * @param[in,out] transform Transform to modify
 * @param[in] twist Twist in the transform child frame
 */
void applyChildTwistInPlace(
  Eigen::Isometry3d & transform, const Eigen::Vector<double, 6> & twist)
{
  Eigen::Vector<double, 6> twist_base;
  twist_base.head<3>() = transform.rotation() * twist.head<3>();
  twist_base.tail<3>() = transform.rotation() * twist.tail<3>();
  applyTwistInPlace(transform, twist_base);
}

/**
 * @brief Convert a twist to an equivalent transform
 * @param[in] twist Twist in the transform base frame
 * @return Equivalent transform
 */
Eigen::Isometry3d getTransform(const Eigen::Vector<double, 6> & twist);

/**
 * @brief Get the twist between two transforms in the same base frame
 * @param[in] transform1 First transform
 * @param[in] transform2 Second transform
 * @param[in] magnitude Maximum norm of the twist (0.0 for no limit)
 * @return Twist from transform1 to transform2 in the transform base frame
 */
Eigen::Vector<double, 6> getTwist(
  const Eigen::Isometry3d & transform1, const Eigen::Isometry3d & transform2,
  double magnitude = 0.0);

/**
 * @brief Get the twist to a transform from the identity transform
 * @param[in] transform Transform
 * @param[in] magnitude Maximum norm of the twist (0.0 for no limit)
 * @return Twist from identity to transform in the transform base frame
 */
Eigen::Vector<double, 6> getTwist(
  const Eigen::Isometry3d & transform, double magnitude = 0.0);
}  // namespace EigenUtils

#endif  // EIGEN_UTILS_HPP
