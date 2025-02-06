#ifndef CLIMB_UTIL__EIGEN_UTILS_HPP_
#define CLIMB_UTIL__EIGEN_UTILS_HPP_

#include <Eigen/Geometry>

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
  const Eigen::Isometry3d & transform, const Eigen::Vector<double, 6> & twist);

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
  Eigen::Isometry3d & transform, const Eigen::Vector<double, 6> & twist);

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

/**
 * @brief Rotate a twist by a rotation matrix
 * @param[in] twist Twist in the original frame
 * @param[in] rotation Rotation matrix
 * @return Twist in the rotated frame
 */
Eigen::Vector<double, 6> rotateTwist(
  const Eigen::Vector<double, 6> & twist, const Eigen::Matrix3d & rotation);

/**
 * @brief Rotate a twist by a rotation matrix
 * @param[in,out] twist Twist in the original frame to modify
 * @param[in] rotation Rotation matrix
 */
void rotateTwistInPlace(
  Eigen::Vector<double, 6> & twist, const Eigen::Matrix3d & rotation);

/**
 * @brief Get the skew-symmetric matrix of a vector
 * @param[in] vector Vector
 * @return Skew-symmetric matrix
 */
Eigen::Matrix3d getSkew(const Eigen::Vector3d & vector);

/**
 * @brief Compute a list of indices from a binary mask
 * @param[in] mask Binary mask of 1s and 0s
 * @return Vector of indices where mask is non-zero
 */
Eigen::VectorXi maskToIndex(const Eigen::VectorXi & mask);
}  // namespace EigenUtils

#endif  // CLIMB_UTIL__EIGEN_UTILS_HPP_
