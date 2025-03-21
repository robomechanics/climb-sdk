#include "climb_util/eigen_utils.hpp"

namespace EigenUtils
{
using Eigen::AngleAxisd;
using Eigen::Isometry3d;
using Eigen::Vector;

Isometry3d applyTwist(const Isometry3d & transform, const Vector<double, 6> & twist)
{
  Isometry3d result = transform;
  result.translation() += twist.head<3>();
  result.linear() =
    AngleAxisd(twist.tail<3>().norm(), twist.tail<3>().normalized()) *
    transform.rotation();
  return result;
}

Eigen::Isometry3d applyChildTwist(
  const Eigen::Isometry3d & transform, const Eigen::Vector<double, 6> & twist)
{
  return applyTwist(transform, rotateTwist(twist, transform.rotation()));
}

void applyTwistInPlace(Isometry3d & transform, const Vector<double, 6> & twist)
{
  transform.translation() += twist.head<3>();
  transform.linear() =
    AngleAxisd(twist.tail<3>().norm(), twist.tail<3>().normalized()) *
    transform.rotation();
}

void applyChildTwistInPlace(
  Eigen::Isometry3d & transform, const Eigen::Vector<double, 6> & twist)
{
  applyTwistInPlace(transform, rotateTwist(twist, transform.rotation()));
}

Isometry3d getTransform(const Vector<double, 6> & twist)
{
  Isometry3d transform = Isometry3d::Identity();
  transform.translation() = twist.head<3>();
  transform.linear() = AngleAxisd(
    twist.tail<3>().norm(),
    twist.tail<3>().normalized()).toRotationMatrix();
  return transform;
}

Vector<double, 6> getTwist(
  const Isometry3d & transform1, const Isometry3d & transform2,
  double magnitude)
{
  Vector<double, 6> twist;
  twist.head<3>() = transform2.translation() - transform1.translation();
  AngleAxisd rot(transform1.rotation().transpose() * transform2.rotation());
  twist.tail<3>() = rot.angle() * rot.axis();
  if (magnitude > 0.0) {
    twist = twist.normalized() * std::min(twist.norm(), magnitude);
  }
  return twist;
}

Vector<double, 6> getTwist(const Isometry3d & transform, double magnitude)
{
  Vector<double, 6> twist;
  twist.head<3>() = transform.translation();
  AngleAxisd rot(transform.rotation());
  twist.tail<3>() = rot.angle() * rot.axis();
  if (magnitude > 0.0) {
    twist = twist.normalized() * std::min(twist.norm(), magnitude);
  }
  return twist;
}

Eigen::Vector<double, 6> rotateTwist(
  const Eigen::Vector<double, 6> & twist, const Eigen::Matrix3d & rotation)
{
  Eigen::Vector<double, 6> rotated;
  rotated.head<3>() = rotation * twist.head<3>();
  rotated.tail<3>() = rotation * twist.tail<3>();
  return rotated;
}

void rotateTwistInPlace(
  Eigen::Vector<double, 6> & twist, const Eigen::Matrix3d & rotation)
{
  twist.head<3>() = rotation * twist.head<3>();
  twist.tail<3>() = rotation * twist.tail<3>();
}

Eigen::Matrix3d getSkew(const Eigen::Vector3d & vector)
{
  Eigen::Matrix3d skew;
  skew << 0, -vector(2), vector(1),
    vector(2), 0, -vector(0),
    -vector(1), vector(0), 0;
  return skew;
}

Eigen::VectorXi maskToIndex(const Eigen::VectorXi & mask)
{
  Eigen::VectorXi index(mask.count());
  int j = 0;
  for (int i = 0; i < mask.size(); ++i) {
    if (mask(i)) {
      index(j++) = i;
    }
  }
  return index;
}
}  // namespace EigenUtils
