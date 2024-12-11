#include "climb_main/util/eigen_utils.hpp"

namespace EigenUtils
{
using namespace Eigen;

Isometry3d applyTwist(const Isometry3d & transform, const Vector<double, 6> & twist)
{
  Isometry3d result = transform;
  result.translation() += twist.head<3>();
  result.linear() =
    AngleAxisd(twist.tail<3>().norm(), twist.tail<3>().normalized()) *
    transform.rotation();
  return result;
}

void applyTwistInPlace(Isometry3d & transform, const Vector<double, 6> & twist)
{
  transform.translation() += twist.head<3>();
  transform.linear() =
    AngleAxisd(twist.tail<3>().norm(), twist.tail<3>().normalized()) *
    transform.rotation();
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
}  // namespace EigenUtils
