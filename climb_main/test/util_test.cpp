#include <gtest/gtest.h>
#include "util/test_utils.hpp"
#include "climb_main/util/ros_utils.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <Eigen/Dense>

using geometry_msgs::msg::Point;
using geometry_msgs::msg::Vector3;
using geometry_msgs::msg::Quaternion;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::Wrench;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Transform;
using namespace RosUtils;

const double TOL = 1e-6;

TEST(RosUtilsTest, Arrays)
{
  Eigen::Vector<double, 6> array_expected = {1, 2, 3, 4, 5, 6};
  Eigen::Vector<double, 6> array = arrayToEigen(eigenToArray(array_expected));
  EXPECT_NEAR_EIGEN(array, array_expected, TOL) << "Array conversion failed";

  Eigen::VectorXd vector_expected = array_expected;
  Eigen::VectorXd vector = vectorToEigen(eigenToVector(vector_expected));
  EXPECT_NEAR_EIGEN(vector, vector_expected, TOL) << "Vector conversion failed";
}

TEST(RosUtilsTest, Geometry)
{
  Eigen::Vector3d point_expected = {1, 2, 3};
  Eigen::Vector3d point = pointToEigen(eigenToPoint(point_expected));
  EXPECT_NEAR_EIGEN(point, point_expected, TOL) << "Point conversion failed";

  Eigen::Vector3d vector3_expected = {1, 2, 3};
  Eigen::Vector3d vector3 = vector3ToEigen(eigenToVector3(vector3_expected));
  EXPECT_NEAR_EIGEN(vector3, vector3_expected, TOL) <<
    "Vector3 conversion failed";

  Eigen::Quaterniond quaternion_expected(0.5, 0.5, 0.5, 0.5);
  Eigen::Quaterniond quaternion =
    quaternionToEigen(eigenToQuaternion(quaternion_expected));
  EXPECT_NEAR_EIGEN(quaternion, quaternion_expected, TOL) <<
    "Quaternion conversion failed";

  Eigen::Vector<double, 6> twist_expected = {1, 2, 3, 4, 5, 6};
  Eigen::VectorXd twist = twistToEigen(eigenToTwist(twist_expected));
  EXPECT_NEAR_EIGEN(twist, twist_expected, TOL) << "Twist conversion failed";

  Eigen::Vector<double, 6> wrench_expected = {1, 2, 3, 4, 5, 6};
  Eigen::VectorXd wrench = wrenchToEigen(eigenToWrench(wrench_expected));
  EXPECT_NEAR_EIGEN(wrench, wrench_expected, TOL) << "Wrench conversion failed";

  Eigen::Isometry3d pose_expected = Eigen::Isometry3d::Identity();
  pose_expected.translation() << vector3_expected;
  pose_expected.linear() = quaternion_expected.toRotationMatrix();
  Eigen::Isometry3d pose = poseToEigen(eigenToPose(pose_expected));
  EXPECT_NEAR_EIGEN(pose.matrix(), pose_expected.matrix(), TOL) <<
    "Pose conversion failed";

  Eigen::Isometry3d transform_expected = pose_expected;
  Eigen::Isometry3d transform =
    transformToEigen(eigenToTransform(transform_expected));
  EXPECT_NEAR_EIGEN(transform.matrix(), transform_expected.matrix(), TOL) <<
    "Transform conversion failed";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
