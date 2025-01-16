#include <gtest/gtest.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>

#include "climb_util/test_utils.hpp"
#include "climb_util/ros_utils.hpp"
#include "climb_util/eigen_utils.hpp"
#include "climb_util/geometry_utils.hpp"

using geometry_msgs::msg::Point;
using geometry_msgs::msg::Vector3;
using geometry_msgs::msg::Quaternion;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::Wrench;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Transform;
const double TOL = 1e-6;

TEST(RosUtilsTest, Arrays)
{
  Eigen::Vector<double, 6> array_expected{1, 2, 3, 4, 5, 6};
  Eigen::Vector<double, 6> array =
    RosUtils::arrayToEigen(RosUtils::eigenToArray(array_expected));
  EXPECT_NEAR_EIGEN(array, array_expected, TOL) << "Array conversion failed";

  Eigen::VectorXd vector_expected = array_expected;
  Eigen::VectorXd vector =
    RosUtils::vectorToEigen(RosUtils::eigenToVector(vector_expected));
  EXPECT_NEAR_EIGEN(vector, vector_expected, TOL) << "Vector conversion failed";
}

TEST(RosUtilsTest, Geometry)
{
  Eigen::Vector3d point_expected{1, 2, 3};
  Eigen::Vector3d point =
    RosUtils::pointToEigen(RosUtils::eigenToPoint(point_expected));
  EXPECT_NEAR_EIGEN(point, point_expected, TOL) << "Point conversion failed";

  Eigen::Vector3d vector3_expected{1, 2, 3};
  Eigen::Vector3d vector3 =
    RosUtils::vector3ToEigen(RosUtils::eigenToVector3(vector3_expected));
  EXPECT_NEAR_EIGEN(vector3, vector3_expected, TOL) <<
    "Vector3 conversion failed";

  Eigen::Quaterniond quaternion_expected(0.5, 0.5, 0.5, 0.5);
  Eigen::Quaterniond quaternion =
    RosUtils::quaternionToEigen(
    RosUtils::eigenToQuaternion(quaternion_expected));
  EXPECT_NEAR_EIGEN(quaternion, quaternion_expected, TOL) <<
    "Quaternion conversion failed";

  Eigen::Vector<double, 6> twist_expected{1, 2, 3, 4, 5, 6};
  Eigen::VectorXd twist =
    RosUtils::twistToEigen(RosUtils::eigenToTwist(twist_expected));
  EXPECT_NEAR_EIGEN(twist, twist_expected, TOL) << "Twist conversion failed";

  Eigen::Vector<double, 6> wrench_expected{1, 2, 3, 4, 5, 6};
  Eigen::VectorXd wrench =
    RosUtils::wrenchToEigen(RosUtils::eigenToWrench(wrench_expected));
  EXPECT_NEAR_EIGEN(wrench, wrench_expected, TOL) << "Wrench conversion failed";

  Eigen::Isometry3d pose_expected = Eigen::Isometry3d::Identity();
  pose_expected.translation() << vector3_expected;
  pose_expected.linear() = quaternion_expected.toRotationMatrix();
  Eigen::Isometry3d pose =
    RosUtils::poseToEigen(RosUtils::eigenToPose(pose_expected));
  EXPECT_NEAR_EIGEN(pose.matrix(), pose_expected.matrix(), TOL) <<
    "Pose conversion failed";

  Eigen::Isometry3d transform_expected = pose_expected;
  Eigen::Isometry3d transform =
    RosUtils::transformToEigen(RosUtils::eigenToTransform(transform_expected));
  EXPECT_NEAR_EIGEN(transform.matrix(), transform_expected.matrix(), TOL) <<
    "Transform conversion failed";
}

TEST(EigenUtilsTest, Skew)
{
  Eigen::Vector3d v1{1, 2, 3};
  Eigen::Vector3d v2{4, 5, 6};
  Eigen::Matrix3d v1_skew = EigenUtils::getSkew(v1);
  EXPECT_NEAR_EIGEN(v1_skew * v2, v1.cross(v2), TOL) << "Skew matrix incorrect";
}

TEST(GeometryUtilsTest, Polytope)
{
  geometry_utils::Polytope p1;
  p1.addFacet({1, 0, 0}, 1);
  p1.addFacet({0, 1, 0}, 2);
  p1.addFacet({0, 0, 1}, 3);
  Eigen::MatrixXd A_expected = Eigen::Matrix3d::Identity();
  EXPECT_NEAR_EIGEN(p1.A, A_expected, TOL) << "Constraint matrix incorrect";
  Eigen::Vector3d b_expected_3{1, 2, 3};
  EXPECT_NEAR_EIGEN(p1.b, b_expected_3, TOL) << "Constraint vector incorrect";

  geometry_utils::Polytope p2 = geometry_utils::Polytope::createBox(
    Eigen::Vector3d{-1, -2, -3}, Eigen::Vector3d{1, 2, 3});
  Eigen::Vector<double, 6> b_expected;
  b_expected << 1, 2, 3, 1, 2, 3;
  EXPECT_NEAR_EIGEN(p2.b, b_expected, TOL) << "Box initialization incorrect";

  Eigen::Vector3d x{0, -1, 2};
  EXPECT_TRUE((p2.A * x - p2.b).maxCoeff() < 0) << "Point in polytope error";
  x = {0, -3, 2};
  EXPECT_TRUE((p2.A * x - p2.b).maxCoeff() > 0) << "Point in polytope error";

  p2 += Eigen::Vector3d{1, 2, 3};
  b_expected << 0, 0, 0, 2, 4, 6;
  EXPECT_NEAR_EIGEN(p2.b, b_expected, TOL) << "Translation error";

  p2 *= 0.5;
  b_expected *= 0.5;
  EXPECT_NEAR_EIGEN(p2.b, b_expected, TOL) << "Uniform scaling error";

  p2.scale({0, 1, 2});
  b_expected << 0, 0, 0, 0, 2, 6;
  EXPECT_NEAR_EIGEN(p2.b, b_expected, TOL) << "Box non-uniform scaling error";

  geometry_utils::Polytope p3 = geometry_utils::Polytope::createBox(
    Eigen::Vector3d{-1, -1, -1}, Eigen::Vector3d{2, 2, 2});
  auto p4 = p2.intersection(p3);
  b_expected << 0, 0, 0, 0, 2, 2;
  EXPECT_NEAR_EIGEN(p4.b, b_expected, TOL) << "Box intersection error";

  p2 += p3;
  b_expected << 1, 1, 1, 2, 4, 8;
  EXPECT_NEAR_EIGEN(p2.b, b_expected, TOL) << "Box addition error";

  p2 = Eigen::Isometry3d::Identity() * p2;
  EXPECT_NEAR_EIGEN(p2.b, b_expected, TOL) << "Transformation error";

  auto p5 = Eigen::Vector3d{2, 4, 6} - 2 * (-p4 / 2 + Eigen::Vector3d{1, 2, 3});
  EXPECT_NEAR_EIGEN(p5.b, p4.b, TOL) << "Operator arithmetic error";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
