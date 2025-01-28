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
  geometry_utils::Polytope p;  // x + y < 5, x > 0, y > 0, -1 < z < 1
  p.addFacet(Eigen::Vector3d{1, 1, 0}, 5);
  p.addFacet(Eigen::Vector3d{-1, 0, 0}, 0);
  p.addFacet(Eigen::Vector3d{0, -1, 0}, 0);
  p.addFacet(Eigen::Vector3d{0, 0, -1}, 1);
  p.addFacet(Eigen::Vector3d{0, 0, 1}, 1);
  Eigen::MatrixXd A_expected(5, 3);
  A_expected << 1, 1, 0, -1, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 1;
  EXPECT_NEAR_EIGEN(p.A, A_expected, TOL) << "Constraint matrix incorrect";
  Eigen::Vector<double, 5> b_expected{5, 0, 0, 1, 1};
  EXPECT_NEAR_EIGEN(p.b, b_expected, TOL) << "Constraint vector incorrect";

  Eigen::Vector3d x1{1, 1, 0};
  Eigen::Vector3d x2{2, 2, 0};
  Eigen::Vector3d x3{3, 3, 0};
  Eigen::Vector3d d_expected{1, sqrt(2) / 2, -sqrt(2) / 2};
  Eigen::Vector<bool, 3> c_expected{true, true, false};
  Eigen::Matrix3d X;
  X << x1, x2, x3;
  EXPECT_TRUE(p.contains(x1)) << "Point in polytope error";
  EXPECT_EQ(p.containsAll(X), c_expected) << "Bulk point in polytope error";
  EXPECT_NEAR(p.distance(x1), d_expected(0), TOL) << "Distance error";
  EXPECT_NEAR_EIGEN(p.distanceAll(X), d_expected, TOL) <<
    "Bulk distance error";
  EXPECT_NEAR(p.distance(x1, x2), 1.5 * sqrt(2), TOL) <<
    "Distance along direction error";
  Eigen::Vector3d dd_expected{1.5 * sqrt(2), 0.5 * sqrt(2), -0.5 * sqrt(2)};
  EXPECT_NEAR_EIGEN(p.distanceAll(X, x2), dd_expected, TOL) <<
    "Bulk distance along direction error";
  Eigen::Vector3d dd_expected_tol{sqrt(2), 0.5 * sqrt(2), -0.5 * sqrt(2)};
  Eigen::Vector3d x_hat{1, 0, 0};
  EXPECT_NEAR_EIGEN(p.distanceAll(X, x_hat, PI / 4), dd_expected_tol, TOL) <<
    "Bulk distance along direction with angular tolerance error";
  Eigen::Vector3d clip_expected{2.5, 2.5, 0};
  EXPECT_NEAR_EIGEN(p.clip(x3, x2), clip_expected, TOL) << "Clipping error";

  EXPECT_NEAR_EIGEN((p * -2).distanceAll(X * -2), d_expected * 2, TOL) <<
    "Uniform scaling error (right multiplication)";
  EXPECT_NEAR_EIGEN((-2 * p).distanceAll(-2 * X), d_expected * 2, TOL) <<
    "Uniform scaling error (left multiplication)";
  EXPECT_NEAR_EIGEN((p / -2).distanceAll(X / -2), d_expected / 2, TOL) <<
    "Uniform scaling error (division)";
  EXPECT_NEAR_EIGEN((-p).distanceAll(-X), d_expected, TOL) <<
    "Uniform scaling error (negation)";
  p *= -2;
  EXPECT_NEAR_EIGEN(p.distanceAll(-2 * X), d_expected * 2, TOL) <<
    "Uniform scaling in place error (multiplication)";
  p /= -2;
  EXPECT_NEAR_EIGEN(p.distanceAll(X), d_expected, TOL) <<
    "Uniform scaling in place error (division)";

  Eigen::Vector3d v = Eigen::Vector3d{1, 2, 3};
  Eigen::Isometry3d V = Eigen::Isometry3d::Identity();
  V.translate(v);
  EXPECT_NEAR_EIGEN((p + v).distanceAll(V * X), d_expected, TOL) <<
    "Translation error (right addition)";
  EXPECT_NEAR_EIGEN((v + p).distanceAll(V * X), d_expected, TOL) <<
    "Translation error (left addition)";
  EXPECT_NEAR_EIGEN((p - v).distanceAll(V.inverse() * X), d_expected, TOL) <<
    "Translation error (right subtraction)";
  EXPECT_NEAR_EIGEN((v - p).distanceAll(V * -X), d_expected, TOL) <<
    "Translation error (left subtraction)";
  p += v;
  EXPECT_NEAR_EIGEN(p.distanceAll(V * X), d_expected, TOL) <<
    "Translation in place error (addition)";
  p -= v;
  EXPECT_NEAR_EIGEN(p.distanceAll(X), d_expected, TOL) <<
    "Translation in place error (subtraction)";

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translate(v);
  T.rotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
  Eigen::Matrix3d R = T.rotation();
  EXPECT_NEAR_EIGEN((R * p).distanceAll(R * X), d_expected, TOL) <<
    "Transformation error (rotation)";
  EXPECT_NEAR_EIGEN((T * p).distanceAll(T * X), d_expected, TOL) <<
    "Transformation error (isometry)";

  Eigen::Array3d s{1, -2, 0};
  Eigen::ArrayXd x1_scaled = x1.array() * s;
  auto p_scaled = p.scaled(s);
  Eigen::Vector3d d_s_expected = {
    p.distance(x1, Eigen::Vector3d{1, 0, 0}) * s(0),
    p.distance(x1, Eigen::Vector3d{0, 1, 0}) * s(1),
    p.distance(x1, Eigen::Vector3d{0, 0, 1}) * s(2)};
  EXPECT_NEAR(
    p_scaled.distance(x1_scaled, Eigen::Vector3d{1, 0, 0}),
    d_s_expected(0), TOL) << "Non-uniform scaling error";

  auto p_top = geometry_utils::Polytope(p.A.topRows(3), p.b.head(3));
  auto p_bot = geometry_utils::Polytope(p.A.bottomRows(2), p.b.bottomRows(2));
  p_top.intersect(p_bot);
  EXPECT_NEAR_EIGEN(p_top.A, p.A, TOL) << "Intersection error";
  EXPECT_NEAR_EIGEN(p_top.b, p.b, TOL) << "Intersection error";

  auto b1 = geometry_utils::Polytope::createBox(
    Eigen::Vector3d{-1, -2, -3}, Eigen::Vector3d{1, 2, 3});
  auto b2 = geometry_utils::Polytope::createBox(
    Eigen::Vector3d{-1, -1, -1}, Eigen::Vector3d{2, 2, 2});
  geometry_utils::Polytope p1(b1.A, b1.b);
  EXPECT_NEAR_EIGEN(
    (b1 + v).distanceAll(X / 2), (p1 + v).distanceAll(X / 2), TOL) <<
    "Box translation error";
  EXPECT_NEAR_EIGEN(
    (b1 * -2).distanceAll(X / 2), (p1 * -2).distanceAll(X / 2), TOL) <<
    "Box uniform scaling error";

  auto b2_scaled = b2.scaled(Eigen::Vector3d{-1, 2, 0});
  Eigen::Vector<double, 6> b2_scaled_expected{2, 2, 0, 1, 4, 0};
  EXPECT_NEAR_EIGEN(b2_scaled.b, b2_scaled_expected, TOL) <<
    "Box non-uniform scaling error";

  Eigen::Vector<double, 6> intersection_expected{1, 1, 1, 1, 2, 2};
  EXPECT_NEAR_EIGEN(b1.intersection(b2).b, intersection_expected, TOL) <<
    "Box intersection error";

  Eigen::Vector<double, 6> sum_expected{2, 3, 4, 3, 4, 5};
  EXPECT_NEAR_EIGEN((b1 + b2).b, sum_expected, TOL) << "Box addition error";

  auto b1_elim = b1.eliminated(1);
  Eigen::Vector<double, 4> b_eliminate_expected{1, 3, 1, 3};
  Eigen::Matrix<double, 4, 2> A_eliminate_expected;
  A_eliminate_expected << -1, 0, 0, -1, 1, 0, 0, 1;
  EXPECT_NEAR_EIGEN(b1_elim.b, b_eliminate_expected, TOL) <<
    "Box elimination error (b)";
  EXPECT_NEAR_EIGEN(b1_elim.A, A_eliminate_expected, TOL) <<
    "Box elimination error (A)";

  auto p_elim = p.eliminated(2);
  Eigen::MatrixXd X_elim = X.block(0, 0, 2, 3);
  EXPECT_EQ(p_elim.containsAll(X_elim), c_expected) << "Elimination error";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
