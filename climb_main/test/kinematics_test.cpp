#include <gtest/gtest.h>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "climb_main/kinematics/kinematics_interface.hpp"
#include "climb_main/kinematics/kdl_interface.hpp"

const double PI = 3.14159265;
const double TOL = 1e-6;

TEST(kinematics, matrices)
{
  // Load URDF

  std::string package_path = ament_index_cpp::get_package_share_directory("climb_main");
  std::string urdf_path = package_path + "/test/resources/test_robot.urdf";
  std::ifstream urdf_file(urdf_path);
  if (!urdf_file.is_open()) {
    FAIL() << "Failed to open URDF file: " << urdf_path;
  }
  std::string urdf(
    (std::istreambuf_iterator<char>(urdf_file)),
    std::istreambuf_iterator<char>());
  urdf_file.close();
  std::unique_ptr<KinematicsInterface> robot = std::make_unique<KdlInterface>();
  robot->loadRobotDescription(urdf);

  // Set parameters

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  robot->setParameter(rclcpp::Parameter("body_frame", "body"), result);
  robot->setParameter(
    rclcpp::Parameter(
      "end_effector_frames",
      std::vector<std::string>{"left_foot", "right_foot"}), result);
  robot->setParameter(
    rclcpp::Parameter(
      "contact_frames",
      std::vector<std::string>{"left_contact", "right_contact"}), result);
  robot->setParameter(
    rclcpp::Parameter(
      "contact_types",
      std::vector<std::string> {"microspine", "microspine"}), result);
  robot->setParameter(
    rclcpp::Parameter(
      "actuator_joints", std::vector<std::string> {
    "left_hip", "right_hip", "left_knee", "right_knee"}), result);

  // Set robot configuration

  JointState js;
  js.name = {"left_hip", "right_hip", "left_knee", "right_knee"};
  js.position = {0.0, PI / 2, PI / 2, -PI / 2};
  robot->updateJointState(js);
  TransformStamped left_foot_transform;
  left_foot_transform.child_frame_id = "left_contact";
  left_foot_transform.header.frame_id = "left_foot";
  left_foot_transform.transform.rotation.w = 1;
  TransformStamped right_foot_transform;
  right_foot_transform.child_frame_id = "right_contact";
  right_foot_transform.header.frame_id = "right_foot";
  left_foot_transform.transform.rotation.w = 1;
  robot->updateContactFrames({left_foot_transform, right_foot_transform});

  // Test mixed Jacobian

  Eigen::MatrixXd Jm = robot->getMixedJacobian(true);
  Eigen::MatrixXd Jm_expected(6, 4);
  ASSERT_EQ(Jm.rows(), Jm_expected.rows());
  ASSERT_EQ(Jm.cols(), Jm_expected.cols());
  Jm_expected <<
    1, 0, 0, 0,
    0, 0, 0, 0,
    1, 0, 1, 0,
    0, 1, 0, 1,
    0, 0, 0, 0,
    0, 1, 0, 0;
  EXPECT_TRUE(Jm.isApprox(Jm_expected, TOL)) <<
    "Mixed Jacobian\nExpected: \n" << Jm_expected << "\nActual: \n" << Jm;

  // Test hand Jacobian

  Eigen::MatrixXd Jh = robot->getHandJacobian();
  Eigen::MatrixXd Jh_expected(6, 4);
  ASSERT_EQ(Jh.rows(), Jh_expected.rows());
  ASSERT_EQ(Jh.cols(), Jh_expected.cols());
  Jh_expected <<
    1, 0, 1, 0,
    0, 0, 0, 0,
    -1, 0, 0, 0,
    0, 1, 0, 1,
    0, 0, 0, 0,
    0, 1, 0, 0;
  EXPECT_TRUE(Jh.isApprox(Jh_expected, TOL)) <<
    "Hand Jacobian\nExpected: \n" << Jh_expected << "\nActual: \n" << Jh;

  // Test grasp map

  Eigen::MatrixXd G = robot->getGraspMap();
  Eigen::MatrixXd G_expected(6, 6);
  ASSERT_EQ(Jh.rows(), Jh_expected.rows());
  ASSERT_EQ(Jh.cols(), Jh_expected.cols());
  G_expected <<
    0, 0, -1, 1, 0, 0,
    0, 1, 0, 0, 1, 0,
    1, 0, 0, 0, 0, 1,
    1, 1, 0, 0, 1, -1,
    -1, 0, 1, -1, 0, -1,
    0, 1, 1, 1, 1, 0;
  EXPECT_TRUE(G.isApprox(G_expected, TOL)) <<
    "Grasp Map\nExpected: \n" << G_expected << "\nActual: \n" << G;
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
