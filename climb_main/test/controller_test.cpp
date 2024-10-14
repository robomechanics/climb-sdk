#include <gtest/gtest.h>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "climb_main/kinematics/kinematics_interface.hpp"
#include "climb_main/kinematics/kdl_interface.hpp"
#include "climb_main/controller/force_estimator.hpp"

const double PI = 3.14159265;
const double TOL = 1e-6;

class ControllerTest : public testing::Test
{
protected:
  void SetUp() override
  {
    // Load URDF
    std::string package_path =
      ament_index_cpp::get_package_share_directory("climb_main");
    std::string urdf_path = package_path + "/test/resources/test_robot.urdf";
    std::ifstream urdf_file(urdf_path);
    if (!urdf_file.is_open()) {
      GTEST_SKIP() << "Failed to open URDF file: " << urdf_path;
    }
    std::string urdf(
      (std::istreambuf_iterator<char>(urdf_file)),
      std::istreambuf_iterator<char>());
    urdf_file.close();
    robot_ = std::make_unique<KdlInterface>();
    robot_->loadRobotDescription(urdf);

    // Set parameters
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    robot_->setParameter(rclcpp::Parameter("body_frame", "body"), result);
    robot_->setParameter(
      rclcpp::Parameter(
        "end_effector_frames",
        std::vector<std::string>{"left_foot", "right_foot"}), result);
    robot_->setParameter(
      rclcpp::Parameter(
        "contact_frames",
        std::vector<std::string>{"left_contact", "right_contact"}), result);
    robot_->setParameter(
      rclcpp::Parameter(
        "contact_types",
        std::vector<std::string> {"microspine", "microspine"}), result);
    robot_->setParameter(
      rclcpp::Parameter(
        "actuator_joints", std::vector<std::string> {
      "left_hip", "right_hip", "left_knee", "right_knee"}), result);

    // Set joint configuration
    JointState joint_state;
    joint_state.name = {"left_hip", "right_hip", "left_knee", "right_knee"};
    joint_state.position = {0.0, PI / 2, PI / 2, -PI / 2};
    robot_->updateJointState(joint_state);

    // Set contact frames
    TransformStamped left_foot_transform;
    left_foot_transform.child_frame_id = "left_contact";
    left_foot_transform.header.frame_id = "left_foot";
    left_foot_transform.transform.rotation.w = 1;
    TransformStamped right_foot_transform;
    right_foot_transform.child_frame_id = "right_contact";
    right_foot_transform.header.frame_id = "right_foot";
    left_foot_transform.transform.rotation.w = 1;
    robot_->updateContactFrames({left_foot_transform, right_foot_transform});
  }

  std::shared_ptr<KinematicsInterface> robot_;
};

TEST_F(ControllerTest, ForceEstimator)
{
  // Initialize estimator
  ForceEstimator estimator(robot_);
  rcl_interfaces::msg::SetParametersResult result;
  estimator.setParameter(
    rclcpp::Parameter(
      "joint_effort_variance", std::vector<double> {1, 1, 1, 1}),
    result);
  estimator.setParameter(
    rclcpp::Parameter("joint_effort_filter_window", 2), result);
  estimator.setParameter(
    rclcpp::Parameter("gravity", 10.0), result);

  // Set joint effort values
  JointState joint_state;
  joint_state.name = {"left_hip", "right_hip", "left_knee", "right_knee"};
  joint_state.effort = {0, 0, -sqrt(2) * 5, sqrt(2) * 5};
  robot_->updateJointState(joint_state);

  // Estimate without IMU (multiple measurements)
  for (int i = 0; i < 3; i++) {
    estimator.update();
  }
  Eigen::VectorXd forces = estimator.update();
  Eigen::VectorXd forces_expected(6);
  forces_expected << sqrt(2) * 10, 0, sqrt(2) * 10,
    -sqrt(2) * 10, 0, sqrt(2) * 10;
  ASSERT_EQ(forces.size(), 6);
  EXPECT_TRUE(forces.isApprox(forces_expected, TOL)) <<
    "Force Estimate without IMU\nExpected: \n" << forces_expected <<
    "\nActual: \n" << forces;

  // Estimate with IMU
  Imu imu;
  imu.orientation_covariance[0] = -1;
  imu.linear_acceleration.x = 7;
  imu.linear_acceleration.z = -7;
  imu.linear_acceleration_covariance[0] = 1;
  imu.linear_acceleration_covariance[4] = 1;
  imu.linear_acceleration_covariance[8] = 1;
  forces = estimator.update(imu);
  ASSERT_EQ(forces.size(), 6);
  EXPECT_TRUE(forces.isApprox(forces_expected, TOL)) <<
    "Force Estimate with IMU\nExpected: \n" << forces_expected <<
    "\nActual: \n" << forces;
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
