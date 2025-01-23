#include <gtest/gtest.h>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <climb_kinematics/interfaces/kdl_interface.hpp>
#include <climb_util/test_utils.hpp>
#include "climb_control/force_controller.hpp"
#include "climb_control/force_estimator.hpp"

const double TOL = 1e-6;

class ControllerTest : public testing::Test
{
protected:
  void SetUp() override
  {
    // Load URDF
    std::string package_path =
      ament_index_cpp::get_package_share_directory("climb_control");
    std::string urdf_path = package_path + "/test/resources/test_robot.urdf";
    std::ifstream urdf_file(urdf_path);
    if (!urdf_file.is_open()) {
      GTEST_FAIL() << "Failed to open URDF file: " << urdf_path;
    }
    std::string urdf(
      (std::istreambuf_iterator<char>(urdf_file)),
      std::istreambuf_iterator<char>());
    urdf_file.close();
    robot_ = std::make_unique<KdlInterface>();

    // Set parameters
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    robot_->setParameter("body_frame", "base_link", result);
    robot_->setParameter(
      "end_effector_frames", std::vector<std::string> {
      "left_foot", "right_foot"}, result);
    robot_->setParameter(
      "contact_frames", std::vector<std::string> {
      "left_contact", "right_contact"}, result);
    robot_->setParameter(
      "contact_types", std::vector<std::string> {
      "microspine", "microspine"}, result);
    robot_->setParameter(
      "wrist_types", std::vector<std::string> {
      "fixed", "fixed"}, result);
    robot_->setParameter(
      "joint_names", std::vector<std::string> {
      "left_hip", "right_hip", "left_knee", "right_knee"}, result);
    if (!result.successful) {
      GTEST_FAIL() << "Failed to set parameters: " << result.reason;
    }
    std::string error_message;
    if (!robot_->loadRobotDescription(urdf, error_message)) {
      GTEST_FAIL() << error_message;
    }

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
  estimator.setParameter(
    "joint_effort_variance", std::vector<double> {1, 1, 1, 1});
  estimator.setParameter("joint_effort_filter_window", 2);
  estimator.setParameter("gravity", 10.0);
  estimator.setParameter("gravity_offset", true);

  // Set joint effort values
  JointState joint_state;
  joint_state.name = {"left_hip", "right_hip", "left_knee", "right_knee"};
  joint_state.effort = {0, 0, -sqrt(2) * 5, sqrt(2) * 5};
  robot_->updateJointState(joint_state);

  // Estimate without IMU (multiple measurements)
  for (int i = 0; i < 3; i++) {
    estimator.update();
  }
  Eigen::VectorXd forces_robot_on_world = estimator.update();
  Eigen::VectorXd forces = -forces_robot_on_world;  // forces of world on robot
  Eigen::VectorXd forces_expected(6);
  forces_expected << sqrt(2) * 10, 0, sqrt(2) * 10,
    -sqrt(2) * 10, 0, sqrt(2) * 10;
  EXPECT_NEAR_EIGEN(forces, forces_expected, TOL) <<
    "Incorrect force estimate without IMU";

  // Estimate with IMU
  Eigen::Vector3d g{7, 0, -7};
  Eigen::Matrix3d gcov = Eigen::Matrix3d::Identity();
  forces_robot_on_world = estimator.update(g, gcov);
  forces = -forces_robot_on_world;
  EXPECT_NEAR_EIGEN(forces, forces_expected, TOL) <<
    "Incorrect force estimate with IMU";
}

TEST_F(ControllerTest, ForceController)
{
  // Initialize controller
  ForceController controller(robot_);
  controller.defaultParameters();
  controller.setParameter("mass_threshold", 1000.0);
  controller.setParameter("verbose", true);
  controller.setParameter("joint_step", 0.01);

  // Set end effector commands
  EndEffectorCommand command;
  command.frame = {"left_contact", "right_contact"};
  command.mode = {
    EndEffectorCommand::MODE_STANCE, EndEffectorCommand::MODE_STANCE};
  controller.setEndEffectorCommand(command);

  // Update controller output (ignore link masses)
  Eigen::Vector<double, 6> forces{-10, 0, -10, 10, 0, -10};
  ASSERT_TRUE(
    controller.update(forces, Eigen::Isometry3d::Identity())) <<
    "Failed to update controller";

  // Check for static equilibrium (ignore link masses)
  Eigen::MatrixXd Gs = -robot_->getGraspMap();
  Eigen::VectorXd gravity_wrench = -Gs * forces;
  Eigen::VectorXd contact_wrench = Gs * controller.getContactForce();
  EXPECT_NEAR_EIGEN(contact_wrench, -gravity_wrench, TOL) <<
    "Static equilibrium constraint violated";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
