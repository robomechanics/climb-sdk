#include <gtest/gtest.h>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "climb_main/kinematics/kdl_interface.hpp"
#include "climb_main/controller/force_estimator.hpp"
#include "climb_main/controller/force_controller.hpp"
#include <climb_main/controller/osqp_interface.hpp>

const double PI = 3.14159265;
const double TOL = 1e-3;

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

    // Set parameters
    robot_->setParameter("body_frame", "root");
    robot_->setParameter(
      "end_effector_frames", std::vector<std::string> {
      "left_foot", "right_foot"});
    robot_->setParameter(
      "contact_frames", std::vector<std::string> {
      "left_contact", "right_contact"});
    robot_->setParameter(
      "contact_types", std::vector<std::string> {
      "microspine", "microspine"});
    robot_->setParameter(
      "joint_names", std::vector<std::string> {
      "left_hip", "right_hip", "left_knee", "right_knee"});
    std::string error_message;
    if (!robot_->loadRobotDescription(urdf, error_message)) {
      GTEST_SKIP() << error_message;
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

TEST_F(ControllerTest, ForceController)
{
  // Initialize controller
  ForceController controller(robot_);
  EndEffectorCommand command;

  // Set end effector commands
  command.frame = {"left_contact", "right_contact"};
  command.mode = {
    EndEffectorCommand::MODE_STANCE, EndEffectorCommand::MODE_STANCE};
  controller.setEndEffectorCommand(command);

  // Update controller output
  // Eigen::VectorXd displacement = controller.update(Eigen::VectorXd::Zero(6));
  // Eigen::VectorXd position = robot_->getJointPosition() + displacement;
  // Eigen::MatrixXd G = robot_->getGraspMap();
  // Eigen::MatrixXd J = robot_->getHandJacobian("left_contact");
  // JointState joint_state;
  // joint_state.name = {"left_hip", "right_hip", "left_knee", "right_knee"};
  // joint_state.position = std::vector<double>(position.data(), position.data() + position.size());
  // robot_->updateJointState(joint_state);
}

TEST_F(ControllerTest, OsqpInterface)
{
  // Setup problem:
  // Minimize 1/2 (x^2 + 2y^2) + 4x + y
  // Subject to 0 <= x <= inf, 0 < y <= inf, 3 <= x + y <= inf
  std::cout << "Begin test" << std::endl;
  std::unique_ptr<QpInterface> solver = std::make_unique<OsqpInterface>();
  Eigen::MatrixXd H(2, 2);
  H << 1, 0,
    0, 2;
  Eigen::VectorXd f(2);
  f << 4, 1;
  Eigen::MatrixXd A(3, 2);
  A << 1, 0,
    0, 1,
    1, 1;
  Eigen::VectorXd lb(3);
  lb << 0, 0, 3;
  Eigen::VectorXd ub(3);
  ub << std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::infinity();

  // Solve without sparsity structure
  std::cout << "About to solve" << std::endl;
  ASSERT_TRUE(solver->solve(H, f, A, lb, ub)) << "Dense solve failed";
  std::cout << "Done solving" << std::endl;
  Eigen::VectorXd solution = solver->getSolution();
  double cost = solver->getCost();
  Eigen::VectorXd solution_expected(2);
  solution_expected << 1, 2;
  EXPECT_TRUE(solution.isApprox(solution_expected, TOL)) <<
    "Dense solution\nExpected: \n" << solution_expected <<
    "\nActual: \n" << solution;
  double cost_expected = 10.5;
  EXPECT_NEAR(cost, cost_expected, TOL) << "Dense solution cost is incorrect";

  // Solve with sparsity structure
  Eigen::SparseMatrix<double> H_sparsity = H.sparseView();
  Eigen::SparseMatrix<double> A_sparsity = A.sparseView();
  ASSERT_TRUE(solver->solve(H, f, A, lb, ub, H_sparsity, A_sparsity)) <<
    "Sparse solve failed";
  solution = solver->getSolution();
  EXPECT_TRUE(solution.isApprox(solution_expected, TOL)) <<
    "Sparse solution\nExpected: \n" << solution_expected <<
    "\nActual: \n" << solution;
  cost = solver->getCost();
  EXPECT_NEAR(cost, cost_expected, TOL) << "Sparse solution cost is incorrect";

  // Update problem
  ASSERT_TRUE(solver->update(H, f, A, lb, ub)) << "Update failed";
  solution = solver->getSolution();
  EXPECT_TRUE(solution.isApprox(solution_expected, TOL)) <<
    "Updated solution\nExpected: \n" << solution_expected <<
    "\nActual: \n" << solution;
  cost = solver->getCost();
  EXPECT_NEAR(cost, cost_expected, TOL) << "Updated solution cost is incorrect";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
