#include <gtest/gtest.h>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "climb_main/kinematics/kdl_interface.hpp"
#include "util/test_utils.hpp"

const double TOL = 1e-6;

class KinematicsTest : public testing::Test
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
    robot_->setParameter("body_frame", "root", result);
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

TEST_F(KinematicsTest, Transform)
{
  auto [p, R] = robot_->getTransform("left_foot");
  Eigen::Vector3d p_expected{1, 1, -1};
  EXPECT_NEAR_EIGEN(p, p_expected, TOL) << "Incorrect left foot position";
  Eigen::Matrix3d R_expected;
  R_expected << 0, 0, -1,
    0, 1, 0,
    1, 0, 0;
  EXPECT_NEAR_EIGEN(R, R_expected, TOL) << "Incorrect left foot rotation";
}

TEST_F(KinematicsTest, MixedJacobian)
{
  Eigen::MatrixXd Jm = robot_->getJacobian(true);
  Eigen::MatrixXd Jm_expected(6, 4);
  Jm_expected <<
    1, 0, 0, 0,
    0, 0, 0, 0,
    1, 0, 1, 0,
    0, 1, 0, 1,
    0, 0, 0, 0,
    0, 1, 0, 0;
  EXPECT_NEAR_EIGEN(Jm, Jm_expected, TOL) << "Incorrect mixed Jacobian";
}

TEST_F(KinematicsTest, HandJacobian)
{
  Eigen::MatrixXd Jh = robot_->getHandJacobian();
  Eigen::MatrixXd Jh_expected(6, 4);
  Jh_expected <<
    1, 0, 1, 0,
    0, 0, 0, 0,
    -1, 0, 0, 0,
    0, 1, 0, 1,
    0, 0, 0, 0,
    0, 1, 0, 0;
  EXPECT_NEAR_EIGEN(Jh, Jh_expected, TOL) << "Incorrect hand Jacobian";
}

TEST_F(KinematicsTest, GraspMap)
{
  Eigen::MatrixXd G = robot_->getGraspMap();
  Eigen::MatrixXd G_expected(6, 6);
  G_expected <<
    0, 0, -1, 1, 0, 0,
    0, 1, 0, 0, 1, 0,
    1, 0, 0, 0, 0, 1,
    1, 1, 0, 0, 1, -1,
    -1, 0, 1, -1, 0, -1,
    0, 1, 1, 1, 1, 0;
  EXPECT_NEAR_EIGEN(G, G_expected, TOL) << "Incorrect grasp map";
}

TEST_F(KinematicsTest, Inertial)
{
  Eigen::MatrixXd M = robot_->getMassMatrix();
  Eigen::MatrixXd M_expected(4, 4);
  M_expected << 2, 0, 1, 0,
    0, 2, 0, 1,
    1, 0, 1, 0,
    0, 1, 0, 1;
  EXPECT_NEAR_EIGEN(M, M_expected, TOL) << "Incorrect mass matrix";
}

TEST_F(KinematicsTest, Coriolis)
{
  Eigen::VectorXd C = robot_->getCoriolisVector();
  Eigen::Vector4d C_expected{0, 0, 0, 0};
  EXPECT_NEAR_EIGEN(C, C_expected, TOL) << "Incorrect Coriolis vector";
}

TEST_F(KinematicsTest, Gravitational)
{
  Eigen::VectorXd V =
    robot_->getGravitationalVector(Eigen::Vector3d{0, 0, -10});
  Eigen::Vector4d V_expected{10, 10, 10, 0};
  EXPECT_NEAR_EIGEN(V, V_expected, TOL) << "Incorrect gravitational vector";

  Eigen::MatrixXd dVdg = robot_->getGravitationalMatrix();
  Eigen::MatrixXd dVdg_expected(4, 3);
  dVdg_expected << -1, 0, -1,
    -1, 0, -1,
    0, 0, -1,
    -1, 0, 0;
  EXPECT_NEAR_EIGEN(dVdg, dVdg_expected, TOL) << "Incorrect gravitational matrix";
}

TEST_F(KinematicsTest, CenterOfMass)
{
  Eigen::Vector3d com = robot_->getCenterOfMass();
  Eigen::Vector3d com_expected{0.5, 0, -0.5};
  EXPECT_NEAR_EIGEN(com, com_expected, TOL) << "Incorrect center of mass";
  double m = robot_->getMass();
  double m_expected = 4.0;
  EXPECT_NEAR(m, m_expected, TOL) << "Incorrect mass";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
