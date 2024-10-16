#include <gtest/gtest.h>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "climb_main/kinematics/kdl_interface.hpp"

const double PI = 3.14159265;
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
      GTEST_SKIP() << "Failed to open URDF file: " << urdf_path;
    }
    std::string urdf(
      (std::istreambuf_iterator<char>(urdf_file)),
      std::istreambuf_iterator<char>());
    urdf_file.close();
    robot_ = std::make_unique<KdlInterface>();

    // Set parameters
    robot_->setParameter("body_frame", "body");
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

TEST_F(KinematicsTest, MixedJacobian)
{
  Eigen::MatrixXd Jm = robot_->getMixedJacobian(true);
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
}

TEST_F(KinematicsTest, HandJacobian)
{
  Eigen::MatrixXd Jh = robot_->getHandJacobian();
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
}

TEST_F(KinematicsTest, GraspMap)
{
  Eigen::MatrixXd G = robot_->getGraspMap();
  Eigen::MatrixXd G_expected(6, 6);
  ASSERT_EQ(G.rows(), G_expected.rows());
  ASSERT_EQ(G.cols(), G_expected.cols());
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

TEST_F(KinematicsTest, Inertial)
{
  Eigen::MatrixXd M = robot_->getMassMatrix();
  Eigen::MatrixXd M_expected(4, 4);
  M_expected << 2, 0, 1, 0,
    0, 2, 0, 1,
    1, 0, 1, 0,
    0, 1, 0, 1;
  ASSERT_EQ(M.rows(), M_expected.rows());
  ASSERT_EQ(M.cols(), M_expected.cols());
  EXPECT_TRUE(M.isApprox(M_expected, TOL)) <<
    "Mass Matrix\nExpected: \n" << M_expected << "\nActual: \n" << M;
}

TEST_F(KinematicsTest, Coriolis)
{
  Eigen::VectorXd C = robot_->getCoriolisVector();
  Eigen::VectorXd C_expected(4);
  C_expected << 0, 0, 0, 0;
  ASSERT_EQ(C.size(), C_expected.size());
  EXPECT_TRUE(C.isApprox(C_expected, TOL)) <<
    "Coriolis Vector\nExpected: \n" << C_expected << "\nActual: \n" << C;
}

TEST_F(KinematicsTest, Gravitational)
{
  Eigen::VectorXd V =
    robot_->getGravitationalVector(Eigen::Vector3d(0, 0, -10));
  Eigen::VectorXd V_expected(4);
  V_expected << 10, 10, 10, 0;
  ASSERT_EQ(V.size(), V_expected.size());
  EXPECT_TRUE(V.isApprox(V_expected, TOL)) <<
    "Gravitational Vector\nExpected: \n" << V_expected << "\nActual: \n" << V;

  Eigen::MatrixXd dVdg = robot_->getGravitationalMatrix();
  Eigen::MatrixXd dVdg_expected(4, 3);
  dVdg_expected << -1, 0, -1,
    -1, 0, -1,
    0, 0, -1,
    -1, 0, 0;
  ASSERT_EQ(dVdg.rows(), 4);
  ASSERT_EQ(dVdg.cols(), 3);
  EXPECT_TRUE(dVdg.isApprox(dVdg_expected, TOL)) <<
    "Gravitational Matrix\nExpected: \n" << dVdg_expected <<
    "\nActual: \n" << dVdg;
}

TEST_F(KinematicsTest, CenterOfMass)
{
  Eigen::Vector3d com = robot_->getCenterOfMass();
  Eigen::Vector3d com_expected(0.5, 0, -0.5);
  EXPECT_TRUE(com.isApprox(com_expected, TOL)) <<
    "Center of Mass\nExpected: \n" << com_expected << "\nActual: \n" << com;
  double m = robot_->getMass();
  double m_expected = 4.0;
  EXPECT_NEAR(m, m_expected, TOL) <<
    "Mass\nExpected: " << m_expected << "\nActual: " << m;
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
