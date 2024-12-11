#include <gtest/gtest.h>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "climb_main/kinematics/kdl_interface.hpp"
#include "climb_main/step_planner/step_planner.hpp"
#include "util/test_utils.hpp"

const double TOL = 1e-6;

class StepPlannerTest : public testing::Test
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

    // Initialize step planner
    step_planner_ = std::make_unique<StepPlanner>(robot_);
  }

  std::shared_ptr<KinematicsInterface> robot_;
  std::unique_ptr<StepPlanner> step_planner_;
};

TEST_F(StepPlannerTest, MoveFoothold)
{
  // Set foothold
  Eigen::Isometry3d foothold = Eigen::Isometry3d::Identity();
  foothold.translation() << 1, 0, 0;
  foothold.linear() =
    Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
  step_planner_->step("left_contact", foothold);

  EXPECT_NEAR_EIGEN(
    step_planner_->getFoothold("left_contact").rotation(),
    foothold.rotation(), TOL) << "Failed to set foothold rotation";
  EXPECT_NEAR_EIGEN(
    step_planner_->getFoothold("left_contact").translation(),
    foothold.translation(), TOL) << "Failed to set foothold translation";

  // Move foothold
  Eigen::Vector<double, 6> twist;
  twist << 0, 2, 0, 0, M_PI / 2, 0;
  step_planner_->moveFoothold("left_contact", twist);

  Eigen::Isometry3d expected = Eigen::Isometry3d::Identity();
  expected.translation() << 1, 2, 0;
  expected.linear() =
    Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 1, 0)) *
    Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();

  EXPECT_NEAR_EIGEN(
    step_planner_->getFoothold("left_contact").rotation(),
    expected.rotation(), TOL) << "Incorrect foothold rotation after move";
  EXPECT_NEAR_EIGEN(
    step_planner_->getFoothold("left_contact").translation(),
    expected.translation(), TOL) << "Incorrect foothold translation after move";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
