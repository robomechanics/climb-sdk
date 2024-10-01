#include "climb_main/kinematics/kdl_interface.hpp"
#include <kdl_parser/kdl_parser.hpp>

bool KdlInterface::loadRobotDescription(std::string urdf_file)
{
  initialized_ = kdl_parser::treeFromString(urdf_file, tree_);
  if (initialized_) {
    update_chains();
  }
  return initialized_;
}

Eigen::MatrixXd KdlInterface::getJacobian()
{
  return Eigen::MatrixXd::Zero(num_constraints, num_joints);
}

Eigen::MatrixXd KdlInterface::getGraspMap()
{
  return Eigen::MatrixXd::Zero(6, num_constraints);
}

void KdlInterface::update_chains()
{
  // chains_.clear();
  // for (const auto & frame : end_effector_frames_) {
  //   KDL::Chain chain;
  //   if (tree_.getChain(base_frame_, frame, chain)) {
  //     chains_[frame] = chain;
  //   }
  // }
}

void KdlInterface::declareParameters(const rclcpp::Node::SharedPtr node)
{
  declareParameter(
    node, "end_effector_frames", std::vector<std::string>(),
    "Name of each end-effector frame");
}

void KdlInterface::setParameter(
  const rclcpp::Parameter & param,
  rcl_interfaces::msg::SetParametersResult & result)
{
  if (param.get_name() == "end_effector_frames") {
    ee_frames_ = param.as_string_array();
    if (initialized_) {
      update_chains();
    }
  }
}
