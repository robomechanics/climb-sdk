#include "climb_main/kinematics/kdl_interface.hpp"
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

bool KdlInterface::loadRobotDescription(std::string description)
{
  initialized_ = KinematicsInterface::loadRobotDescription(description);
  initialized_ = initialized_ && kdl_parser::treeFromString(description, tree_);
  if (initialized_) {
    updateChains();
  }
  return initialized_;
}

void KdlInterface::updateChains()
{
  chains_.clear();
  joints_.clear();
  // Initialize joint indices
  for (size_t i = 0; i < joint_names_.size(); i++) {
    joints_[joint_names_[i]].index = i;
  }
  for (size_t i = 0; i < contact_frames_.size(); i++) {
    std::string contact = contact_frames_[i];
    // Create a chain for each end effector
    KDL::Chain chain;
    if (tree_.getChain(body_frame_, end_effector_frames_[i], chain)) {
      // Add a contact frame to the end of the chain
      chain.addSegment(
        KDL::Segment(KDL::Joint(contact, KDL::Joint::Fixed)));
      // Store the chain, joint positions, and transform
      chains_[contact].chain = chain;
      chains_[contact].joint_pos = KDL::JntArray(chain.getNrOfJoints());
      updateContactFrame(contact, KDL::Frame::Identity());
      // Cache information for each joint in the chain
      int index = 0;  // Joint index in the chain
      for (size_t j = 0; j < chain.getNrOfSegments(); j++) {
        std::string joint = chain.getSegment(j).getJoint().getName();
        if (std::find(joint_names_.begin(), joint_names_.end(), joint) !=
          joint_names_.end())
        {
          chains_[contact].joint_names.push_back(joint);
          joints_[joint].chain_names.push_back(contact);
          joints_[joint].chain_indices.push_back(index);
          index++;
        }
      }
    }
  }
}

Eigen::MatrixXd KdlInterface::getHandJacobian(std::string contact_frame)
{
  if (chains_.find(contact_frame) == chains_.end()) {
    return Eigen::MatrixXd();
  }
  auto chain = chains_[contact_frame];
  KDL::ChainJntToJacSolver solver(chain.chain);
  KDL::Jacobian jac(chain.chain.getNrOfJoints());
  solver.JntToJac(chain.joint_pos, jac);
  jac.changeBase(chain.transform.M.Inverse());
  Eigen::MatrixXd full_jac = Eigen::MatrixXd::Zero(jac.rows(), num_joints_);
  for (size_t i = 0; i < jac.columns(); i++) {
    int j = joints_[chain.joint_names[i]].index;
    full_jac.col(j) = jac.data.col(i);
  }
  return wrench_bases_[contact_frame].transpose() * full_jac;
}

Eigen::MatrixXd KdlInterface::getMixedJacobian(
  std::string contact_frame, bool linear)
{
  if (chains_.find(contact_frame) == chains_.end()) {
    return Eigen::MatrixXd();
  }
  auto chain = chains_[contact_frame];
  KDL::ChainJntToJacSolver solver(chain.chain);
  KDL::Jacobian jac(chain.chain.getNrOfJoints());
  solver.JntToJac(chain.joint_pos, jac);
  Eigen::MatrixXd full_jac = Eigen::MatrixXd::Zero(jac.rows(), num_joints_);
  for (size_t i = 0; i < jac.columns(); i++) {
    int j = joints_[chain.joint_names[i]].index;
    full_jac.col(j) = jac.data.col(i);
  }
  if (linear) {
    return full_jac.topRows(3);
  }
  return full_jac;
}

Eigen::MatrixXd KdlInterface::getGraspMap()
{
  Eigen::MatrixXd grasp(6, num_constraints_);
  int col = 0;
  for (auto contact : contact_frames_) {
    auto transform = chains_[contact].transform.Inverse();
    Eigen::Vector3d position(transform.p.data);
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> rotation(transform.M.data);
    Eigen::MatrixXd block = getAdjoint(position, rotation).transpose() *
      wrench_bases_[contact];
    grasp.block(0, col, 6, block.cols()) = block;
    col += block.cols();
  }
  return grasp;
}

void KdlInterface::updateJointState(const JointState & state)
{
  KinematicsInterface::updateJointState(state);
  for (size_t j = 0; j < state.name.size(); j++) {
    auto indices = joints_[state.name[j]].chain_indices;
    auto names = joints_[state.name[j]].chain_names;
    for (size_t i = 0; i < indices.size(); i++) {
      chains_[names[i]].joint_pos.data[indices[i]] = state.position[j];
    }
  }
  // Update chain transforms
  for (const auto & contact : contact_frames_) {
    KDL::ChainFkSolverPos_recursive fk_solver(chains_[contact].chain);
    fk_solver.JntToCart(chains_[contact].joint_pos, chains_[contact].transform);
  }
}

void KdlInterface::updateContactFrames(
  const std::vector<TransformStamped> & transforms)
{
  for (const auto & transform : transforms) {
    std::string contact = transform.child_frame_id;
    // Compute transform from end effector to contact frame
    auto ee_to_contact = KDL::Frame(
      KDL::Rotation::Quaternion(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w),
      KDL::Vector(
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z));
    // Update contact frame
    updateContactFrame(contact, ee_to_contact);
  }
}

void KdlInterface::updateContactFrame(
  std::string contact, KDL::Frame transform)
{
  // Check if chain exists
  if (chains_.find(contact) == chains_.end()) {
    return;
  }
  // Update contact frame
  auto & segment = chains_[contact].chain.getSegment(
    chains_[contact].chain.getNrOfSegments() - 1);
  segment = KDL::Segment(segment.getName(), segment.getJoint(), transform);
  // Update chain transform
  KDL::ChainFkSolverPos_recursive fk_solver(chains_[contact].chain);
  fk_solver.JntToCart(chains_[contact].joint_pos, chains_[contact].transform);
}

void KdlInterface::declareParameters(const rclcpp::Node::SharedPtr node)
{
  declareParameter(
    node, "body_frame", "base_link",
    "Name of the body frame");
  declareParameter(
    node, "end_effector_frames", std::vector<std::string>(),
    "Name of each end-effector frame");
  declareParameter(
    node, "contact_frames", std::vector<std::string>(),
    "Name of each contact frame");
  declareParameter(
    node, "contact_types", std::vector<std::string>(),
    "Type of each end-effector");
  declareParameter(
    node, "actuator_joints", std::vector<std::string>(),
    "Names of each actuated joint");
}

void KdlInterface::setParameter(
  const rclcpp::Parameter & param,
  [[maybe_unused]] rcl_interfaces::msg::SetParametersResult & result)
{
  if (param.get_name() == "body_frame") {
    body_frame_ = param.as_string();
    if (initialized_) {
      updateChains();
    }
  }
  if (param.get_name() == "end_effector_frames") {
    end_effector_frames_ = param.as_string_array();
    if (initialized_) {
      updateChains();
    }
  }
  if (param.get_name() == "contact_frames") {
    contact_frames_ = param.as_string_array();
    num_contacts_ = contact_frames_.size();
    updateBases();
    if (initialized_) {
      updateChains();
    }
  }
  if (param.get_name() == "contact_types") {
    contact_types_.clear();
    ContactType contact_type;
    for (auto name : param.as_string_array()) {
      if (name == "microspine") {
        contact_type = ContactType::MICROSPINE;
      } else {
        contact_type = ContactType::DEFAULT;
      }
      contact_types_.push_back(contact_type);
    }
    updateBases();
  }
  if (param.get_name() == "actuator_joints") {
    std::vector<std::string> joints = param.as_string_array();
    for (auto joint : joint_names_) {
      if (std::find(joints.begin(), joints.end(), joint) == joints.end()) {
        joints.push_back(joint);
      }
    }
    joint_names_ = joints;
    if (initialized_) {
      updateChains();
    }
  }
}
