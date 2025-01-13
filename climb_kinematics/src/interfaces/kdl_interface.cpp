#include "climb_kinematics/interfaces/kdl_interface.hpp"

#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>

bool KdlInterface::initialize(std::string & error_message)
{
  // Parent class initialization
  if (!KinematicsInterface::initialize(error_message)) {
    return false;
  }
  // Replace floating joints with fixed joints to suppress warnings
  for (auto & joint : urdf_model_.joints_) {
    if (joint.second->type == urdf::Joint::FLOATING) {
      joint.second->type = urdf::Joint::FIXED;
    }
  }
  // Load KDL tree from URDF model
  initialized_ = false;
  if (!kdl_parser::treeFromUrdfModel(urdf_model_, tree_)) {
    error_message = "KDL could not parse URDF model";
    return false;
  }
  // Clear cached data
  chains_.clear();
  matrices_.clear();
  transforms_.clear();
  jacobians_.clear();
  // Initialize joint indices
  joint_indices_.clear();
  for (size_t i = 0; i < num_joints_; i++) {
    joint_indices_[joint_names_[i]] = i;
  }
  // Initialize missing contact frames
  for (size_t i = 0; i < num_contacts_; i++) {
    std::string contact = contact_frames_[i];
    std::string end_effector = end_effector_frames_[i];
    if (tree_.getSegment(contact) == tree_.getSegments().end()) {
      if (tree_.getSegment(end_effector) == tree_.getSegments().end()) {
        error_message = "Could not find chain: " + body_frame_ +
          " -> " + contact + " or " + body_frame_ + " -> " + end_effector;
        return false;
      } else {
        tree_.addSegment(
          KDL::Segment(
            contact, KDL::Joint(contact + "_joint", KDL::Joint::None)),
          end_effector);
      }
    }
    contact_transforms_[contact] = KDL::Frame::Identity();
  }
  return initialized_ = true;
}

KDL::Frame KdlInterface::getTransformKdl(
  const std::string & parent, const std::string & child)
{
  if (transforms_.find({parent, child}) != transforms_.end()) {
    return transforms_[{parent, child}];
  }
  auto chain = getChain(parent, child);
  KDL::ChainFkSolverPos_recursive solver(chain.kdl_chain);
  KDL::Frame transform;
  solver.JntToCart(getJointArray(joint_pos_, chain), transform);
  transforms_[{parent, child}] = transform;
  return transform;
}

Eigen::Isometry3d KdlInterface::getTransform(
  const std::string & parent, const std::string & child)
{
  auto transform = getTransformKdl(parent, child);
  Eigen::Vector3d position(transform.p.data);
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> rotation(transform.M.data);
  Eigen::Isometry3d transform_eigen = Eigen::Isometry3d::Identity();
  transform_eigen.linear() = rotation;
  transform_eigen.translation() = position;
  return transform_eigen;
}

Eigen::MatrixXd KdlInterface::getHandJacobian(const std::string & contact_frame)
{
  auto chain = getChain(body_frame_, contact_frame);
  KDL::ChainJntToJacSolver solver(chain.kdl_chain);
  KDL::Jacobian jac(chain.joints);
  solver.JntToJac(getJointArray(joint_pos_, chain), jac);
  auto transform = getTransformKdl(body_frame_, contact_frame);
  jac.changeBase(transform.M.Inverse());
  Eigen::MatrixXd full_jac = Eigen::MatrixXd::Zero(jac.rows(), num_joints_);
  for (size_t i = 0; i < jac.columns(); i++) {
    full_jac.col(chain.joint_indices[i]) = jac.data.col(i);
  }
  return wrench_bases_[contact_frame].transpose() * full_jac;
}

Eigen::MatrixXd KdlInterface::getHandJacobian()
{
  if (matrices_.find("hand_jacobian") != matrices_.end()) {
    return matrices_["hand_jacobian"];
  }
  Eigen::MatrixXd jac = KinematicsInterface::getHandJacobian();
  matrices_["hand_jacobian"] = jac;
  return jac;
}

Eigen::MatrixXd KdlInterface::getJacobian(
  const std::string & parent, const std::string & child, bool linear)
{
  if (jacobians_.find({parent, child}) != jacobians_.end()) {
    return linear ?
           jacobians_[{parent, child}].topRows(3) :
           jacobians_[{parent, child}];
  }
  auto chain = getChain(parent, child);
  KDL::ChainJntToJacSolver solver(chain.kdl_chain);
  KDL::Jacobian jac(chain.joints);
  solver.JntToJac(getJointArray(joint_pos_, chain), jac);
  Eigen::MatrixXd full_jac = Eigen::MatrixXd::Zero(jac.rows(), num_joints_);
  for (size_t i = 0; i < jac.columns(); i++) {
    full_jac.col(chain.joint_indices[i]) = jac.data.col(i);
  }
  jacobians_[{parent, child}] = full_jac;
  if (linear) {
    return full_jac.topRows(3);
  }
  return full_jac;
}

Eigen::MatrixXd KdlInterface::getJacobian(bool linear)
{
  if (matrices_.find(linear ? "jac_linear" : "jac") != matrices_.end()) {
    return matrices_[linear ? "jac_linear" : "jac"];
  }
  Eigen::MatrixXd jac = KinematicsInterface::getJacobian(linear);
  matrices_[linear ? "jac_linear" : "jac"] = jac;
  return jac;
}

Eigen::Matrix<double, 6, Eigen::Dynamic> KdlInterface::getGraspMap()
{
  if (matrices_.find("grasp") != matrices_.end()) {
    return matrices_["grasp"];
  }
  Eigen::MatrixXd grasp = KinematicsInterface::getGraspMap();
  matrices_["grasp"] = grasp;
  return grasp;
}

Eigen::MatrixXd KdlInterface::getMassMatrix()
{
  if (matrices_.find("mass") != matrices_.end()) {
    return matrices_["mass"];
  }
  Eigen::MatrixXd full_mass = Eigen::MatrixXd::Zero(num_joints_, num_joints_);
  std::unordered_set<std::string> segments;
  for (const auto & end_segment : tree_.getSegments()) {
    // Skip segments that are not leaves of the KDL tree
    if (!end_segment.second.children.empty()) {
      continue;
    }
    auto chain = getChain(body_frame_, end_segment.first);
    auto copy_chain = KDL::Chain(chain.kdl_chain);
    // Remove duplicate contributions from segments shared by multiple chains
    for (auto & segment : copy_chain.segments) {
      if (!segments.insert(segment.getName()).second) {
        segment.setInertia(KDL::RigidBodyInertia());
      }
    }
    // Compute mass matrix for each chain
    KDL::ChainDynParam solver(copy_chain, KDL::Vector(0, 0, 0));
    KDL::JntSpaceInertiaMatrix mass(chain.joints);
    solver.JntToMass(getJointArray(joint_pos_, chain), mass);
    // Accumulate full mass matrix
    Eigen::MatrixXd intermediate_mass =
      Eigen::MatrixXd::Zero(num_joints_, mass.columns());
    for (size_t i = 0; i < chain.joints; i++) {
      intermediate_mass.row(chain.joint_indices[i]) = mass.data.row(i);
    }
    for (size_t i = 0; i < chain.joints; i++) {
      full_mass.col(chain.joint_indices[i]) += intermediate_mass.col(i);
    }
  }
  matrices_["mass"] = full_mass;
  return full_mass;
}

Eigen::VectorXd KdlInterface::getCoriolisVector()
{
  if (matrices_.find("coriolis") != matrices_.end()) {
    return matrices_["coriolis"];
  }
  Eigen::VectorXd full_coriolis = Eigen::VectorXd::Zero(num_joints_);
  std::unordered_set<std::string> segments;
  for (const auto & end_segment : tree_.getSegments()) {
    // Skip segments that are not leaves of the KDL tree
    if (!end_segment.second.children.empty()) {
      continue;
    }
    auto chain = getChain(body_frame_, end_segment.first);
    auto copy_chain = KDL::Chain(chain.kdl_chain);
    // Remove duplicate contributions from segments shared by multiple chains
    for (auto & segment : copy_chain.segments) {
      if (!segments.insert(segment.getName()).second) {
        segment.setInertia(KDL::RigidBodyInertia());
      }
    }
    // Compute Coriolis vector for each chain
    KDL::ChainDynParam solver(copy_chain, KDL::Vector(0, 0, 0));
    KDL::JntArray coriolis_vec(chain.joints);
    solver.JntToCoriolis(
      getJointArray(joint_pos_, chain),
      getJointArray(joint_vel_, chain), coriolis_vec);
    // Accumulate full coriolis vector
    for (size_t i = 0; i < chain.joint_names.size(); i++) {
      full_coriolis(chain.joint_indices[i]) += coriolis_vec(i);
    }
  }
  matrices_["coriolis"] = full_coriolis;
  return full_coriolis;
}

Eigen::VectorXd KdlInterface::getGravitationalVector(
  const Eigen::Vector3d & gravity)
{
  Eigen::VectorXd full_gravity = Eigen::VectorXd::Zero(num_joints_);
  std::unordered_set<std::string> segments;
  for (const auto & end_segment : tree_.getSegments()) {
    // Skip segments that are not leaves of the KDL tree
    if (!end_segment.second.children.empty()) {
      continue;
    }
    auto chain = getChain(body_frame_, end_segment.first);
    auto copy_chain = KDL::Chain(chain.kdl_chain);
    // Remove duplicate contributions from segments shared by multiple chains
    for (auto & segment : copy_chain.segments) {
      if (!segments.insert(segment.getName()).second) {
        segment.setInertia(KDL::RigidBodyInertia());
      }
    }
    // Compute gravity vector for each chain
    KDL::Vector gravity_kdl(gravity(0), gravity(1), gravity(2));
    KDL::ChainDynParam solver(copy_chain, gravity_kdl);
    KDL::JntArray gravity_vec(chain.joints);
    solver.JntToGravity(getJointArray(joint_pos_, chain), gravity_vec);
    // Accumulate full gravity vector
    for (size_t i = 0; i < chain.joint_names.size(); i++) {
      full_gravity(chain.joint_indices[i]) += gravity_vec(i);
    }
  }
  return full_gravity;
}

Eigen::MatrixXd KdlInterface::getGravitationalMatrix()
{
  if (matrices_.find("dVdg") != matrices_.end()) {
    return matrices_["dVdg"];
  }
  Eigen::MatrixXd gravity = Eigen::MatrixXd::Zero(num_joints_, 3);
  Eigen::Vector3d g;
  g << 1, 0, 0;
  gravity.col(0) = getGravitationalVector(g);
  g << 0, 1, 0;
  gravity.col(1) = getGravitationalVector(g);
  g << 0, 0, 1;
  gravity.col(2) = getGravitationalVector(g);
  matrices_["dVdg"] = gravity;
  return gravity;
}

Eigen::Vector3d KdlInterface::getCenterOfMass()
{
  if (matrices_.find("com") != matrices_.end()) {
    return matrices_["com"];
  }
  KDL::Vector com = KDL::Vector::Zero();
  double mass = 0;
  std::unordered_set<std::string> segments;
  // Sum contributions from each chain
  for (const auto & end_segment : tree_.getSegments()) {
    // Skip segments that are not leaves of the KDL tree
    if (!end_segment.second.children.empty()) {
      continue;
    }
    auto chain = getChain(body_frame_, end_segment.first);
    KDL::ChainFkSolverPos_recursive solver(chain.kdl_chain);
    KDL::Frame transform;
    // Sum contributions from each segment
    for (size_t i = 0; i < chain.segments; i++) {
      auto segment = chain.kdl_chain.getSegment(i);
      if (segment.getInertia().getMass() == 0) {
        continue;
      }
      if (segments.insert(segment.getName()).second) {
        solver.JntToCart(getJointArray(joint_pos_, chain), transform, i + 1);
        mass += segment.getInertia().getMass();
        com += transform * segment.getInertia().getCOG() *
          segment.getInertia().getMass();
      }
    }
  }
  if (mass == 0) {
    matrices_["com"] = Eigen::Vector3d::Zero();
    return Eigen::Vector3d::Zero();
  }
  matrices_["com"] = Eigen::Vector3d(com.data) / mass;
  return Eigen::Vector3d(com.data) / mass;
}

void KdlInterface::updateJointState(const JointState & state)
{
  KinematicsInterface::updateJointState(state);
  matrices_.clear();
  transforms_.clear();
  jacobians_.clear();
}

void KdlInterface::updateContactFrames(
  const std::vector<TransformStamped> & transforms)
{
  matrices_.clear();
  transforms_.clear();
  jacobians_.clear();
  for (const auto & transform : transforms) {
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
    contact_transforms_[transform.child_frame_id] = ee_to_contact;
  }
}

KDL::JntArray KdlInterface::getJointArray(
  Eigen::VectorXd var, Chain chain) const
{
  KDL::JntArray joint_array(chain.joints);
  for (size_t i = 0; i < chain.joints; i++) {
    joint_array(i) = var(chain.joint_indices[i]);
  }
  return joint_array;
}

KdlInterface::Chain KdlInterface::getChain(
  const std::string & parent, const std::string & child)
{
  if (chains_.find({parent, child}) == chains_.end()) {
    // Compute chain
    Chain chain;
    auto [parent_frame, parent_inertial] = findSuffix(parent, "_inertial");
    auto [child_frame, child_inertial] = findSuffix(child, "_inertial");
    if (!tree_.getChain(parent_frame, child_frame, chain.kdl_chain)) {
      throw std::invalid_argument(
              "Could not find chain: " + parent + " -> " + child);
    }
    // Identify joints in chain
    for (size_t i = 0; i < chain.kdl_chain.getNrOfSegments(); i++) {
      auto joint = chain.kdl_chain.getSegment(i).getJoint().getName();
      if (std::find(joint_names_.begin(), joint_names_.end(), joint) !=
        joint_names_.end())
      {
        chain.joint_names.push_back(joint);
        chain.joint_indices.push_back(joint_indices_.at(joint));
      }
    }
    // Identify contact frames if present
    if (contact_transforms_.find(parent_frame) != contact_transforms_.end()) {
      chain.contact_indices[parent_frame] =
        -chain.kdl_chain.getNrOfSegments() - (child_inertial ? 1 : 0);
    }
    if (contact_transforms_.find(child_frame) != contact_transforms_.end()) {
      chain.contact_indices[child_frame] =
        chain.kdl_chain.getNrOfSegments() - 1 + (parent_inertial ? 1 : 0);
    }
    // Add parent/child inertial links if requested
    if (parent_inertial) {
      KDL::Frame transform;
      transform.p =
        -tree_.getSegment(parent_frame)->second.segment.getInertia().getCOG();
      KDL::Chain new_chain;
      new_chain.addSegment(
        KDL::Segment(
          parent_frame, KDL::Joint(
            parent_frame + "_joint", KDL::Joint::None), transform));
      new_chain.addChain(chain.kdl_chain);
      chain.kdl_chain = new_chain;
    }
    if (child_inertial) {
      KDL::Frame transform;
      transform.p =
        tree_.getSegment(child_frame)->second.segment.getInertia().getCOG();
      chain.kdl_chain.addSegment(
        KDL::Segment(
          child, KDL::Joint(child + "_joint", KDL::Joint::None), transform));
    }
    // Count joints and segments
    chain.joints = chain.joint_names.size();
    chain.segments = chain.kdl_chain.getNrOfSegments();
    if (chain.joints != chain.kdl_chain.getNrOfJoints()) {
      throw std::invalid_argument(
              "Unexpected joints in chain: " + parent + " -> " + child);
    }
    chains_[{parent, child}] = chain;
  }
  // Update contact frame transforms
  auto & chain = chains_.at({parent, child});
  for (const auto & [frame, index] : chain.contact_indices) {
    if (index >= 0) {
      auto & segment = chain.kdl_chain.getSegment(index);
      segment.setFrameToTip(contact_transforms_.at(frame));
    } else {
      auto & segment = chain.kdl_chain.getSegment(chain.segments + index);
      segment.setFrameToTip(contact_transforms_.at(frame).Inverse());
    }
  }
  return chains_[{parent, child}];
}

std::pair<std::string, bool> KdlInterface::findSuffix(
  const std::string & name, const std::string & suffix)
{
  size_t i = name.find(suffix);
  if (i == name.size() - suffix.size()) {
    return std::make_pair(name.substr(0, i), true);
  }
  return std::make_pair(name, false);
}
