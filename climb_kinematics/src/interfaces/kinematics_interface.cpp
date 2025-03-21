#include "climb_kinematics/interfaces/kinematics_interface.hpp"

KinematicsInterface::KinematicsInterface() {}

bool KinematicsInterface::loadRobotDescription(
  const std::string & description, std::string & error_message)
{
  initialized_ = false;
  if (!urdf_model_.initString(description)) {
    error_message = "Failed to parse URDF";
    return false;
  }
  return initialize(error_message);
}

bool KinematicsInterface::initialize(std::string & error_message)
{
  initialized_ = false;

  // Check that robot description has been loaded
  if (!urdf_model_.getRoot()) {
    error_message = "Robot description not yet loaded";
    return false;
  }

  // Check for parameter length mismatch
  if (num_contacts_ == 0) {
    error_message = "Contact frames not set";
    return false;
  }
  if (end_effector_frames_.size() != num_contacts_) {
    error_message =
      "End-effector frames and contact frames must have same length";
    return false;
  }
  if (contact_types_.size() != num_contacts_) {
    error_message = "Contact types and contact frames must have same length";
    return false;
  }
  if (wrist_types_.size() != num_contacts_) {
    error_message = "Wrist types and contact frames must have same length";
    return false;
  }

  // Validate parameters (they may have been set before URDF model was loaded)
  SetParametersResult result;
  result.successful = true;
  setParameter(Parameter("body_frame", body_frame_), result);
  setParameter(Parameter("end_effector_frames", end_effector_frames_), result);
  setParameter(Parameter("joint_names", joint_names_), result);
  if (!result.successful) {
    error_message = result.reason;
    return false;
  }

  // Load any additional joint names not specified in parameters
  for (const auto & joint : urdf_model_.joints_) {
    if (joint.second->type == urdf::Joint::REVOLUTE ||
      joint.second->type == urdf::Joint::CONTINUOUS ||
      joint.second->type == urdf::Joint::PRISMATIC)
    {
      if (std::find(joint_names_.begin(), joint_names_.end(), joint.first) ==
        joint_names_.end())
      {
        joint_names_.push_back(joint.first);
      }
    }
  }
  num_joints_ = joint_names_.size();

  // Initialize joint state and limits
  joint_pos_ = Eigen::VectorXd::Zero(num_joints_);
  joint_vel_ = Eigen::VectorXd::Zero(num_joints_);
  joint_eff_ = Eigen::VectorXd::Zero(num_joints_);
  joint_pos_min_.resize(num_joints_);
  joint_pos_max_.resize(num_joints_);
  joint_vel_min_.resize(num_joints_);
  joint_vel_max_.resize(num_joints_);
  joint_eff_min_.resize(num_joints_);
  joint_eff_max_.resize(num_joints_);
  for (size_t i = 0; i < num_joints_; i++) {
    auto joint = urdf_model_.getJoint(joint_names_[i]);
    if (joint->type == urdf::Joint::CONTINUOUS) {
      joint_pos_min_[i] = -INFINITY;
      joint_pos_max_[i] = INFINITY;
    } else {
      joint_pos_min_[i] = joint->limits->lower;
      joint_pos_max_[i] = joint->limits->upper;
    }
    if (joint->limits != nullptr) {
      joint_vel_min_[i] = -joint->limits->velocity;
      joint_vel_max_[i] = joint->limits->velocity;
      joint_eff_min_[i] = -joint->limits->effort;
      joint_eff_max_[i] = joint->limits->effort;
    } else {
      joint_vel_min_[i] = -INFINITY;
      joint_vel_max_[i] = INFINITY;
      joint_eff_min_[i] = -INFINITY;
      joint_eff_max_[i] = INFINITY;
    }
  }

  // Compute mass properties
  mass_ = 0;
  link_masses_.clear();
  for (const auto & link_pair : urdf_model_.links_) {
    if (link_pair.second->inertial) {
      mass_ += link_pair.second->inertial->mass;
      link_masses_[link_pair.first] = link_pair.second->inertial->mass;
    }
  }

  // Initialize wrench bases
  num_constraints_ = 0;
  wrench_bases_.clear();
  for (size_t index = 0; index < num_contacts_; index++) {
    auto contact = contact_frames_[index];
    auto basis = WRENCH_BASES.at(contact_types_[index]);
    int c = 6 - std::count(basis.begin(), basis.end(), 0);
    wrench_bases_[contact] = Eigen::MatrixXd::Zero(6, c);
    int j = 0;
    for (int i = 0; i < 6; i++) {
      if (basis[i] != 0) {
        wrench_bases_[contact](i, j) = 1;
        j++;
      }
    }
    num_constraints_ += c;
  }

  return initialized_ = true;
}

Eigen::Matrix3d KinematicsInterface::getSkew(
  const Eigen::Vector3d & vector) const
{
  Eigen::Matrix3d skew;
  skew << 0, -vector(2), vector(1),
    vector(2), 0, -vector(0),
    -vector(1), vector(0), 0;
  return skew;
}

Eigen::Matrix<double, 6, 6> KinematicsInterface::getAdjoint(
  const Eigen::Isometry3d & transform)
{
  Eigen::Matrix<double, 6, 6> adjoint;
  adjoint.block<3, 3>(0, 0) = transform.rotation();
  adjoint.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero();
  adjoint.block<3, 3>(0, 3) =
    getSkew(transform.translation()) * adjoint.block<3, 3>(0, 0);
  adjoint.block<3, 3>(3, 3) = adjoint.block<3, 3>(0, 0);
  return adjoint;
}

Eigen::MatrixXd KinematicsInterface::getJacobian(bool linear)
{
  int p = linear ? 3 : 6;
  Eigen::MatrixXd jac(p * num_contacts_, num_joints_);
  for (size_t i = 0; i < num_contacts_; i++) {
    Eigen::MatrixXd jac_i = getJacobian(contact_frames_[i], linear);
    if (jac_i.size() == 0) {
      return Eigen::MatrixXd();
    }
    jac.block(i * p, 0, p, num_joints_) = jac_i;
  }
  return jac;
}

Eigen::MatrixXd KinematicsInterface::getHandJacobian()
{
  Eigen::MatrixXd jac(num_constraints_, num_joints_);
  int row = 0;
  for (size_t i = 0; i < num_contacts_; i++) {
    Eigen::MatrixXd jac_i = getHandJacobian(contact_frames_[i]);
    if (jac_i.size() == 0) {
      return Eigen::MatrixXd();
    }
    jac.block(row, 0, jac_i.rows(), num_joints_) = jac_i;
    row += jac_i.rows();
  }
  return jac;
}

Eigen::Matrix<double, 6, Eigen::Dynamic> KinematicsInterface::getGraspMap()
{
  Eigen::MatrixXd grasp(6, num_constraints_);
  int col = 0;
  for (const auto & contact : contact_frames_) {
    Eigen::MatrixXd block =
      getAdjoint(contact, body_frame_).transpose() * wrench_bases_[contact];
    grasp.block(0, col, 6, block.cols()) = block;
    col += block.cols();
  }
  return grasp;
}

void KinematicsInterface::updateJointState(const JointState & state)
{
  for (size_t i = 0; i < state.name.size(); i++) {
    auto j = getJointIndex(state.name[i]);
    if (j == -1) {
      continue;
    }
    if (state.position.size() > i) {
      joint_pos_[j] = state.position[i];
    }
    if (state.velocity.size() > i) {
      joint_vel_[j] = state.velocity[i];
    }
    if (state.effort.size() > i) {
      joint_eff_[j] = state.effort[i];
    }
  }
}

int KinematicsInterface::getContactIndex(const std::string & contact) const
{
  auto c = std::find(contact_frames_.begin(), contact_frames_.end(), contact);
  if (c == contact_frames_.end()) {
    throw std::invalid_argument("Contact frame " + contact + " not found");
  }
  return std::distance(contact_frames_.begin(), c);
}

int KinematicsInterface::getJointIndex(const std::string & joint) const
{
  auto j = std::find(joint_names_.begin(), joint_names_.end(), joint);
  if (j == joint_names_.end()) {
    throw std::invalid_argument("Joint " + joint + " not found");
  }
  return std::distance(joint_names_.begin(), j);
}

void KinematicsInterface::clampJointCommand(JointCommand & command) const
{
  for (size_t i = 0; i < command.name.size(); i++) {
    auto j = getJointIndex(command.name[i]);
    if (command.position.size() > i) {
      command.position[i] = std::max(
        joint_pos_min_[j], std::min(joint_pos_max_[j], command.position[i]));
    }
    if (command.velocity.size() > i) {
      command.velocity[i] = std::max(
        joint_vel_min_[j], std::min(joint_vel_max_[j], command.velocity[i]));
    }
    if (command.effort.size() > i) {
      command.effort[i] = std::max(
        joint_eff_min_[j], std::min(joint_eff_max_[j], command.effort[i]));
    }
  }
}

void KinematicsInterface::declareParameters()
{
  declareParameter("body_frame", "base_link", "Name of the body frame");
  declareParameter(
    "end_effector_frames", std::vector<std::string>(),
    "Name of each end-effector frame");
  declareParameter(
    "contact_frames", std::vector<std::string>(),
    "Name of each contact frame");
  declareParameter(
    "contact_types", std::vector<std::string>(),
    "Type of each end-effector contact");
  declareParameter(
    "wrist_types", std::vector<std::string>(),
    "Type of each end-effector wrist");
  declareParameter(
    "joint_names", std::vector<std::string>(),
    "Names of each joint in preferred order");
}

void KinematicsInterface::setParameter(
  const Parameter & param, SetParametersResult & result)
{
  if (param.get_name() == "body_frame") {
    // Validate parameter
    if (urdf_model_.getRoot()) {
      if (!urdf_model_.getLink(param.as_string())) {
        result.successful = false;
        result.reason =
          "Body frame " + body_frame_ + " not found in robot description";
        return;
      }
    }
    // Update value and attempt to initialize if parameter has changed
    if (body_frame_ != param.as_string()) {
      body_frame_ = param.as_string();
      initialize(result.reason);
    }
  } else if (param.get_name() == "end_effector_frames") {
    // Validate parameter
    if (urdf_model_.getRoot()) {
      for (const auto & frame : param.as_string_array()) {
        if (!urdf_model_.getLink(frame)) {
          result.successful = false;
          result.reason =
            "End effector frame " + frame + " not found in robot description";
          return;
        }
      }
    }
    // Update value and attempt to initialize if parameter has changed
    if (end_effector_frames_ != param.as_string_array()) {
      end_effector_frames_ = param.as_string_array();
      initialize(result.reason);
    }
  } else if (param.get_name() == "contact_frames") {
    // Update value and attempt to initialize if parameter has changed
    if (contact_frames_ != param.as_string_array()) {
      contact_frames_ = param.as_string_array();
      num_contacts_ = contact_frames_.size();
      initialize(result.reason);
    }
  } else if (param.get_name() == "joint_names") {
    // Validate parameter
    if (urdf_model_.getRoot()) {
      for (const auto & joint : param.as_string_array()) {
        if (!urdf_model_.getJoint(joint)) {
          result.successful = false;
          result.reason = "Joint " + joint + " not found in robot description";
          return;
        }
        if (urdf_model_.getJoint(joint)->type != urdf::Joint::REVOLUTE &&
          urdf_model_.getJoint(joint)->type != urdf::Joint::CONTINUOUS &&
          urdf_model_.getJoint(joint)->type != urdf::Joint::PRISMATIC)
        {
          result.successful = false;
          result.reason = "Joint " + joint + " type is not supported";
          return;
        }
      }
    }
    // Update value and attempt to initialize if parameter has changed
    if (joint_names_ != param.as_string_array()) {
      joint_names_.clear();
      for (const auto & name : param.as_string_array()) {
        if (std::find(joint_names_.begin(), joint_names_.end(), name) ==
          joint_names_.end())
        {
          joint_names_.push_back(name);
        }
      }
      num_joints_ = joint_names_.size();
      initialize(result.reason);
    }
  } else if (param.get_name() == "contact_types") {
    // Validate parameter
    std::vector<ContactType> contact_types;
    for (const auto & name : param.as_string_array()) {
      if (name == "microspine") {
        contact_types.push_back(ContactType::MICROSPINE);
      } else if (name == "tail") {
        contact_types.push_back(ContactType::TAIL);
      } else if (name == "magnetic wheel" || name == "magnetic_wheel") {
        contact_types.push_back(ContactType::MAGNET_WHEEL);
      } else if (name == "friction") {
        contact_types.push_back(ContactType::FRICTION);
      } else if (name == "default") {
        contact_types.push_back(ContactType::DEFAULT);
      } else {
        result.successful = false;
        result.reason = "Invalid contact type: " + name;
        return;
      }
    }
    // Update value and attempt to initialize if parameter has changed
    if (contact_types_ != contact_types) {
      contact_types_ = contact_types;
      initialize(result.reason);
    }
  } else if (param.get_name() == "wrist_types") {
    // Validate parameter
    std::vector<WristType> wrist_types;
    for (const auto & name : param.as_string_array()) {
      if (name == "fixed") {
        wrist_types.push_back(WristType::FIXED);
      } else if (name == "free") {
        wrist_types.push_back(WristType::FREE);
      } else if (name == "gravity") {
        wrist_types.push_back(WristType::GRAVITY);
      } else if (name == "spring") {
        wrist_types.push_back(WristType::SPRING);
      } else {
        result.successful = false;
        result.reason = "Invalid wrist type: " + name;
        return;
      }
    }
    // Update value and attempt to initialize if parameter has changed
    if (wrist_types_ != wrist_types) {
      wrist_types_ = wrist_types;
      initialize(result.reason);
    }
  }
}
