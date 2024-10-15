#include "climb_main/kinematics/kinematics_interface.hpp"

KinematicsInterface::KinematicsInterface()
{
}

bool KinematicsInterface::loadRobotDescription(std::string description)
{
  initialized_ = false;
  return urdf_model_.initString(description);
}

bool KinematicsInterface::initialize(std::string & error_message)
{
  initialized_ = false;

  // Check that robot description has been loaded
  if (!urdf_model_.getRoot()) {
    error_message = "Robot description not loaded";
    return false;
  }

  // Check that requested frames and joints are present in robot description
  if (!urdf_model_.getLink(body_frame_)) {
    error_message =
      "Body frame " + body_frame_ + " not found in robot description";
    return false;
  }
  for (const auto & frame : contact_frames_) {
    if (!urdf_model_.getLink(frame)) {
      error_message =
        "Contact frame " + frame + " not found in robot description";
      return false;
    }
  }
  for (const auto & frame : end_effector_frames_) {
    if (!urdf_model_.getLink(frame)) {
      error_message =
        "End effector frame " + frame + " not found in robot description";
      return false;
    }
  }
  for (const auto & joint : joint_names_) {
    if (!urdf_model_.getJoint(joint)) {
      error_message = "Joint " + joint + " not found in robot description";
      return false;
    }
    if (urdf_model_.getJoint(joint)->type != urdf::Joint::REVOLUTE &&
      urdf_model_.getJoint(joint)->type != urdf::Joint::CONTINUOUS &&
      urdf_model_.getJoint(joint)->type != urdf::Joint::PRISMATIC)
    {
      error_message = "Joint " + joint + " type is not supported";
      return false;
    }
  }

  // Check that parameter lengths match
  if (contact_frames_.size() != end_effector_frames_.size()) {
    error_message =
      "Contact frames and end effector frames must be same length";
    return false;
  }
  if (contact_frames_.size() != contact_types_.size()) {
    error_message = "Contact frames and contact types must be same length";
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

  // Initialize joint state and limits
  num_joints_ = joint_names_.size();
  joint_pos_ = Eigen::VectorXd::Zero(num_joints_);
  joint_vel_ = Eigen::VectorXd::Zero(num_joints_);
  joint_eff_ = Eigen::VectorXd::Zero(num_joints_);
  joint_pos_min_.resize(num_joints_);
  joint_pos_max_.resize(num_joints_);
  joint_vel_min_.resize(num_joints_);
  joint_vel_max_.resize(num_joints_);
  joint_eff_min_.resize(num_joints_);
  joint_eff_max_.resize(num_joints_);
  for (int i = 0; i < num_joints_; i++) {
    auto joint = urdf_model_.getJoint(joint_names_[i]);
    joint_pos_min_[i] = joint->limits->lower;
    joint_pos_max_[i] = joint->limits->upper;
    if (joint->type == urdf::Joint::CONTINUOUS) {
      joint_pos_min_[i] = -std::numeric_limits<double>::infinity();
      joint_pos_max_[i] = std::numeric_limits<double>::infinity();
    }
    joint_vel_min_[i] = -joint->limits->velocity;
    joint_vel_max_[i] = joint->limits->velocity;
    joint_eff_min_[i] = -joint->limits->effort;
    joint_eff_max_[i] = joint->limits->effort;
  }

  // Compute total mass
  mass_ = 0;
  for (const auto & link_pair : urdf_model_.links_) {
    if (link_pair.second->inertial) {
      mass_ += link_pair.second->inertial->mass;
    }
  }

  // Initialize wrench bases
  num_constraints_ = 0;
  wrench_bases_.clear();
  for (size_t i = 0; i < contact_frames_.size(); i++) {
    auto contact = contact_frames_[i];
    auto basis = WRENCH_BASES.at(contact_types_[i]);
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

Eigen::Matrix3d KinematicsInterface::getSkew(const Eigen::Vector3d & vector)
{
  Eigen::Matrix3d skew;
  skew << 0, -vector(2), vector(1),
    vector(2), 0, -vector(0),
    -vector(1), vector(0), 0;
  return skew;
}

Eigen::Matrix<double, 6, 6> KinematicsInterface::getAdjoint(
  const Eigen::Vector3d & position,
  const Eigen::Matrix3d & rotation)
{
  Eigen::Matrix<double, 6, 6> adjoint;
  adjoint.block<3, 3>(0, 0) = rotation;
  adjoint.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero();
  adjoint.block<3, 3>(0, 3) = getSkew(position) * rotation;
  adjoint.block<3, 3>(3, 3) = rotation;
  return adjoint;
}

Eigen::MatrixXd KinematicsInterface::getMixedJacobian(bool linear)
{
  int p = linear ? 3 : 6;
  Eigen::MatrixXd jac(p * num_contacts_, num_joints_);
  for (int i = 0; i < num_contacts_; i++) {
    Eigen::MatrixXd jac_i = getMixedJacobian(contact_frames_[i], linear);
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
  for (int i = 0; i < num_contacts_; i++) {
    Eigen::MatrixXd jac_i = getHandJacobian(contact_frames_[i]);
    if (jac_i.size() == 0) {
      return Eigen::MatrixXd();
    }
    jac.block(row, 0, jac_i.rows(), num_joints_) = jac_i;
    row += jac_i.rows();
  }
  return jac;
}

void KinematicsInterface::updateJointState(const JointState & state)
{
  for (size_t i = 0; i < state.name.size(); i++) {
    auto j = std::find(joint_names_.begin(), joint_names_.end(), state.name[i]);
    if (j != joint_names_.end()) {
      const size_t index = std::distance(joint_names_.begin(), j);
      if (state.position.size() > i) {
        joint_pos_[index] = state.position[i];
      }
      if (state.velocity.size() > i) {
        joint_vel_[index] = state.velocity[i];
      }
      if (state.effort.size() > i) {
        joint_eff_[index] = state.effort[i];
      }
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
    "Type of each end-effector");
  declareParameter(
    "joint_names", std::vector<std::string>(),
    "Names of each joint in preferred order");
}

void KinematicsInterface::setParameter(
  const Parameter & param, SetParametersResult & result)
{
  if (param.get_name() == "body_frame") {
    body_frame_ = param.as_string();
    if (initialized_) {
      std::string error_message;
      if (!initialize(error_message)) {
        result.successful = false;
        result.reason = error_message;
        return;
      }
    }
  } else if (param.get_name() == "end_effector_frames") {
    end_effector_frames_ = param.as_string_array();
    if (initialized_) {
      std::string error_message;
      if (!initialize(error_message)) {
        result.successful = false;
        result.reason = error_message;
        return;
      }
    }
  } else if (param.get_name() == "contact_frames") {
    contact_frames_ = param.as_string_array();
    num_contacts_ = contact_frames_.size();
    if (initialized_) {
      std::string error_message;
      if (!initialize(error_message)) {
        result.successful = false;
        result.reason = error_message;
        return;
      }
    }
  } else if (param.get_name() == "joint_names") {
    auto original = joint_names_;
    joint_names_.clear();
    for (auto name : param.as_string_array()) {
      if (std::find(joint_names_.begin(), joint_names_.end(), name) ==
        joint_names_.end())
      {
        joint_names_.push_back(name);
      }
    }
    num_joints_ = joint_names_.size();
    if (initialized_) {
      std::string error_message;
      if (!initialize(error_message)) {
        result.successful = false;
        result.reason = error_message;
        joint_names_ = original;
        return;
      }
    }
  } else if (param.get_name() == "contact_types") {
    std::vector<ContactType> contact_types;
    for (auto name : param.as_string_array()) {
      if (name == "microspine") {
        contact_types.push_back(ContactType::MICROSPINE);
      } else if (name == "tail") {
        contact_types.push_back(ContactType::TAIL);
      } else if (name == "magnetic wheel" || name == "magnetic_wheel") {
        contact_types.push_back(ContactType::MAGNET_WHEEL);
      } else if (name == "default") {
        contact_types.push_back(ContactType::DEFAULT);
      } else {
        result.successful = false;
        result.reason = "Invalid contact type: " + name;
        return;
      }
    }
    contact_types_ = contact_types;
    if (initialized_) {
      std::string error_message;
      if (!initialize(error_message)) {
        result.successful = false;
        result.reason = error_message;
        return;
      }
    }
  }
}
