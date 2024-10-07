#include "climb_main/kinematics/kinematics_interface.hpp"
#include <urdf/model.h>

KinematicsInterface::KinematicsInterface()
{
}

bool KinematicsInterface::loadRobotDescription(std::string description)
{
  urdf::Model urdf_model;
  initialized_ = urdf_model.initString(description);
  if (!initialized_) {
    return initialized_;
  }
  for (const auto & joint : urdf_model.joints_) {
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
    auto joint = urdf_model.joints_[joint_names_[i]];
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
  return initialized_;
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

void KinematicsInterface::updateBases()
{
  num_constraints_ = 0;
  wrench_bases_.clear();
  for (size_t i = 0; i < contact_frames_.size(); i++) {
    auto contact = contact_frames_[i];
    Eigen::MatrixXd basis;
    if (i >= contact_types_.size()) {
      basis = Eigen::MatrixXd::Identity(6, 6);
    } else if (contact_types_[i] == ContactType::MICROSPINE) {
      basis = (Eigen::MatrixXd(6, 3) <<
        Eigen::MatrixXd::Identity(3, 3),
        Eigen::MatrixXd::Zero(3, 3)).finished();
    } else {
      basis = Eigen::MatrixXd::Identity(6, 6);
    }
    wrench_bases_[contact] = basis;
    num_constraints_ += basis.cols();
  }
}
