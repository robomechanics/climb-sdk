#include "climb_main/controller/force_controller.hpp"

#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <osqp.h>

ForceController::ForceController(std::shared_ptr<KinematicsInterface> robot)
: robot_(robot)
{
}

Eigen::VectorXd ForceController::update(const Eigen::VectorXd & force)
{
  // Contact kinematics
  int n = robot_->getNumJoints();
  int m = robot_->getNumConstraints();
  int p = 6;
  Eigen::MatrixXd J = robot_->getHandJacobian();
  Eigen::MatrixXd G = robot_->getGraspMap();
  Eigen::MatrixXd A(m, p + n);
  A << G.transpose(), J;
  Eigen::MatrixXd K = Eigen::MatrixXd::Identity(m, m) * stiffness_;

  // Joint state
  Eigen::VectorXd q = robot_->getJointPosition();
  Eigen::VectorXd q0 = Eigen::VectorXd::Zero(n);

  // Quadratic cost
  Eigen::SparseMatrix<double> H(p + n + 1, p + n + 1);
  H.setIdentity();
  H *= normalization_;
  H.coeffRef(p + n, p + n) = 0;

  // Linear cost
  Eigen::VectorXd f(p + n + 1);
  f << Eigen::VectorXd::Zero(p), (q - q0) * normalization_ * 2, -1;

  // Constraint matrix

  return Eigen::VectorXd();
}

void ForceController::setEndEffectorCommand(const EndEffectorCommand & command)
{
  end_effector_goals_.clear();
  for (size_t i = 0; i < command.frame.size(); i++) {
    std::string frame = command.frame[i];
    EndEffectorGoal goal;
    if (i < command.frame.size()) {
      goal.mode = command.mode[i];
    }
    switch (goal.mode) {
      case EndEffectorCommand::MODE_FREE:
        if (i < command.twist.size()) {
          goal.setpoint <<
            command.twist[i].linear.x,
            command.twist[i].linear.y,
            command.twist[i].linear.z,
            command.twist[i].angular.x,
            command.twist[i].angular.y,
            command.twist[i].angular.z;
        } else {
          goal.setpoint = Eigen::VectorXd::Zero(6);
        }
        break;
      case EndEffectorCommand::MODE_TRANSITION:
        if (i < command.wrench.size()) {
          goal.setpoint <<
            command.wrench[i].force.x,
            command.wrench[i].force.y,
            command.wrench[i].force.z,
            command.wrench[i].torque.x,
            command.wrench[i].torque.y,
            command.wrench[i].torque.z;
        } else {
          goal.setpoint = Eigen::VectorXd::Zero(6);
        }
        break;
      default:
        goal.setpoint = Eigen::VectorXd::Zero(6);
        break;
    }
    end_effector_goals_[frame] = goal;
  }
}

void ForceController::setObstacleConstraints(
  const std::vector<std::string> & frames,
  const std::vector<Eigen::Vector3d> & displacements)
{
  obstacles_.clear();
  for (size_t i = 0; i < frames.size(); i++) {
    if (i >= displacements.size()) {
      break;
    }
    ObstacleConstraint constraint;
    constraint.frame = frames[i];
    constraint.displacement = displacements[i];
    obstacles_.push_back(constraint);
  }
}

void ForceController::declareParameters()
{
  declareParameter(
    "stiffness", 1.0,
    "Expected compliance at the point of contact in N/m", 0.0);
  declareParameter(
    "joint_step", 0.01,
    "Maximum joint displacement in rad or m", 0.0);
  declareParameter(
    "clearance", 0.0,
    "Minimum height of body frame origin above ground plane in m");
  declareParameter(
    "normalization", 0.01,
    "Cost of joint displacement and body twist relative to stability margin " \
    "in N/rad and N/m", 0.0);
}

void ForceController::setParameter(const Parameter & param, SetParametersResult & result)
{
  if (param.get_name() == "stiffness") {
    stiffness_ = param.as_double();
  } else if (param.get_name() == "joint_step") {
    joint_step_ = param.as_double();
  } else if (param.get_name() == "clearance") {
    clearance_ = param.as_double();
  } else if (param.get_name() == "normalization") {
    normalization_ = param.as_double();
  }
}
