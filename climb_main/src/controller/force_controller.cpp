#include "climb_main/controller/force_controller.hpp"

#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <osqp.h>
#include "climb_main/optimization/osqp_interface.hpp"

ForceController::ForceController(std::shared_ptr<KinematicsInterface> robot)
: robot_(robot)
{
  solver_ = std::make_unique<OsqpInterface>();
}

Eigen::VectorXd ForceController::update(const Eigen::VectorXd & force)
{
  // VARIABLES
  //   Joint displacement     (n joints)
  //   Body displacement      (p dof)
  //   Tracking error         (m constraints)
  //   Stability margin       (1)

  // CONSTRAINTS
  //   Joint angle            (n)
  //   Joint effort           (n)
  //   Adhesion/tracking      (2m)
  //   Equilibrium            (p)
  //   Obstacle               (1)

  using Eigen::MatrixXd, Eigen::VectorXd, Eigen::SparseMatrix;

  // Problem dimensions
  int n = robot_->getNumJoints();       // Number of joints
  int m = robot_->getNumConstraints();  // Number of contact constraints
  int p = 6;                            // Number of body degrees of freedom
  int N = n + p + m + 1;                // Number of optimization variables
  int M = 2 * n + 2 * m + p + 1;        // Number of optimization constraints

  // Contact kinematics
  MatrixXd J = robot_->getHandJacobian();   // Hand Jacobian (m x n)
  MatrixXd G = robot_->getGraspMap();       // Grasp map (p x m)
  MatrixXd A(m, n + p);                     // Constraint matrix (m x (n + p))
  A << J, G.transpose();
  std::cout << "Ac\n" << A << std::endl;
  MatrixXd K =                              // Stiffness matrix (m x m)
    MatrixXd::Identity(m, m) * stiffness_;
  std::cout << "K\n" << K << std::endl;

  // Joint state vectors (n)
  VectorXd q = robot_->getJointPosition();
  VectorXd q0 = VectorXd::Zero(n);
  VectorXd qmin = robot_->getJointPositionMin();
  VectorXd qmax = robot_->getJointPositionMax();
  VectorXd umin = robot_->getJointEffortMin();
  VectorXd umax = robot_->getJointEffortMax();

  // Quadratic cost matrix (N x N)
  VectorXd Hvec = VectorXd::Ones(N) * normalization_;
  Hvec.tail(1).setZero();
  MatrixXd H = Hvec.asDiagonal();
  std::cout << "H\n" << H << std::endl;

  // Linear cost vector (N)
  VectorXd f(N);
  f << (q - q0) * normalization_,             // Joint home position offset
    VectorXd::Zero(p),
    VectorXd::Constant(m, tracking_),         // End-effector tracking
    -1;                                       // Stability margin
  std::cout << "f\n" << f << std::endl;

  // Joint position constraint matrix (n x n)
  MatrixXd A_joint = MatrixXd::Identity(n, n);
  std::cout << "A_joint\n" << A_joint << std::endl;

  // Joint effort constraint matrix ((n + p) x n)
  MatrixXd A_effort = J.transpose() * K * A;
  std::cout << "A_effort\n" << A_effort << std::endl;

  // End effector constraint matrix (2m x (n + p + m + 1))
  MatrixXd A_end_effector(2 * m, N);
  A_end_effector <<
    -K * A, -MatrixXd::Identity(m, m) * tracking_, MatrixXd::Ones(m, 1),
    K * A, -MatrixXd::Identity(m, m) * tracking_, MatrixXd::Ones(m, 1);
  size_t row = 0;
  for (auto frame : robot_->getContactFrames()) {
    auto m_i = robot_->getWrenchBasis(frame).cols();  // Number of constraints
    switch (end_effector_goals_[frame].mode) {
      case EndEffectorCommand::MODE_FREE:
      case EndEffectorCommand::MODE_TRANSITION:
        A_end_effector.block(row, n + p + m, m_i, 1).setZero();
        A_end_effector.block(row + m, n + p + m, m_i, 1).setZero();
        break;
      default:
        // Ac_lower = robot_->getForceConstraintLower(frame);
        A_end_effector.block(row, n + p, m_i, m).setZero();
        A_end_effector.block(row + m, n + p, m_i, m).setZero();
        // A_end_effector.block(row, 0, m_i, n + p) =
        //   Ac_lower * A_end_effector.block(row, 0, m_i, n + p);
        // A_end_effector.block(row, 0, m_i, n + p) =
        //   Ac_upper * A_end_effector.block(row + m, 0, m_i, n + p);
        break;
    }
    row += m_i;
  }
  std::cout << "A_end_effector\n" << A_end_effector << std::endl;

  // Equilibrium constraint matrix (p x (n + p))
  MatrixXd A_equilibrium = G * K * A;
  std::cout << "A_equilibrium\n" << A_equilibrium << std::endl;

  // Obstacle constraint matrix (1 x (n + p))
  MatrixXd A_obstacle = MatrixXd::Zero(1, n + p);
  std::cout << "A_obstacle\n" << A_obstacle << std::endl;

  // Full constraint matrix (M x N)
  MatrixXd A_full(M, N);
  A_full <<
    A_joint, MatrixXd::Zero(n, M - n),                // Joint angle
    A_effort, MatrixXd::Zero(n, m + 1),               // Joint effort
    A_end_effector,                                   // End effector
    A_equilibrium, MatrixXd::Zero(p, m + 1),          // Equilibrium
    A_obstacle, MatrixXd::Zero(1, m + 1);             // Obstacle
  std::cout << "A_full\n" << A_full << std::endl;

  // Constraint bounds
  VectorXd lb = VectorXd::Constant(M, -INFINITY);
  std::cout << "lb\n" << lb << std::endl;
  VectorXd ub = VectorXd::Constant(M, INFINITY);
  std::cout << "ub\n" << ub << std::endl;

  // Solve QP (TODO: apply sparsity structure)
  bool success;
  if (solver_->getInitialized()) {
    success = solver_->update(H, f, A_full, lb, ub);
  } else {
    success = solver_->solve(H, f, A_full, lb, ub);
  }
  if (success) {
    return solver_->getSolution().head(n);
  }
  return VectorXd();
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
    obstacles_.push_back({frames[i], displacements[i]});
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
    "Cost of joint and body displacement relative to stability margin " \
    "in 1/N^2", 0.0);
  declareParameter(
    "tracking", 0.01,
    "Cost of end-effector error relative to stability margin in 1/N", 0.0);
  for (const auto & param : solver_->getParameters()) {
    parameters_.push_back(param);
  }
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
  } else if (param.get_name() == "tracking") {
    tracking_ = param.as_double();
  }
  solver_->setParameter(param, result);
}
