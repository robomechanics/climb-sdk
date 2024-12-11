#include "climb_main/controller/force_controller.hpp"

#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <osqp.h>
#include "climb_main/optimization/osqp_interface.hpp"
#include "climb_main/optimization/qp_problem.hpp"

ForceController::ForceController(std::shared_ptr<KinematicsInterface> robot)
: robot_(robot)
{
  solver_ = std::make_unique<OsqpInterface>();
}

void ForceController::reset()
{
  end_effector_goals_.clear();
  position_cmd_ = robot_->getJointPosition();
}

bool ForceController::update(const Eigen::VectorXd & force)
{
  // VARIABLES
  //   Joint displacement     (n joints)
  //   Body displacement      (p dof)
  //   Tracking error         (m constraints)
  //   Stability margin       (1)

  // CONSTRAINTS
  //   Joint angle            (n)
  //   Joint effort           (n)
  //   Adhesion/tracking      (0-2m)
  //   Equilibrium            (p)
  //   Obstacle               (o)

  using Eigen::MatrixXd, Eigen::VectorXd, Eigen::SparseMatrix;

  // Contact kinematics
  int n = robot_->getNumJoints();           // Number of joints
  int m = robot_->getNumConstraints();      // Number of contact constraints
  int p = 6;                                // Number of body degrees of freedom
  MatrixXd Jh = robot_->getHandJacobian();  // Hand Jacobian (m x n)
  MatrixXd Gs = -robot_->getGraspMap();     // Self-manip grasp map (p x m)
  MatrixXd A(m, n + p);                     // Constraint matrix (m x (n + p))
  A << Jh, -Gs.transpose();
  MatrixXd K =                              // Stiffness matrix (m x m)
    MatrixXd::Identity(m, m) * stiffness_;
  int ind = 0;
  for (const auto & frame : robot_->getContactFrames()) {
    int c = robot_->getNumConstraints(frame);
    if (end_effector_goals_.find(frame) != end_effector_goals_.end() &&
      end_effector_goals_.at(frame).mode == EndEffectorCommand::MODE_FREE)
    {
      K.diagonal().segment(ind, c).setZero();
    }
    ind += c;
  }

  // Joint state vectors
  VectorXd q = robot_->getJointPosition();
  VectorXd qmin = robot_->getJointPositionMin();
  VectorXd qmax = robot_->getJointPositionMax();
  q = q.cwiseMin(qmax).cwiseMax(qmin);
  VectorXd u = robot_->getJointEffort();
  VectorXd umin = robot_->getJointEffortMin();
  VectorXd umax = robot_->getJointEffortMax();
  if (max_effort_ > 0) {
    umax = umax.cwiseMin(max_effort_);
    umin = umin.cwiseMax(-max_effort_);
  }
  if (!configuration_.size()) {
    configuration_ = Eigen::VectorXd::Zero(n);
  }

  // Problem structure
  QpProblem problem({"joint", "body", "error", "margin"}, {n, p, m, 1});
  problem.addAlias("displacement", 0, n + p);

  // Quadratic penalty on distance from home position
  problem.addQuadraticCost("joint", normalization_, configuration_ - q);
  problem.addQuadraticCost("body", normalization_, {});

  // Linear penalty on tracking error
  problem.addLinearCost("error", tracking_);

  // Maximize stability margin
  problem.addLinearCost("margin", -stiffness_);

  // Respect joint limits and maximum displacement per timestep
  // problem.addBounds("joint", qmin - q, qmax - q);
  problem.addLinearConstraint(
    {"joint"}, {MatrixXd::Identity(n, n) * stiffness_},
    (qmin - q) * stiffness_, (qmax - q) * stiffness_);

  // Respect joint effort limits
  problem.addLinearConstraint(
    {"displacement"}, {Jh.transpose() * K * A},
    umin - u, umax - u);

  // Mode-dependent constraints for each contact
  int row = 0;
  for (auto frame : robot_->getContactFrames()) {
    MatrixXd B_i = robot_->getWrenchBasis(frame);
    auto m_i = B_i.cols();
    MatrixXd A_i = A.block(row, 0, m_i, n + p);
    MatrixXd K_i = K.block(row, row, m_i, m_i);
    MatrixXd I_i = MatrixXd::Identity(m, m).block(row, 0, m_i, m);
    VectorXd force_i = force.segment(row, m_i);
    uint8_t mode;
    if (end_effector_goals_.find(frame) != end_effector_goals_.end()) {
      mode = end_effector_goals_[frame].mode;
    } else {
      switch (robot_->getContactType(frame)) {
        case KinematicsInterface::ContactType::TAIL:
          mode = EndEffectorCommand::MODE_CONTACT;
          break;
        default:
          mode = EndEffectorCommand::MODE_STANCE;
          break;
      }
    }
    if (mode == EndEffectorCommand::MODE_FREE) {
      problem.addLinearConstraint(
        // -displacement - error <= -setpoint
        {"displacement", "error"}, {-stiffness_ * A_i, -stiffness_ * I_i},
        {}, -stiffness_ * B_i.transpose() * end_effector_goals_[frame].setpoint);
      problem.addLinearConstraint(
        // displacement - error <= setpoint
        {"displacement", "error"}, {stiffness_ * A_i, -stiffness_ * I_i},
        {}, stiffness_ * B_i.transpose() * end_effector_goals_[frame].setpoint);
    } else if (mode == EndEffectorCommand::MODE_TRANSITION) {
      problem.addLinearConstraint(
        // -displacement - error <= -setpoint
        {"displacement", "error"}, {-K_i * A_i, -K_i * I_i},
        {}, -(B_i.transpose() * end_effector_goals_[frame].setpoint - force_i));
      problem.addLinearConstraint(
        // displacement - error <= setpoint
        {"displacement", "error"}, {K_i * A_i, -K_i * I_i},
        {}, B_i.transpose() * end_effector_goals_[frame].setpoint - force_i);
    } else if (mode == EndEffectorCommand::MODE_CONTACT) {
      auto constraint = getAdhesionConstraint(frame);
      problem.addLinearConstraint(
        // A_c * displacement <= b_c
        {"displacement"},
        {constraint.A * K_i * A_i},
        {}, (constraint.b - constraint.A * force_i));
      problem.addLinearConstraint({"error"}, {I_i}, {});
    } else if (mode == EndEffectorCommand::MODE_STANCE) {
      auto constraint = getAdhesionConstraint(frame);
      problem.addLinearConstraint(
        // A_c * displacement + margin <= b_c
        {"displacement", "margin"},
        {constraint.A * K_i * A_i,
          VectorXd::Constant(constraint.b.size(), stiffness_)},
        {}, (constraint.b - constraint.A * force_i));
      problem.addLinearConstraint({"error"}, {I_i}, {});
    }
    row += m_i;
  }

  // Static equilibrium constraint
  // Mapping from center of mass displacement to body torque
  Eigen::Matrix3d gskew = robot_->getSkew(Gs.topRows(3) * force);
  // Total mass of the robot
  double total_mass = robot_->getMass();
  // Mapping from joint velocity to center of mass velocity in body frame
  MatrixXd J_mass = MatrixXd::Zero(3, n);
  // Mapping from body twist to center of mass velocity in body frame
  MatrixXd G_mass = MatrixXd::Zero(3, p);
  for (auto [link, mass] : robot_->getLinkMasses()) {
    auto frame = link + "_inertial";
    if (mass > mass_threshold_) {
      J_mass += mass / total_mass * robot_->getJacobian(frame, true);
      G_mass.leftCols(3) += mass / total_mass * Eigen::Matrix3d::Identity();
      G_mass.rightCols(3) +=
        mass / total_mass * robot_->getSkew(robot_->getTransform(frame).translation());
    }
  }
  problem.addLinearConstraint(
    {"displacement"}, {Gs.topRows(3) * K * A}, {});
  problem.addLinearConstraint(
    {"displacement", "joint", "body"},
    {Gs.bottomRows(3) * K * A, gskew * J_mass, gskew * G_mass}, {});

  // Obstacle collision constraints
  for (const auto & obstacle : obstacles_) {
    // Mapping from joint velocity to obstacle velocity in body frame
    MatrixXd J_ob = robot_->getJacobian(obstacle.frame, true);
    // Mapping from body twist to obstacle velocity in body frame
    MatrixXd G_ob(3, p);
    G_ob <<
      Eigen::Matrix3d::Identity(),
      robot_->getSkew(robot_->getTransform(obstacle.frame).translation());
    Eigen::RowVector3d normal(obstacle.normal);
    // normal * x <= dist
    problem.addLinearConstraint(
      {"joint", "body"}, {normal * J_ob, normal * G_ob},
      {}, VectorXd::Constant(1, std::max(0.0, obstacle.distance)));
  }

  // Solve QP
  bool success;
  if (solver_->getInitialized()) {
    success = solver_->update(problem);
  } else {
    success = solver_->solve(problem);
  }

  // Store results
  if (success) {
    displacement_cmd_ = solver_->getSolution().head(n + p);
    displacement_cmd_ /= std::max(1.0, displacement_cmd_.cwiseAbs().maxCoeff() / joint_step_);
    displacement_cmd_.head(n) =
      displacement_cmd_.head(n).cwiseMin(qmax - q).cwiseMax(qmin - q);
    margin_ = solver_->getSolution().tail(1)(0) * stiffness_;
    error_ = solver_->getSolution().segment(n + p, m).maxCoeff();
  } else {
    displacement_cmd_ = VectorXd::Zero(n + p);
    margin_ = -INFINITY;
    error_ = INFINITY;
  }
  if (!position_cmd_.size()) {
    position_cmd_ = q;
  }
  position_cmd_ += displacement_cmd_.head(n);
  force_cmd_ = force + K * A * displacement_cmd_;
  effort_cmd_ = u + Jh.transpose() * (force_cmd_ - force);
  return success;
}

void ForceController::setEndEffectorCommand(const EndEffectorCommand & command)
{
  end_effector_goals_.clear();
  for (size_t i = 0; i < command.frame.size(); i++) {
    std::string frame = command.frame[i];
    EndEffectorGoal goal;
    if (i < command.mode.size()) {
      goal.mode = command.mode[i];
    } else {
      goal.mode = EndEffectorCommand::MODE_FREE;
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

void ForceController::setGroundConstraint(
  const Eigen::Vector3d & normal, double distance)
{
  setObstacleConstraints(
    {robot_->getBodyFrame()}, {normal}, {distance - clearance_});
}

void ForceController::setObstacleConstraints(
  const std::vector<std::string> & frames,
  const std::vector<Eigen::Vector3d> & normals,
  const std::vector<double> & distances)
{
  obstacles_.clear();
  for (size_t i = 0; i < frames.size(); i++) {
    if (i >= normals.size() || i >= distances.size()) {
      break;
    }
    obstacles_.push_back({frames[i], normals[i], distances[i]});
  }
}

ForceController::Constraint ForceController::getAdhesionConstraint(const std::string & frame) const
{
  Constraint constraint;
  switch (robot_->getContactType(frame)) {
    case KinematicsInterface::ContactType::MICROSPINE:
      // X = normal force
      // Y = lateral force
      // Z = axial force
      constraint.A = Eigen::MatrixXd(5, 3);
      constraint.A <<
        0, 0, 1,                                 // min axial force
        0, 0, -1,                                // max axial force
        0, -1, tan(microspine_yaw_angle_),       // min lateral force
        0, 1, tan(microspine_yaw_angle_),        // max lateral force
        -1, 0, tan(microspine_pitch_angle_);    // min normal force
      constraint.b = Eigen::Vector<double, 5> {
        -microspine_min_force_, microspine_max_force_, 0, 0, 0};
      break;
    case KinematicsInterface::ContactType::FRICTION:
      constraint.A = Eigen::MatrixXd(5, 3);
      constraint.A <<
        -friction_coefficient_, 0, 1,
        -friction_coefficient_, 0, -1,
        -friction_coefficient_, 1, 0,
        -friction_coefficient_, -1, 0,
        -1, 0, 0;
      constraint.b = Eigen::VectorXd::Zero(5);
      break;
    case KinematicsInterface::ContactType::TAIL:
      constraint.A = Eigen::MatrixXd::Constant(1, 1, -1);
      constraint.b = Eigen::VectorXd::Zero(1);
      break;
    case KinematicsInterface::ContactType::DEFAULT:
      constraint.A.resize(0, 6);
      constraint.b.resize(0);
      break;
    default:
      throw std::invalid_argument("Unsupported contact type");
  }
  return constraint;
}

void ForceController::declareParameters()
{
  declareParameter(
    "stiffness", 1000.0,
    "Expected compliance at the point of contact in N/m", 0.0);
  declareParameter(
    "joint_step", 0.001,
    "Maximum joint displacement in rad or m", 0.0);
  declareParameter(
    "clearance", 0.0,
    "Minimum height of body frame origin above ground plane in m");
  declareParameter(
    "normalization", 0.0001,
    "Cost of joint and body displacement relative to stability margin " \
    "in N/rad and N/m", 0.0);
  declareParameter(
    "tracking", 1.0,
    "Cost of end-effector error relative to stability margin in N/rad", 0.0);
  declareParameter(
    "mass_threshold", 0.0,
    "Minimum link mass for center of mass optimization in kg", 0.0);
  declareParameter(
    "microspine_pitch_angle", 0.35,
    "Maximum out-of-plane loading angle of microspine gripper in rad");
  declareParameter(
    "microspine_yaw_angle", 0.785,
    "Maximum in-plane loading angle of microspine gripper in rad");
  declareParameter(
    "microspine_min_force", 0.0,
    "Minimum tangential force on microspine gripper in N");
  declareParameter(
    "microspine_max_force", 25.0,
    "Maximum tangential force on microspine gripper in N");
  declareParameter(
    "friction_coefficient", 0.5,
    "End effector coefficient of friction", 0.0);
  declareParameter(
    "effort_limit", 0.0,
    "Maximum joint effort in Nm (0 for no limit)", 0.0);
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
  } else if (param.get_name() == "mass_threshold") {
    mass_threshold_ = param.as_double();
  } else if (param.get_name() == "microspine_pitch_angle") {
    microspine_pitch_angle_ = param.as_double();
  } else if (param.get_name() == "microspine_yaw_angle") {
    microspine_yaw_angle_ = param.as_double();
  } else if (param.get_name() == "friction_coefficient") {
    friction_coefficient_ = param.as_double();
  } else if (param.get_name() == "microspine_min_force") {
    microspine_min_force_ = param.as_double();
  } else if (param.get_name() == "microspine_max_force") {
    microspine_max_force_ = param.as_double();
  } else if (param.get_name() == "effort_limit") {
    max_effort_ = param.as_double();
  }
  solver_->setParameter(param, result);
}
