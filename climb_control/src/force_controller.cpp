#include "climb_control/force_controller.hpp"

#include <climb_optimization/qp_interfaces/osqp_interface.hpp>
#include <climb_optimization/qp_problem.hpp>
#include <climb_util/eigen_utils.hpp>
#include <climb_util/ros_utils.hpp>

ForceController::ForceController(std::shared_ptr<KinematicsInterface> robot)
: robot_(robot)
{
  solver_ = std::make_unique<OsqpInterface>();
}

void ForceController::reset()
{
  end_effector_goals_.clear();
  for (auto frame : robot_->getContactFrames()) {
    if (robot_->getContactType(frame) == ContactType::TAIL) {
      end_effector_goals_[frame] = {EndEffectorCommand::MODE_CONTACT, {}};
    } else {
      end_effector_goals_[frame] = {EndEffectorCommand::MODE_STANCE, {}};
    }
  }
  position_cmd_ = robot_->getJointPosition();
}

bool ForceController::update(
  const Eigen::VectorXd & force, const Eigen::Isometry3d & pose)
{
  using Eigen::MatrixXd, Eigen::VectorXd;
  if (end_effector_goals_.empty()) {
    reset();
  }
  int n = robot_->getNumJoints();               // Number of joints
  int m = robot_->getNumConstraints();          // Number of contact constraints
  MatrixXd Gs = -robot_->getGraspMap();         // Self-manip grasp map (p x m)
  MatrixXd Jh = robot_->getHandJacobian();      // Hand Jacobian (m x n)
  Eigen::Vector<double, 6> fext = -Gs * force;  // External wrench (6 x 1)
  Eigen::MatrixXd N =                           // Gravity torque offset (n x 1)
    robot_->getGravitationalVector(fext.head(3) / robot_->getMass());
  VectorXd twist = EigenUtils::getTwist(pose);  // Nominal body twist (6 x 1)

  QpProblem problem({"force", "com", "margin"}, {m, 3, 1});

  // Cost function
  problem.addQuadraticCost("force", joint_normalization_, {});
  problem.addQuadraticCost("com", body_normalization_, twist.head(3));
  problem.addLinearCost("margin", -1);

  // Static equilibrium constraint
  problem.addEqualityConstraint(
    {"force"}, {Gs.topRows(3)}, -fext.head(3));
  problem.addEqualityConstraint(
    {"force", "com"},
    {Gs.bottomRows(3), EigenUtils::getSkew(-fext.head(3))},
    -fext.tail(3));

  // Joint torque limits
  problem.addInequalityConstraint(
    {"force"}, {Jh.transpose()},
    robot_->getJointEffortMin().cwiseMax(-max_effort_) - N,
    robot_->getJointEffortMax().cwiseMin(max_effort_) - N);

  // Contact force constraints
  int row = 0;
  for (auto frame : robot_->getContactFrames()) {
    auto mode = end_effector_goals_.at(frame).mode;
    auto m_i = robot_->getNumConstraints(frame);
    MatrixXd I_i = MatrixXd::Identity(m, m).block(row, 0, m_i, m);
    auto constraint = getAdhesionConstraint(frame);
    if (mode == EndEffectorCommand::MODE_FREE) {
      // Set force equal to zero
      problem.addEqualityConstraint(
        {"force"}, {I_i}, VectorXd::Zero(m_i));
    } else if (mode == EndEffectorCommand::MODE_TRANSITION) {
      // Set force equal to wrench setpoint
      problem.addEqualityConstraint(
        {"force"}, {I_i}, end_effector_goals_.at(frame).setpoint);
    } else if (mode == EndEffectorCommand::MODE_CONTACT) {
      // Set force to be within adhesion constraints
      problem.addInequalityConstraint(
        {"force"}, {constraint.A * I_i}, constraint.b);
    } else if (mode == EndEffectorCommand::MODE_STANCE) {
      // Set force + margin to be within adhesion constraints
      problem.addInequalityConstraint(
        {"force", "margin"},
        {constraint.A * I_i, MatrixXd::Ones(constraint.b.size(), 1)},
        constraint.b);
    }
    row += m_i;
  }

  // Obstacle collision constraints
  for (const auto & obstacle : obstacles_) {
    problem.addInequalityConstraint(
      {"com"}, {obstacle.normal.transpose()},
      VectorXd::Constant(1, std::max(obstacle.distance, -body_step_)));
  }

  // Body position bounds
  Eigen::Vector3d ee;
  Eigen::Vector3d lower = Eigen::Vector3d::Zero();
  Eigen::Vector3d upper = Eigen::Vector3d::Zero();
  for (const auto & frame : robot_->getContactFrames()) {
    if (robot_->getContactType(frame) == ContactType::TAIL) {
      continue;
    }
    ee = robot_->getTransform(frame).translation();
    lower = lower.cwiseMin(ee);
    upper = upper.cwiseMax(ee);
  }
  Eigen::Vector3d lb = body_min_limits_ + upper;
  Eigen::Vector3d ub = body_max_limits_ + lower;
  problem.addBounds(
    "com", lb.cwiseMin((lb + ub) / 2), ub.cwiseMax((lb + ub) / 2));

  // Solve the optimization problem
  bool success;
  if (solver_->getInitialized()) {
    success = solver_->update(problem);
  } else {
    success = solver_->solve(problem);
  }
  if (!success) {
    return false;
  }
  force_cmd_ = solver_->getSolution().head(m);
  twist.head(3) = solver_->getSolution().segment(m, 3);
  margin_ = solver_->getSolution().tail(1)(0);

  // Compute end effector displacements
  Eigen::VectorXd df = force_cmd_ - force;
  Eigen::VectorXd feedback = Eigen::VectorXd::Zero(m);
  Eigen::VectorXd feedforward = Eigen::VectorXd::Zero(m);
  row = 0;
  for (const auto & frame : robot_->getContactFrames()) {
    uint8_t mode = end_effector_goals_[frame].mode;
    auto m_i = robot_->getNumConstraints(frame);
    if (mode == EndEffectorCommand::MODE_FREE) {
      feedforward.segment(row, m_i) = end_effector_goals_[frame].setpoint;
    } else {
      feedback.segment(row, m_i) = force_kp_ * df.segment(row, m_i);
    }
    row += m_i;
  }
  feedback -= Gs.transpose() *
    Gs.transpose().completeOrthogonalDecomposition().pseudoInverse() * feedback;
  feedforward += Gs.transpose() * twist * body_kp_;
  Eigen::VectorXd dx = feedback + feedforward;

  // Constraint body joint angle (TODO: generalize shape constraints)
  MatrixXd Jh_augmented(m + 1, n);
  Jh_augmented << Jh, MatrixXd::Zero(1, n);
  Jh_augmented(m, 12) = 1;
  VectorXd dx_augmented(m + 1);
  dx_augmented << dx, -0.1 * robot_->getJointPosition()(12);

  // Compute joint displacements
  if (!displacement_cmd_.size()) {
    displacement_cmd_ = Eigen::VectorXd::Zero(n + 6);
    position_cmd_ = robot_->getJointPosition();
  }
  displacement_cmd_.head(n) =
    Jh_augmented.completeOrthogonalDecomposition().solve(dx_augmented);
  displacement_cmd_ /= std::max(
    displacement_cmd_.head(n).cwiseAbs().maxCoeff() / joint_step_, 1.0);
  Eigen::VectorXd q0 = position_cmd_;
  position_cmd_ += displacement_cmd_.head(n);
  position_cmd_ = position_cmd_
    .cwiseMin((robot_->getJointPosition().array() + joint_max_error_).matrix())
    .cwiseMax((robot_->getJointPosition().array() - joint_max_error_).matrix());
  Eigen::VectorXd dx_actual = Jh * (position_cmd_ - q0);
  row = 0;
  for (const auto & frame : robot_->getContactFrames()) {
    auto mode = end_effector_goals_.at(frame).mode;
    auto m_i = robot_->getNumConstraints(frame);
    if (mode != EndEffectorCommand::MODE_STANCE) {
      dx_actual.segment(row, m_i).setZero();
      Gs.block(0, row, 6, m_i).setZero();
    }
    row += m_i;
  }
  displacement_cmd_.segment(n, 6) =
    Gs.transpose().completeOrthogonalDecomposition().pseudoInverse() *
    dx_actual;
  effort_cmd_ = Jh.transpose() * force_cmd_ + N;
  return true;
}

void ForceController::setEndEffectorCommand(const EndEffectorCommand & command)
{
  Eigen::Vector<double, 6> setpoint;
  for (size_t i = 0; i < command.frame.size(); i++) {
    std::string frame = command.frame[i];
    EndEffectorGoal goal;
    goal.mode =
      i < command.mode.size() ? command.mode[i] : EndEffectorCommand::MODE_FREE;
    setpoint.setZero();
    if (goal.mode == EndEffectorCommand::MODE_FREE) {
      if (i < command.twist.size()) {
        setpoint = RosUtils::twistToEigen(command.twist[i]);
      }
    } else if (goal.mode == EndEffectorCommand::MODE_TRANSITION) {
      if (i < command.wrench.size()) {
        setpoint = RosUtils::wrenchToEigen(command.wrench[i]);
      }
    }
    goal.setpoint = robot_->getWrenchBasis(frame).transpose() * setpoint;
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
    case ContactType::MICROSPINE:
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
    case ContactType::FRICTION:
      constraint.A = Eigen::MatrixXd(5, 3);
      constraint.A <<
        -friction_coefficient_, 0, 1,
        -friction_coefficient_, 0, -1,
        -friction_coefficient_, 1, 0,
        -friction_coefficient_, -1, 0,
        -1, 0, 0;
      constraint.b = Eigen::VectorXd::Zero(5);
      break;
    case ContactType::TAIL:
      constraint.A = Eigen::MatrixXd::Constant(1, 1, -1);
      constraint.b = Eigen::VectorXd::Zero(1);
      break;
    case ContactType::DEFAULT:
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
    "force_kp", 0.0001,
    "Contact force feedback gain in m/N", 0.0);
  declareParameter(
    "body_kp", 0.1,
    "Body position feedback gain in m/m", 0.0);
  declareParameter(
    "joint_step", 0.0,
    "Maximum joint displacement per timestep in rad or m (0 to disable)", 0.0);
  declareParameter(
    "body_step", 0.0,
    "Maximum body displacement per timestep in m (0 to disable)", 0.0);
  declareParameter(
    "joint_max_error", 0.0,
    "Maximum joint position error in rad (0 to disable)", 0.0);
  declareParameter(
    "joint_normalization", 0.0,
    "Cost of joint displacement relative to stability margin " \
    "in N/rad^2 and N/m^2", 0.0);
  declareParameter(
    "body_normalization", 0.0,
    "Cost of body displacement relative to stability margin " \
    "in N/m^2", 0.0);
  declareParameter(
    "clearance", 0.0,
    "Minimum height of body frame origin above ground plane in m");
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
  declareParameter(
    "body_min_limits", std::vector<double>{0, 0, 0},
    "Minimum body displacement in body frame in m");
  declareParameter(
    "body_max_limits", std::vector<double>{0, 0, 0},
    "Maximum body displacement in body frame in m");
  for (const auto & param : solver_->getParameters()) {
    parameters_.push_back(param);
  }
}

void ForceController::setParameter(const Parameter & param, SetParametersResult & result)
{
  if (param.get_name() == "force_kp") {
    force_kp_ = param.as_double();
  } else if (param.get_name() == "body_kp") {
    body_kp_ = param.as_double();
  } else if (param.get_name() == "joint_step") {
    joint_step_ = param.as_double() ? param.as_double() : INFINITY;
  } else if (param.get_name() == "body_step") {
    body_step_ = param.as_double() ? param.as_double() : INFINITY;
  } else if (param.get_name() == "joint_max_error") {
    joint_max_error_ = param.as_double() ? param.as_double() : INFINITY;
  } else if (param.get_name() == "joint_normalization") {
    joint_normalization_ = param.as_double();
  } else if (param.get_name() == "body_normalization") {
    body_normalization_ = param.as_double();
  } else if (param.get_name() == "clearance") {
    clearance_ = param.as_double();
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
    max_effort_ = param.as_double() ? param.as_double() : INFINITY;
  } else if (param.get_name() == "body_min_limits") {
    std::vector<double> limits = param.as_double_array();
    if (limits.size() == 3) {
      body_min_limits_ = RosUtils::vectorToEigen(limits);
    } else {
      result.successful = false;
      result.reason = "Invalid vector size (expected 3)";
    }
  } else if (param.get_name() == "body_max_limits") {
    std::vector<double> limits = param.as_double_array();
    if (limits.size() == 3) {
      body_max_limits_ = RosUtils::vectorToEigen(limits);
    } else {
      result.successful = false;
      result.reason = "Invalid vector size (expected 3)";
    }
  }
  solver_->setParameter(param, result);
}
