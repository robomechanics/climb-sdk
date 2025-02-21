#include "climb_control/force_controller.hpp"

#include <climb_optimization/qp_interfaces/osqp_interface.hpp>
#include <climb_optimization/qp_problem.hpp>
#include <climb_util/eigen_utils.hpp>
#include <climb_util/ros_utils.hpp>

using geometry_utils::Polytope;

ForceController::ForceController(std::shared_ptr<KinematicsInterface> robot)
: robot_(robot),
  solver_(std::make_unique<OsqpInterface>()),
  configuration_(Eigen::VectorXd::Zero(robot->getNumJoints())) {}

void ForceController::reset()
{
  end_effector_goals_.clear();
  for (auto frame : robot_->getContactFrames()) {
    if (robot_->getContactType(frame) == ContactType::TAIL) {
      end_effector_goals_[frame] = {ControllerCommand::MODE_CONTACT, {}};
    } else {
      end_effector_goals_[frame] = {ControllerCommand::MODE_STANCE, {}};
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
  if (workspaces_.empty()) {
    // TODO: Temporarily hardcoded workspaces for each end effector
    auto W1 = Polytope::createBox(workspace_min_limits_, workspace_max_limits_);
    workspaces_ = {
      {"gripper_1", W1},
      {"gripper_2", W1.scaled(Eigen::Vector3d{1.0, -1.0, 1.0})},
      {"gripper_3", W1.scaled(Eigen::Vector3d{-1.0, 1.0, 1.0})},
      {"gripper_4", W1.scaled(Eigen::Vector3d{-1.0, -1.0, 1.0})}};
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
  problem.addEqualityConstraint({"force"}, {Gs}, -fext);

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
    if (mode == ControllerCommand::MODE_FREE) {
      // Set force equal to zero
      problem.addEqualityConstraint(
        {"force"}, {I_i}, VectorXd::Zero(m_i));
    } else if (mode == ControllerCommand::MODE_TRANSITION) {
      // Set force equal to wrench setpoint
      problem.addEqualityConstraint(
        {"force"}, {I_i}, end_effector_goals_.at(frame).setpoint);
    } else if (mode == ControllerCommand::MODE_CONTACT) {
      // Set force to be within adhesion constraints
      problem.addInequalityConstraint(
        {"force"}, {constraint.A * I_i}, constraint.b);
    } else if (mode == ControllerCommand::MODE_STANCE) {
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
  auto W_body = Polytope::createBox(3);
  for (const auto & contact : robot_->getContactFrames()) {
    if (robot_->getContactType(contact) == ContactType::TAIL ||
      workspaces_.find(contact) == workspaces_.end())
    {
      continue;
    }
    W_body.intersect(
      robot_->getTransform(contact).translation() - workspaces_.at(contact));
  }
  if (W_body.box) {
    // Ensure body workspace is non-empty
    for (int i = 0; i < 3; i++) {
      if (W_body.b(i) + W_body.b(i + 3) < 0) {
        double avg = (-W_body.b(i) + W_body.b(i + 3)) / 2;
        W_body.b(i) = -avg;
        W_body.b(i + 3) = avg;
      }
    }
  }
  problem.addInequalityConstraint({"com"}, {W_body.A}, W_body.b);

  // Solve the optimization problem
  bool success;
  if (problem_changed_) {
    success = solver_->solve(problem);
    problem_changed_ = false;
  } else {
    success = solver_->update(problem);
  }
  if (!success) {
    return false;
  }
  force_cmd_ = solver_->getSolution().head(m);
  twist.head(3) = solver_->getSolution().segment(m, 3);
  margin_ = solver_->getSolution().tail(1)(0);

  // Compute end effector displacements
  Eigen::MatrixXd Gs_stance = Gs;
  Eigen::VectorXd df = force_cmd_ - force;
  Eigen::VectorXd feedback = Eigen::VectorXd::Zero(m);
  Eigen::VectorXd feedforward = Eigen::VectorXd::Zero(m);
  row = 0;
  for (const auto & frame : robot_->getContactFrames()) {
    uint8_t mode = end_effector_goals_[frame].mode;
    auto m_i = robot_->getNumConstraints(frame);
    if (mode == ControllerCommand::MODE_FREE) {
      feedforward.segment(row, m_i) = end_effector_goals_[frame].setpoint;
    } else {
      feedback.segment(row, m_i) = force_kp_ * df.segment(row, m_i);
    }
    if (mode != ControllerCommand::MODE_STANCE) {
      Gs_stance.block(0, row, 6, m_i).setZero();
    }
    row += m_i;
  }
  Eigen::MatrixXd Gs_inverse =
    Gs_stance.completeOrthogonalDecomposition().pseudoInverse();
  feedback -= Gs_stance.transpose() * Gs_inverse.transpose() * feedback;
  feedforward += Gs.transpose() * twist * body_kp_;
  Eigen::VectorXd dx = feedback + feedforward;

  // Constrain body joint angle
  MatrixXd Jh_augmented(m + joint_overrides_.size(), n);
  Jh_augmented << Jh, MatrixXd::Zero(joint_overrides_.size(), n);
  VectorXd dx_augmented(m + joint_overrides_.size());
  dx_augmented << dx, VectorXd::Zero(joint_overrides_.size());
  int i = m;
  for (const auto & [index, value] : joint_overrides_) {
    Jh_augmented(i, index) = 1;
    dx_augmented(i++) = 0.1 * (value - robot_->getJointPosition()(index));
  }

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
  displacement_cmd_.segment(n, 6) = Gs_inverse.transpose() * dx_actual;
  effort_cmd_ = Jh.transpose() * force_cmd_ + N;
  return true;
}

void ForceController::setControllerCommand(const ControllerCommand & command)
{
  Eigen::Vector<double, 6> setpoint;
  for (size_t i = 0; i < command.frame.size(); i++) {
    std::string frame = command.frame[i];
    EndEffectorGoal goal;
    goal.mode =
      i < command.mode.size() ? command.mode[i] : ControllerCommand::MODE_FREE;
    setpoint.setZero();
    if (goal.mode == ControllerCommand::MODE_FREE) {
      if (i < command.twist.size()) {
        setpoint = RosUtils::twistToEigen(command.twist[i]);
      }
    } else if (goal.mode == ControllerCommand::MODE_TRANSITION) {
      if (i < command.wrench.size()) {
        setpoint = RosUtils::wrenchToEigen(command.wrench[i]);
      }
    }
    goal.setpoint = robot_->getWrenchBasis(frame).transpose() * setpoint;
    if (end_effector_goals_.find(frame) == end_effector_goals_.end() ||
      goal.mode != end_effector_goals_[frame].mode)
    {
      problem_changed_ = true;
    }
    end_effector_goals_[frame] = goal;
  }
  joint_overrides_.clear();
  for (size_t i = 0; i < command.overrides.name.size(); ++i) {
    int index = robot_->getJointIndex(command.overrides.name[i]);
    if (i < command.overrides.position.size()) {
      joint_overrides_[index] = command.overrides.position[i];
    }
  }
}

std::vector<std::string> ForceController::getStanceFrames() const
{
  std::vector<std::string> frames;
  for (const auto & [frame, goal] : end_effector_goals_) {
    if (goal.mode == ControllerCommand::MODE_STANCE) {
      frames.push_back(frame);
    }
  }
  return frames;
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
    "workspace_min_limits", std::vector<double>{0, 0, 0},
    "Lower bounds of front left end effector workspace in body frame in m");
  declareParameter(
    "workspace_max_limits", std::vector<double>{0, 0, 0},
    "Upper bounds of front left end effector workspace in body frame in m");
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
  } else if (param.get_name() == "workspace_min_limits") {
    std::vector<double> limits = param.as_double_array();
    if (limits.size() == 3) {
      workspace_min_limits_ = RosUtils::vectorToEigen(limits);
      workspaces_.clear();
    } else {
      result.successful = false;
      result.reason = "Invalid vector size (expected 3)";
    }
  } else if (param.get_name() == "workspace_max_limits") {
    std::vector<double> limits = param.as_double_array();
    if (limits.size() == 3) {
      workspace_max_limits_ = RosUtils::vectorToEigen(limits);
      workspaces_.clear();
    } else {
      result.successful = false;
      result.reason = "Invalid vector size (expected 3)";
    }
  }
  solver_->setParameter(param, result);
}
