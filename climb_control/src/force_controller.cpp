#include "climb_control/force_controller.hpp"

#include <Eigen/Dense>
#include <climb_optimization/qp_interfaces/piqp_interface.hpp>
#include <climb_optimization/qp_problem.hpp>
#include <climb_util/ros_utils.hpp>
#include <climb_util/eigen_utils.hpp>

ForceController::ForceController(std::shared_ptr<KinematicsInterface> robot)
: robot_(robot)
{
  solver_ = std::make_unique<PiqpInterface>();
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

bool ForceController::updateDecoupled(
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
  Eigen::Vector3d margin =
    (lower + body_max_limits_) - (upper + body_min_limits_);
  margin = margin.cwiseMin(0);
  problem.addBounds(
    "com",
    upper + body_min_limits_ + margin / 2,
    lower + body_max_limits_ - margin / 2);

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

  using Eigen::MatrixXd, Eigen::VectorXd;
  if (end_effector_goals_.empty()) {
    reset();
  }

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
    if (end_effector_goals_.at(frame).mode == EndEffectorCommand::MODE_FREE) {
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
  problem.addQuadraticCost("joint", joint_normalization_, configuration_ - q);
  problem.addQuadraticCost("body", joint_normalization_, {});
  problem.addQuadraticCost("error", joint_normalization_, {});
  problem.addQuadraticCost("margin", joint_normalization_, {});

  // Linear penalty on tracking error
  problem.addLinearCost("error", tracking_);

  // Maximize stability margin
  problem.addLinearCost("margin", -stiffness_);

  // Respect joint limits
  problem.addBounds("joint", qmin - q, qmax - q);

  // Respect joint effort limits
  problem.addInequalityConstraint(
    {"displacement"}, {Jh.transpose() * K * A},
    umin - u, umax - u);

  // Mode-dependent constraints for each contact
  int row = 0;
  for (auto frame : robot_->getContactFrames()) {
    uint8_t mode = end_effector_goals_.at(frame).mode;
    auto m_i = robot_->getNumConstraints(frame);
    MatrixXd A_i = A.block(row, 0, m_i, n + p);
    MatrixXd K_i = K.block(row, row, m_i, m_i);
    MatrixXd I_i = MatrixXd::Identity(m, m).block(row, 0, m_i, m);
    VectorXd force_i = force.segment(row, m_i);
    if (mode == EndEffectorCommand::MODE_FREE) {
      problem.addInequalityConstraint(
        // -displacement - error <= -setpoint
        {"displacement", "error"}, {-stiffness_ * A_i, -stiffness_ * I_i},
        -stiffness_ * end_effector_goals_[frame].setpoint);
      problem.addInequalityConstraint(
        // displacement - error <= setpoint
        {"displacement", "error"}, {stiffness_ * A_i, -stiffness_ * I_i},
        stiffness_ * end_effector_goals_[frame].setpoint);
    } else if (mode == EndEffectorCommand::MODE_TRANSITION) {
      problem.addInequalityConstraint(
        // -displacement - error <= -setpoint
        {"displacement", "error"}, {-K_i * A_i, -K_i * I_i},
        -(end_effector_goals_[frame].setpoint - force_i));
      problem.addInequalityConstraint(
        // displacement - error <= setpoint
        {"displacement", "error"}, {K_i * A_i, -K_i * I_i},
        end_effector_goals_[frame].setpoint - force_i);
    } else if (mode == EndEffectorCommand::MODE_CONTACT) {
      auto constraint = getAdhesionConstraint(frame);
      problem.addInequalityConstraint(
        // A_c * displacement <= b_c
        {"displacement"},
        {constraint.A * K_i * A_i},
        (constraint.b - constraint.A * force_i));
      problem.addEqualityConstraint({"error"}, {I_i}, 0.0);
    } else if (mode == EndEffectorCommand::MODE_STANCE) {
      auto constraint = getAdhesionConstraint(frame);
      problem.addInequalityConstraint(
        // A_c * displacement + margin <= b_c
        {"displacement", "margin"},
        {constraint.A * K_i * A_i,
          VectorXd::Constant(constraint.b.size(), stiffness_)},
        (constraint.b - constraint.A * force_i));
      problem.addEqualityConstraint({"error"}, {I_i}, 0.0);
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
  problem.addEqualityConstraint(
    {"displacement"}, {Gs.topRows(3) * K * A}, 0.0);
  problem.addEqualityConstraint(
    {"displacement", "joint", "body"},
    {Gs.bottomRows(3) * K * A, gskew * J_mass, gskew * G_mass}, 0.0);

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
    problem.addInequalityConstraint(
      {"joint", "body"},
      {stiffness_ * normal * J_ob, stiffness_ * normal * G_ob},
      stiffness_ * VectorXd::Constant(1, std::max(0.0, obstacle.distance)));
  }

  problem.A *= constraint_scale_;
  problem.b *= constraint_scale_;
  problem.Aeq *= constraint_scale_;
  problem.beq *= constraint_scale_;

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
    displacement_cmd_ /=
      std::max(1.0, displacement_cmd_.cwiseAbs().maxCoeff() / joint_step_);
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
    "stiffness", 1000.0,
    "Expected compliance at the point of contact in N/m", 0.0);
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
  declareParameter(
    "constraint_scale", 1.0,
    "Scale factor for constraints in optimization", 0.0);
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
  if (param.get_name() == "stiffness") {
    stiffness_ = param.as_double();
  } else if (param.get_name() == "force_kp") {
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
  } else if (param.get_name() == "constraint_scale") {
    constraint_scale_ = param.as_double();
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
