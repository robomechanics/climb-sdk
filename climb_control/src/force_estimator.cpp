#include "climb_control/force_estimator.hpp"
#include "climb_util/ros_utils.hpp"

ForceEstimator::ForceEstimator(std::shared_ptr<KinematicsInterface> robot)
: robot_(robot)
{
  reset(robot->getNumJoints());
}

Eigen::VectorXd ForceEstimator::update(const Eigen::VectorXd & effort)
{
  // Update effort estimate
  Eigen::VectorXd u = filterEffort(effort);

  // Compute robot kinematics
  Eigen::MatrixXd Jh = robot_->getHandJacobian();

  // Set up linear system of equations
  Eigen::MatrixXd A = Jh.transpose();
  Eigen::VectorXd b = u;

  // Apply gravity offset
  if (gravity_offset_) {
    Eigen::Matrix<double, 6, Eigen::Dynamic> Gs = -robot_->getGraspMap();
    Eigen::MatrixXd dVdg = robot_->getGravitationalMatrix();
    double m = robot_->getMass();
    A -= dVdg * Gs.topRows(3) / m;
  }

  // Solve system
  return A.completeOrthogonalDecomposition().solve(b);
}

Eigen::VectorXd ForceEstimator::update(
  const Eigen::VectorXd & effort, const Eigen::Vector3d & gravity,
  const Eigen::Matrix3d & gravity_covariance)
{
  // Update effort estimate
  Eigen::VectorXd u = filterEffort(effort);

  // Estimate external wrench from IMU data
  Eigen::Matrix<double, 6, 3> g_to_f_ext;
  g_to_f_ext << Eigen::Matrix3d::Identity(3, 3), robot_->getSkew(robot_->getCenterOfMass());
  if (gravity.norm() > 0) {
    g_to_f_ext *= robot_->getMass() * g_ / gravity.norm();
  }
  Eigen::VectorXd f_ext = g_to_f_ext * gravity;

  // Compute robot kinematics
  Eigen::MatrixXd Jh = robot_->getHandJacobian();
  Eigen::MatrixXd Gs = -robot_->getGraspMap();

  // Set up linear system of equations
  Eigen::MatrixXd A(Jh.cols() + Gs.rows(), Gs.cols());
  A << Jh.transpose(), -Gs;
  Eigen::VectorXd b(Jh.cols() + Gs.rows());
  b << u, f_ext;

  // Apply gravity offset
  if (gravity_offset_) {
    Eigen::MatrixXd N =
      robot_->getGravitationalVector(f_ext.head(3) / robot_->getMass());
    b.head(Jh.cols()) -= N;
  }

  // Solve fully or under-determined system
  if (A.rows() <= A.cols()) {
    return A.completeOrthogonalDecomposition().solve(b);
  }

  // Solve over-determined system
  // If covariance is unknown, assume equal weighting
  if (effort_variance_.size() == 0 || gravity_covariance(0) == -1) {
    return A.completeOrthogonalDecomposition().solve(b);
  }
  // If effort variance size is mismatched just use the first element
  if (effort_variance_.size() != u.size()) {
    effort_variance_ = Eigen::VectorXd::Ones(u.size()) * effort_variance_[0];
  }
  // Compute individual sensor covariance matrices
  Eigen::MatrixXd u_cov = effort_variance_.asDiagonal();
  Eigen::MatrixXd f_cov =
    g_to_f_ext * gravity_covariance * g_to_f_ext.transpose();
  // Compute inverse of combined sensor covariance matrix
  Eigen::MatrixXd sigma_inv = Eigen::MatrixXd::Zero(A.rows(), A.rows());
  sigma_inv.block(0, 0, u.size(), u.size()) =
    u_cov.completeOrthogonalDecomposition().pseudoInverse();
  sigma_inv.block(u.size(), u.size(), 6, 6) =
    f_cov.completeOrthogonalDecomposition().pseudoInverse();
  // Solve weighted least squares problem
  return (A.transpose() * sigma_inv * A).ldlt().solve(
    A.transpose() * sigma_inv * b);
}

Eigen::VectorXd ForceEstimator::filterEffort(const Eigen::VectorXd & effort)
{
  // Check for mismatched input size
  if (effort.size() && effort.size() != effort_total_.size()) {
    reset(effort.size());
  }
  // Store latest value (empty measurements do not contribute to the average)
  if (effort.size()) {
    effort_total_ += effort;
    effort_count_ += 1;
  }
  effort_queue_.push(effort);
  // Remove old values
  while (effort_queue_.size() > effort_window_) {
    if (effort_queue_.front().size()) {
      effort_total_ -= effort_queue_.front();
      effort_count_ -= 1;
    }
    effort_queue_.pop();
  }
  return effort_count_ ? effort_total_ / effort_count_ : effort_total_;
}

void ForceEstimator::reset(int num_joints)
{
  effort_queue_ = std::queue<Eigen::VectorXd>();
  effort_total_ = Eigen::VectorXd::Zero(num_joints);
  effort_count_ = 0;
}

ContactForce ForceEstimator::getContactForceMessage(
  const Eigen::VectorXd & forces, const rclcpp::Time & stamp)
{
  ContactForce message;
  Eigen::Matrix<double, 6, Eigen::Dynamic> basis;
  Eigen::Vector<double, 6> wrench;
  size_t index = 0;
  for (auto frame : robot_->getContactFrames()) {
    basis = robot_->getWrenchBasis(frame);
    wrench = basis * forces.segment(index, basis.cols());
    index += basis.cols();
    message.frame.emplace_back(std::move(frame));
    message.wrench.emplace_back(RosUtils::eigenToWrench(wrench));
  }
  message.header.stamp = stamp;
  return message;
}

std::vector<WrenchStamped> ForceEstimator::splitContactForceMessage(
  const ContactForce & message)
{
  using RosUtils::operator-;
  std::vector<WrenchStamped> messages;
  messages.reserve(message.frame.size());
  for (size_t i = 0; i < message.frame.size(); i++) {
    WrenchStamped wrench;
    wrench.header.stamp = message.header.stamp;
    wrench.header.frame_id = message.frame[i];
    wrench.wrench = -(message.wrench[i]);
    messages.push_back(std::move(wrench));
  }
  return messages;
}

WrenchStamped ForceEstimator::getGravityForceMessage(
  const Eigen::VectorXd & forces, const rclcpp::Time & stamp)
{
  WrenchStamped message;
  Eigen::Matrix<double, 6, Eigen::Dynamic> Gs = -robot_->getGraspMap();
  Eigen::Vector<double, 6> wrench = -Gs * forces;
  message.header.frame_id = robot_->getBodyFrame();
  message.header.stamp = stamp;
  message.wrench = RosUtils::eigenToWrench(wrench);
  return message;
}

void ForceEstimator::declareParameters()
{
  declareParameter(
    "joint_effort_filter_window", 1,
    "Number of past joint effort measurements to average", 1);
  declareParameter(
    "joint_effort_variance", std::vector<double>(),
    "Variance of joint effort measurements in N^2 or (Nm)^2", 0);
  declareParameter(
    "gravity", 9.81, "Gravitational acceleration in m/s^2", 0);
  declareParameter(
    "gravity_offset", true,
    "Offset joint torque estimates to account for mass of robot limbs");
}

void ForceEstimator::setParameter(
  const rclcpp::Parameter & param,
  [[maybe_unused]] rcl_interfaces::msg::SetParametersResult & result)
{
  if (param.get_name() == "joint_effort_filter_window") {
    effort_window_ = param.as_int();
  } else if (param.get_name() == "joint_effort_variance") {
    effort_variance_ = Eigen::VectorXd::Map(
      param.as_double_array().data(), param.as_double_array().size()).eval();
  } else if (param.get_name() == "gravity") {
    g_ = param.as_double();
  } else if (param.get_name() == "gravity_offset") {
    gravity_offset_ = param.as_bool();
  }
}
