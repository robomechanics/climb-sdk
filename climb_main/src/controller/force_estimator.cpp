#include "climb_main/controller/force_estimator.hpp"

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
  Eigen::MatrixXd J = robot_->getHandJacobian();

  // Set up linear system of equations
  Eigen::MatrixXd A = -J.transpose();

  // Apply gravity offset
  if (gravity_offset_) {
    Eigen::MatrixXd Gs = -robot_->getGraspMap();
    Eigen::MatrixXd dVdg = robot_->getGravitationalMatrix();
    double m = robot_->getMass();
    A += dVdg * Gs.topRows(3) / m;
  }

  // Solve system (use SVD in case of singular A)
  return A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(u);
}

Eigen::VectorXd ForceEstimator::update(
  const Eigen::VectorXd & effort, const Imu & imu)
{
  // Update effort estimate
  Eigen::VectorXd u = filterEffort(effort);

  // Estimate external wrench from IMU data
  Eigen::Vector3d g;
  g << imu.linear_acceleration.x, imu.linear_acceleration.y,
    imu.linear_acceleration.z;
  Eigen::Matrix<double, 6, 3> g_to_f_ext;
  g_to_f_ext << Eigen::Matrix3d::Identity(3, 3), robot_->getSkew(robot_->getCenterOfMass());
  if (g.norm() > 0) {
    g_to_f_ext *= robot_->getMass() * g_ / g.norm();
  }
  Eigen::VectorXd f_ext = g_to_f_ext * g;

  // Compute robot kinematics
  Eigen::MatrixXd J = robot_->getHandJacobian();
  Eigen::MatrixXd Gs = -robot_->getGraspMap();

  // Set up linear system of equations
  Eigen::MatrixXd A(J.cols() + Gs.rows(), Gs.cols());
  A.block(0, 0, J.cols(), J.rows()) = -J.transpose();
  A.block(J.cols(), 0, Gs.rows(), Gs.cols()) = Gs;
  Eigen::VectorXd b = Eigen::VectorXd::Zero(J.cols() + Gs.rows());
  b.head(J.cols()) = u;
  b.tail(Gs.rows()) = f_ext;

  // Apply gravity offset
  if (gravity_offset_) {
    Eigen::MatrixXd N =
      robot_->getGravitationalVector(f_ext.head(3) / robot_->getMass());
    b.head(J.cols()) -= N;
  }

  // Solve fully or under-determined system (use SVD in case of singular A)
  if (A.rows() <= A.cols()) {
    return A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
  }

  // Solve over-determined system
  // If covariance is unknown, assume equal weighting
  if (effort_variance_.size() == 0 ||
    imu.linear_acceleration_covariance.size() == 0)
  {
    return (A.transpose() * A).ldlt().solve(A.transpose() * b);
  }
  // If effort variance size is mismatched just use the first element
  if (effort_variance_.size() != u.size()) {
    effort_variance_ = Eigen::VectorXd::Ones(u.size()) * effort_variance_[0];
  }
  // Compute individual sensor covariance matrices
  Eigen::MatrixXd u_cov = effort_variance_.asDiagonal();
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> g_cov(
    imu.linear_acceleration_covariance.data());
  Eigen::MatrixXd f_cov = g_to_f_ext * g_cov * g_to_f_ext.transpose();
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

std::vector<WrenchStamped> ForceEstimator::forcesToMessages(
  Eigen::VectorXd forces, rclcpp::Time stamp, std::string tf_prefix)
{
  std::vector<WrenchStamped> messages;
  auto contacts = robot_->getContactFrames();
  contacts.push_back(robot_->getBodyFrame());
  size_t index = 0;
  for (size_t i = 0; i < contacts.size(); i++) {
    WrenchStamped message;
    message.header.stamp = stamp;
    if (tf_prefix.size()) {
      message.header.frame_id = tf_prefix + "/" + contacts[i];
    } else {
      message.header.frame_id = contacts[i];
    }
    Eigen::VectorXd wrench;
    if (i == contacts.size() - 1) {
      Eigen::MatrixXd Gs = -robot_->getGraspMap();
      wrench = Gs * forces;
    } else {
      Eigen::MatrixXd basis = robot_->getWrenchBasis(contacts[i]);
      Eigen::VectorXd f = forces(Eigen::seq(index, index + basis.cols() - 1));
      wrench = basis * f;
      index += basis.cols();
    }
    message.wrench.force.x = wrench(0);
    message.wrench.force.y = wrench(1);
    message.wrench.force.z = wrench(2);
    message.wrench.torque.x = wrench(3);
    message.wrench.torque.y = wrench(4);
    message.wrench.torque.z = wrench(5);
    messages.push_back(message);
  }
  return messages;
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
    effort_variance_ = Eigen::Map<const Eigen::VectorXd>(
      param.as_double_array().data(), param.as_double_array().size());
  } else if (param.get_name() == "gravity") {
    g_ = param.as_double();
  } else if (param.get_name() == "gravity_offset") {
    gravity_offset_ = param.as_bool();
  }
}
