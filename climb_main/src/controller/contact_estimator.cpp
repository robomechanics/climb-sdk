#include "climb_main/controller/contact_estimator.hpp"

ContactEstimator::ContactEstimator(std::shared_ptr<KinematicsInterface> robot)
: robot_(robot)
{
}

void ContactEstimator::reset()
{
  // Reset contact estimator
}

std::vector<TransformStamped> ContactEstimator::update(
  Eigen::Vector3d gravity, const PointCloud2 & terrain)
{
  std::vector<TransformStamped> transforms;
  for (int i = 0; i < robot_->getNumContacts(); ++i) {
    auto ee_frame = robot_->getEndEffectorFrames()[i];
    auto contact_frame = robot_->getContactFrames()[i];
    Eigen::Vector3d normal;   // TODO Get normal from point cloud
    TransformStamped transform;
    transform.header.frame_id = ee_frame;
    transform.child_frame_id = contact_frame;
    Eigen::Quaterniond q(
      getContactOrientation(ee_frame, contact_frame, normal, gravity));
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    transforms.push_back(transform);
  }
  return transforms;
}

std::vector<TransformStamped> ContactEstimator::update(Eigen::Vector3d gravity)
{
  std::vector<TransformStamped> transforms;
  auto ground = getGroundPlane();
  for (int i = 0; i < robot_->getNumContacts(); ++i) {
    auto ee_frame = robot_->getEndEffectorFrames()[i];
    auto contact_frame = robot_->getContactFrames()[i];
    TransformStamped transform;
    transform.header.frame_id = ee_frame;
    transform.child_frame_id = contact_frame;
    Eigen::Quaterniond q(
      getContactOrientation(ee_frame, contact_frame, ground.normal, gravity));
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    transforms.push_back(transform);
  }
  return transforms;
}

ContactEstimator::Plane ContactEstimator::getGroundPlane(
  const std::vector<std::string> & contact_frames)
{
  Plane plane;
  Eigen::MatrixXd contacts(3, contact_frames.size());
  for (size_t i = 0; i < contact_frames.size(); ++i) {
    contacts.col(i) = robot_->getTransform(contact_frames[i]).first;
  }
  Eigen::Vector3d mean = contacts.rowwise().mean();
  if (contact_frames.size() < 3) {
    plane.normal = {0, 0, 1};
  } else {
    contacts.colwise() -= mean;
    plane.normal = contacts.jacobiSvd(
      Eigen::ComputeThinU | Eigen::ComputeThinV).matrixU().col(2);
  }
  if (plane.normal.dot(Eigen::Vector3d::UnitZ()) > 0) {
    plane.normal *= -1;
  }
  plane.distance = mean.dot(plane.normal);
  return plane;
}

Eigen::Matrix3d ContactEstimator::getContactOrientation(
  const std::string & end_effector_frame, const std::string & contact_frame,
  const Eigen::Vector3d & normal, const Eigen::Vector3d & gravity)
{
  auto wrist = robot_->getWristType(contact_frame);
  Eigen::Matrix3d R_ec =
    robot_->getTransform(end_effector_frame, contact_frame).second;
  if (!normal.norm()) {
    return R_ec;
  }
  Eigen::Matrix3d R_be = robot_->getTransform(end_effector_frame).second;
  Eigen::Vector3d n_e = R_be.transpose() * normal;
  Eigen::Vector3d g_e = R_be.transpose() * gravity;
  Eigen::Vector3d com_e = robot_->getTransform(
    end_effector_frame, contact_frame + "_inertial").first;
  if (wrist == KinematicsInterface::WristType::GRAVITY) {
    // Align center of mass with gravity, then align x-axis with normal
    Eigen::Matrix3d R_ec1 = R_ec;
    if (g_e.norm() && com_e.norm() &&
      g_e.dot(n_e) / (g_e.norm() * n_e.norm()) < std::cos(min_incline_) &&
      com_e.dot(n_e) / (com_e.norm() * n_e.norm()) < std::cos(min_incline_))
    {
      R_ec1 = Eigen::Quaterniond::FromTwoVectors(com_e, g_e) * R_ec;
    }
    Eigen::Matrix3d R_ec2 =
      Eigen::Quaterniond::FromTwoVectors(R_ec1.col(0), n_e) * R_ec1;
    R_ec2 /= R_ec2.col(0).norm();
    return R_ec2;
  } else if (wrist == KinematicsInterface::WristType::FIXED) {
    // Maintain constant orientation
    return Eigen::Matrix3d::Identity();
  } else if (wrist == KinematicsInterface::WristType::SPRING) {
    // Rotate from end effector normal to surface normal
    return Eigen::Quaterniond::FromTwoVectors(
      Eigen::Vector3d::UnitX(), n_e).toRotationMatrix();
  } else if (wrist == KinematicsInterface::WristType::FREE) {
    // Rotate from previous contact normal to surface normal
    return Eigen::Quaterniond::FromTwoVectors(
      R_ec.col(0), n_e).toRotationMatrix() * R_ec;
  }
  return Eigen::Matrix3d::Identity();
}

void ContactEstimator::declareParameters()
{
  declareParameter(
    "min_incline", 0.0,
    "Minimum incline for gravitational wrist alignment (rad)", 0);
}

void ContactEstimator::setParameter(
  const Parameter & param, [[maybe_unused]] SetParametersResult & result)
{
  if (param.get_name() == "min_incline") {
    min_incline_ = param.as_double();
  }
}
