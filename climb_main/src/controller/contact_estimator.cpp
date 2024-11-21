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
  Eigen::Matrix3d R_be = robot_->getTransform(end_effector_frame).second;
  Eigen::Matrix3d R_bc = robot_->getTransform(contact_frame).second;
  if (wrist == KinematicsInterface::WristType::GRAVITY) {
    // Align x-axis with normal and negative z-axis with gravity
    if (gravity.size() && gravity.norm()) {
      Eigen::Vector3d g = gravity / gravity.norm();
      if (abs(normal.dot(g)) < cos(min_incline_)) {
        R_bc.col(0) = normal;
        R_bc.col(1) = -g.cross(R_bc.col(0));
        R_bc.col(1) /= R_bc.col(1).norm();
        R_bc.col(2) = R_bc.col(0).cross(R_bc.col(1));
      }
    }
    return R_be.transpose() * R_bc;
  } else if (wrist == KinematicsInterface::WristType::FIXED) {
    // Maintain constant orientation
    return Eigen::Matrix3d::Identity();
  } else if (wrist == KinematicsInterface::WristType::SPRING) {
    // Rotate from end effector normal to surface normal
    Eigen::Vector3d ee_normal = R_be.col(2);
    Eigen::Vector3d axis = ee_normal.cross(normal);
    if (axis.norm() < 1e-6) {
      return Eigen::Matrix3d::Identity();
    }
    axis /= axis.norm();
    double angle = acos(ee_normal.dot(normal));
    return Eigen::AngleAxisd(angle, axis).toRotationMatrix();
  } else if (wrist == KinematicsInterface::WristType::FREE) {
    // Rotate from previous contact normal to surface normal
    Eigen::Vector3d contact_normal = R_bc.col(2);
    Eigen::Vector3d axis = contact_normal.cross(normal);
    if (axis.norm() < 1e-6) {
      return R_be.transpose() * R_bc;
    }
    axis /= axis.norm();
    double angle = acos(contact_normal.dot(normal));
    return R_be.transpose() * R_bc *
           Eigen::AngleAxisd(angle, axis).toRotationMatrix();
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
