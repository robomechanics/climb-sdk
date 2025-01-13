#ifndef CLIMB_CONTROL__CONTACT_ESTIMATOR_HPP_
#define CLIMB_CONTROL__CONTACT_ESTIMATOR_HPP_

#include <Eigen/Geometry>
#include <memory>
#include <string>
#include <vector>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <climb_kinematics/interfaces/kinematics_interface.hpp>
#include <climb_util/parameterized.hpp>

using sensor_msgs::msg::PointCloud2;
using geometry_msgs::msg::TransformStamped;

/**
 * Estimates contact frames from gravity vector and point cloud if present
 */
class ContactEstimator : public Parameterized
{
public:
  /**
   * @brief Plane defined by a normal vector and a reference point on the plane
   */
  struct Plane
  {
    Eigen::Vector3d normal;   // Normal vector of the plane
    Eigen::Vector3d origin;   // Reference point on plane
    double distance;          // Distance to plane along normal vector
    Plane()
    : normal(0, 0, 1), origin(0, 0, 0), distance(0) {}
    Plane(const Eigen::Vector3d & normal, const Eigen::Vector3d & origin)
    : normal(normal), origin(origin), distance(normal.dot(origin)) {}
    Plane(const Eigen::Vector3d & normal, double distance)
    : normal(normal), origin(normal * distance), distance(distance) {}
  };

  /**
   * @brief Constructor for ContactEstimator
   * @param[in] robot Kinematics interface for the robot
   */
  explicit ContactEstimator(std::shared_ptr<KinematicsInterface> robot);

  /**
   * @brief Reset the contact estimator
   */
  void reset();

  /**
   * @brief Update the contact frames based on the given point cloud
   * @param[in] gravity Gravity direction vector in the body frame
   * @param[in] terrain Point cloud message from the depth camera
   * @return Transforms from each end effector frame to its contact frame
   */
  std::vector<TransformStamped> update(
    const Eigen::Vector3d & gravity, const PointCloud2 & terrain);

  /**
   * @brief Update the contact frames based on nominal surface plane
   * @param[in] gravity Gravity direction vector in the body frame
   * @return Transforms from each end effector frame to its contact frame
   */
  std::vector<TransformStamped> update(const Eigen::Vector3d & gravity);

  /**
   * @brief Get the ground plane normal vector and distance from origin
   * @param[in] contact_frames List of contact frames currently in contact
   * @return Best fit ground plane
   */
  Plane getGroundPlane(const std::vector<std::string> & contact_frames);

  /**
   * @brief Get the ground plane normal vector and distance from origin
   * @return Best fit ground plane
   */
  Plane getGroundPlane() {return getGroundPlane(robot_->getContactFrames());}

  /**
   * @brief Get the rotation matrix from the parent to child frame
   * @param[in] end_effector_frame End effector frame name
   * @param[in] contact_frame Contact frame name
   * @param[in] normal Normal vector of the contact surface in the body frame
   * @param[in] gravity Gravity direction vector in the body frame
   * @return Rotation matrix from end effector to contact frame
   */
  Eigen::Matrix3d getContactOrientation(
    const std::string & end_effector_frame, const std::string & contact_frame,
    const Eigen::Vector3d & normal, const Eigen::Vector3d & gravity);

  void declareParameters() override;

  void setParameter(
    const Parameter & param, SetParametersResult & result) override;

  using Parameterized::setParameter;

private:
  // Robot kinematics interface
  std::shared_ptr<KinematicsInterface> robot_;
  // Minimum incline for gravitational wrist alignment in radians
  double min_incline_;
};

#endif  // CLIMB_CONTROL__CONTACT_ESTIMATOR_HPP_
