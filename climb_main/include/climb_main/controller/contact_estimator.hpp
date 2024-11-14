#ifndef CONTACT_ESTIMATOR_HPP
#define CONTACT_ESTIMATOR_HPP

#include <vector>
#include <Eigen/Dense>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "climb_main/kinematics/kinematics_interface.hpp"
#include "climb_main/util/parameterized.hpp"

using sensor_msgs::msg::PointCloud2;
using geometry_msgs::msg::TransformStamped;

/**
 * Estimates contact frames from gravity vector and point cloud if present
 */
class ContactEstimator : public Parameterized
{
public:
  struct Plane
  {
    Eigen::Vector3d normal;   // Normal vector of the plane
    double distance;          // Distance from origin along normal vector
  };

  /**
   * @brief Constructor for ContactEstimator
   * @param[in] robot Kinematics interface for the robot
   */
  ContactEstimator(std::shared_ptr<KinematicsInterface> robot);

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
    Eigen::Vector3d gravity, const PointCloud2 & terrain);

  /**
   * @brief Update the contact frames based on nominal surface plane
   * @param[in] gravity Gravity direction vector in the body frame
   * @return Transforms from each end effector frame to its contact frame
   */
  std::vector<TransformStamped> update(Eigen::Vector3d gravity);

  /**
   * @brief Get the ground plane normal vector and distance from origin
   * @param[in] contact_frames List of contact frames currently in contact
   * @return Best fit ground plane
   */
  Plane getGroundPlane(std::vector<std::string> contact_frames);

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

#endif  // CONTACT_ESTIMATOR_HPP
