#ifndef FORCE_ESTIMATOR_HPP
#define FORCE_ESTIMATOR_HPP

#include <string>
#include <queue>
#include <Eigen/Geometry>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include "climb_main/util/parameterized.hpp"
#include "climb_main/kinematics/kinematics_interface.hpp"

using sensor_msgs::msg::Imu;
using geometry_msgs::msg::WrenchStamped;

/**
 * @brief Estimates contact forces from joint effort and IMU if present
 */
class ForceEstimator : public Parameterized
{
public:
  /**
   * @brief Constructor for ForceEstimator
   * @param[in] robot Kinematics interface for the robot
   */
  ForceEstimator(std::shared_ptr<KinematicsInterface> robot);

  /**
   * @brief Estimate contact force from the given effort data
   * @param[in] effort Latest joint effort measurement
   * @return Vector of estimated forces (robot on world)
   */
  Eigen::VectorXd update(const Eigen::VectorXd & effort);

  /**
   * @brief Estimate contact force from the latest robot data
   * @return Vector of estimated forces (robot on world)
   */
  Eigen::VectorXd update() {return update(robot_->getJointEffort());}

  /**
   * @brief Estimate contact force from the given effort and IMU data
   * @param[in] effort Latest joint effort measurement
   * @param[in] imu Latest IMU data
   * @return Vector of estimated forces (robot on world)
   */
  Eigen::VectorXd update(const Eigen::VectorXd & effort, const Imu & imu);

  /**
   * @brief Estimate contact force from the latest robot data and given IMU data
   * @param[in] imu Latest IMU data
   * @return Vector of estimated forces (robot on world)
   */
  Eigen::VectorXd update(const Imu & imu)
  {
    return update(robot_->getJointEffort(), imu);
  }

  /**
   * @brief Reset the force estimator
   * @param[in] num_joints Number of joints in the robot
   */
  void reset(int num_joints);

  /**
   * @brief Convert contact force vector to individual WrenchStamped messages
   * @param[in] forces Vector of contact forces
   * @param[in] stamp Timestamp of contact force measurement
   * @param[in] tf_prefix Optional prefix for contact frame names
   * @return Vector of WrenchStamped messages
   */
  std::vector<WrenchStamped> forcesToMessages(
    Eigen::VectorXd forces, rclcpp::Time stamp, std::string tf_prefix = "");

  void declareParameters() override;

  void setParameter(
    const Parameter & param, SetParametersResult & result) override;

  using Parameterized::setParameter;

private:
  /**
   * @brief Filter the most recent effort measurement
   * @param[in] effort Latest joint effort measurement
   * @return Filtered joint effort estimate
   */
  Eigen::VectorXd filterEffort(const Eigen::VectorXd & effort);

  // Robot kinematics interface
  std::shared_ptr<KinematicsInterface> robot_;
  // Past effort measurements for filtering
  std::queue<Eigen::VectorXd> effort_queue_;
  // Number of past effort measurements to average
  size_t effort_window_;
  // Gravitational acceleration in m/s^2
  double g_;
  // Offset joint torques to account for robot weight
  bool gravity_offset_;
  // Diagonal elements of joint effort covariance matrix in N^2 or (Nm)^2
  Eigen::VectorXd effort_variance_;
  // Running total of past joint effort measurements
  Eigen::VectorXd effort_total_;
  // Number of valid effort measurements
  int effort_count_;
};

#endif  // FORCE_ESTIMATOR_HPP
