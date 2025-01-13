#ifndef CLIMB_CONTROL__CONTROLLER_NODE_HPP_
#define CLIMB_CONTROL__CONTROLLER_NODE_HPP_

#include <Eigen/Geometry>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <climb_msgs/msg/joint_command.hpp>
#include <climb_msgs/msg/contact_force.hpp>
#include <climb_msgs/msg/end_effector_command.hpp>
#include <climb_kinematics/kinematics_node.hpp>
#include "climb_control/contact_estimator.hpp"
#include "climb_control/force_controller.hpp"
#include "climb_control/force_estimator.hpp"

using geometry_msgs::msg::WrenchStamped;
using sensor_msgs::msg::JointState;
using sensor_msgs::msg::Imu;
using climb_msgs::msg::JointCommand;
using climb_msgs::msg::ContactForce;
using climb_msgs::msg::EndEffectorCommand;
using std_srvs::srv::SetBool;

/**
 * @brief ROS node that determines optimal joint commands to maximize adhesion
 *
 * Services: controller_enable
 * Subscribers: end_effector_commands, joint_states, robot_description
 * Publishers: joint_commands, contact_forces, tf contact frames
 */
class ControllerNode : public KinematicsNode
{
public:
  /**
   * @brief Constructor for ControllerNode
   */
  ControllerNode();

  /**
   * @brief Compute the controller output based on the latest sensor data
   */
  void update();

private:
  /**
   * @brief Update joint state with the latest data
   * @param[in] msg Message containing joint state
   */
  void jointCallback(const JointState::SharedPtr msg) override;

  /**
   * @brief Update gravity vector estimate with the latest data
   * @param[in] msg Message containing imu data
   */
  void imuCallback(const Imu::SharedPtr msg);

  /**
   * @brief Update controller with the latest end effector command
   * @param[in] msg Message containing end effector command
   */
  void endEffectorCmdCallback(const EndEffectorCommand::SharedPtr msg);

  /**
   * @brief Enable or disable the controller
   * @param[in] request Request containing the enable flag
   * @param[out] response Response containing the result of the request
   */
  void controllerEnableCallback(
    const SetBool::Request::SharedPtr request,
    SetBool::Response::SharedPtr response);

  /**
   * @brief Update modified parameters
   * @param[in] parameters Modified parameter values
   * @return Result of the parameter update
   */
  rcl_interfaces::msg::SetParametersResult parameterCallback(
    const std::vector<rclcpp::Parameter> & parameters) override;

  // Contact frame transform estimator
  std::unique_ptr<ContactEstimator> contact_estimator_;
  // Contact force estimator
  std::unique_ptr<ForceEstimator> force_estimator_;
  // Contact force controller
  std::unique_ptr<ForceController> force_controller_;
  // Joint command publisher
  rclcpp::Publisher<JointCommand>::SharedPtr joint_cmd_pub_;
  // Contact force publisher
  rclcpp::Publisher<ContactForce>::SharedPtr contact_force_pub_;
  // Individual contact force publishers (for Rviz)
  std::vector<rclcpp::Publisher<WrenchStamped>::SharedPtr> contact_force_pubs_;
  // IMU subscriber
  rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
  // Contact command subscriber
  rclcpp::Subscription<EndEffectorCommand>::SharedPtr ee_cmd_sub_;
  // Controller enable service
  rclcpp::Service<SetBool>::SharedPtr controller_enable_srv_;
  // Parameter callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    param_handle_;
  // Most recent gravity direction vector estimate in map frame
  Eigen::Vector3d gravity_;
  // Most recent gravity covariance matrix estimate in map frame
  Eigen::Matrix3d gravity_covariance_;
  // Flag to enable the controller
  bool enabled_;
  // Flag to use a dummy robot driver (publish joint efforts directly)
  bool offline_;
  // Flag to use the current gravity estimate for contact force estimation
  bool use_gravity_;
  // Parameter to print debugging messages
  bool debug_;
  // Parameter to publish odometry estimates using dead reckoning
  bool compute_odometry_;
  // Parameter for maximum joint effort (declared by force controller)
  double max_effort_;
};

#endif  // CLIMB_CONTROL__CONTROLLER_NODE_HPP_
