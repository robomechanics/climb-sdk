#ifndef CONTROLLER_NODE_HPP
#define CONTROLLER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include "climb_msgs/msg/joint_command.hpp"
#include "climb_msgs/msg/contact_state.hpp"
#include "climb_msgs/msg/end_effector_command.hpp"
#include "climb_msgs/srv/controller_enable.hpp"

#include "climb_main/kinematics/kinematics_node.hpp"
#include "climb_main/controller/contact_estimator.hpp"
#include "climb_main/controller/force_estimator.hpp"
#include "climb_main/controller/force_controller.hpp"

using std_msgs::msg::String;
using geometry_msgs::msg::WrenchStamped;
using sensor_msgs::msg::JointState;
using climb_msgs::msg::JointCommand;
using climb_msgs::msg::ContactState;
using climb_msgs::msg::EndEffectorCommand;
using climb_msgs::srv::ControllerEnable;

/**
 * @brief ROS node that determines optimal joint commands to maximize adhesion
 *
 * Services: controller_enable
 * Subscribers: end_effector_commands, joint_states, robot_description
 * Publishers: joint_commands, contact_states, contact_forces, tf contact frames
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
    const ControllerEnable::Request::SharedPtr request,
    ControllerEnable::Response::SharedPtr response);

  /**
   * @brief Update modified parameters
   * @param[in] parameters Modified parameter values
   * @return Result of the parameter update
   */
  rcl_interfaces::msg::SetParametersResult parameterCallback(
    const std::vector<rclcpp::Parameter> & parameters) override;

  // TF prefix
  std::string name_;
  // Contact frame transform estimator
  std::unique_ptr<ContactEstimator> contact_estimator_;
  // Contact force estimator
  std::unique_ptr<ForceEstimator> force_estimator_;
  // Contact force controller
  std::unique_ptr<ForceController> force_controller_;
  // Joint command publisher
  rclcpp::Publisher<JointCommand>::SharedPtr joint_cmd_pub_;
  // Contact force publisher
  std::vector<rclcpp::Publisher<WrenchStamped>::SharedPtr> contact_force_pubs_;
  // Contact command subscriber
  rclcpp::Subscription<EndEffectorCommand>::SharedPtr ee_cmd_sub_;
  // Controller enable service
  rclcpp::Service<ControllerEnable>::SharedPtr controller_enable_srv_;
  // TF broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  // Parameter callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    param_handle_;
  // Enabled flag
  bool enabled_;
  // Debug flag
  bool debug_;
  // Maximum joint effort
  double max_effort_;
  // Most recent gravity vector estimate
  Eigen::Vector3d gravity_;
  // Simulated wrench for offline testing
  Eigen::VectorXd sim_wrench_;
};

#endif  // CONTROLLER_NODE_HPP
