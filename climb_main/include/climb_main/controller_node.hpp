#ifndef CONTROLLER_NODE_HPP
#define CONTROLLER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include "climb_msgs/msg/joint_command.hpp"
#include "climb_msgs/msg/contact_state.hpp"
#include "climb_msgs/msg/contact_command.hpp"

#include "climb_main/kinematics/kinematics_interface.hpp"
#include "climb_main/controller/force_estimator.hpp"

using std_msgs::msg::String;
using geometry_msgs::msg::WrenchStamped;
using sensor_msgs::msg::JointState;
using climb_msgs::msg::JointCommand;
using climb_msgs::msg::ContactState;
using climb_msgs::msg::ContactCommand;

/**
 * @brief ROS node that determines optimal joint commands to maximize adhesion
 *
 * Subscribers: joint_commands, joint_states, robot_description
 * Publishers: joint_states, contact_forces, tf contact frames
 * Parameters: tf_prefix
 */
class ControllerNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for ControllerNode
   */
  ControllerNode();

  /**
   * @brief Declare additional parameters
   */
  void init();

  /**
   * @brief Compute the controller output based on the latest sensor data
   */
  void update();

private:
  /**
   * @brief Update the kinematics model with the provided robot description
   * @param[in] msg Message containing robot description
   */
  void descriptionCallback(const String::SharedPtr msg);

  /**
   * @brief Update joint state with the latest data
   * @param[in] msg Message containing joint state
   */
  void jointCallback(const JointState::SharedPtr msg);

  /**
   * @brief Update controller with the latest contact command
   * @param[in] msg Message containing contact command
   */
  void contactCmdCallback(const ContactCommand::SharedPtr msg);

  /**
   * @brief Update modified parameters
   * @param[in] parameters Modified parameter values
   * @return Result of the parameter update
   */
  rcl_interfaces::msg::SetParametersResult parameterCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  // TF prefix
  std::string name_;
  // Kinematics interface
  std::shared_ptr<KinematicsInterface> robot_;
  // Contact force estimator
  std::unique_ptr<ForceEstimator> force_estimator_;
  // Joint command publisher
  rclcpp::Publisher<JointCommand>::SharedPtr joint_cmd_pub_;
  // Contact force publisher
  std::vector<rclcpp::Publisher<WrenchStamped>::SharedPtr> contact_force_pubs_;
  // Robot description subscriber
  rclcpp::Subscription<String>::SharedPtr description_sub_;
  // Joint state subscriber
  rclcpp::Subscription<JointState>::SharedPtr joint_sub_;
  // Contact command subscriber
  rclcpp::Subscription<ContactCommand>::SharedPtr contact_cmd_sub_;
  // TF broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  // Parameter callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    param_handle_;
};

#endif  // CONTROLLER_NODE_HPP
