#ifndef CONTROLLER_NODE_HPP
#define CONTROLLER_NODE_HPP

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "climb_msgs/msg/joint_command.hpp"
#include "climb_main/kinematics/kinematics_interface.hpp"
#include "climb_msgs/msg/contact_state.hpp"

using std_msgs::msg::String;
using sensor_msgs::msg::JointState;
using climb_msgs::msg::ContactState;
using climb_msgs::msg::JointCommand;

/**
 * @brief ROS node that sends joint commands to regulate contact forces
 *
 * Subscribers: joint_commands
 * Publishers: joint_states, actuator_states
 * Services: actuator_command
 * Parameters: actuator_ids, actuator_joints
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
  // Kinematics interface
  std::unique_ptr<KinematicsInterface> interface_;
  // Joint command publisher
  rclcpp::Publisher<JointCommand>::SharedPtr joint_cmd_pub_;
  // Robot description subscriber
  rclcpp::Subscription<String>::SharedPtr description_sub_;
  // Joint state subscriber
  rclcpp::Subscription<JointState>::SharedPtr joint_sub_;
  // Contact state subscriber
  rclcpp::Subscription<ContactState>::SharedPtr contact_sub_;
  // Contact command subscriber
  rclcpp::Subscription<ContactState>::SharedPtr contact_cmd_sub_;
  // Parameter callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    param_handle_;

  /**
   * @brief Update the kinematics model with the provided robot description
   * @param[in] msg Message containing robot description
   */
  void description_callback(const String::SharedPtr msg);

  /**
   * @brief Update joint state with the latest data
   * @param[in] msg Message containing joint state
   */
  void jointCallback(const JointState::SharedPtr msg);

  /**
   * @brief Update contact state with the latest data
   * @param[in] msg Message containing contact state
   */
  void contactCallback(const ContactState::SharedPtr msg);

  /**
   * @brief Update controller with the latest contact command
   * @param[in] msg Message containing contact command
   */
  void contactCommandCallback(const ContactState::SharedPtr msg);

  /**
   * @brief Update modified parameters
   * @param[in] parameters Modified parameter values
   * @return Result of the parameter update
   */
  rcl_interfaces::msg::SetParametersResult parameterCallback(
    const std::vector<rclcpp::Parameter> & parameters);
};

#endif  // CONTROLLER_NODE_HPP
