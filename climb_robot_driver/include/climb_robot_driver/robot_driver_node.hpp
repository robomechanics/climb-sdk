#ifndef CLIMB_ROBOT_DRIVER__ROBOT_DRIVER_NODE_HPP_
#define CLIMB_ROBOT_DRIVER__ROBOT_DRIVER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <climb_msgs/msg/actuator_state.hpp>
#include <climb_msgs/msg/joint_command.hpp>
#include <climb_msgs/srv/actuator_enable.hpp>
#include "climb_robot_driver/interfaces/hardware_interface.hpp"

using sensor_msgs::msg::JointState;
using climb_msgs::msg::JointCommand;
using climb_msgs::msg::ActuatorState;
using climb_msgs::srv::ActuatorEnable;

/**
 * @brief ROS node that interfaces with the robot's physical actuators
 *
 * Subscribers: joint_commands
 * Publishers: joint_states, actuator_states
 * Services: actuator_enable
 */
class HardwareNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for HardwareNode
   */
  HardwareNode();

  /**
   * @brief Read and publish current joint and actuator values
   */
  void update();

private:
  /**
   * @brief Send commanded values to the joint actuators
   * @param[in] msg Message containing joint commands
   */
  void jointCmdCallback(const JointCommand::SharedPtr msg);

  /**
   * @brief Send enable/disable commands to the actuators
   * @param[in] request Request containing actuator commands
   * @param[out] response Response containing command results
   */
  void actuatorEnableCallback(
    const ActuatorEnable::Request::SharedPtr request,
    ActuatorEnable::Response::SharedPtr response);

  /**
   * @brief Update modified parameters
   * @param[in] parameters Modified parameter values
   * @return Result of the parameter update
   */
  rcl_interfaces::msg::SetParametersResult parameterCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  /**
   * @brief Update actuators in the hardware interface
   */
  void updateActuators();

  // Hardware interface
  std::unique_ptr<HardwareInterface> interface_;
  // Joint state publisher
  rclcpp::Publisher<JointState>::SharedPtr joint_pub_;
  // Actuator state publisher
  rclcpp::Publisher<ActuatorState>::SharedPtr actuator_pub_;
  // Joint command subscriber
  rclcpp::Subscription<JointCommand>::SharedPtr joint_cmd_sub_;
  // Actuator enable/disable service
  rclcpp::Service<ActuatorEnable>::SharedPtr actuator_enable_srv_;
  // Parameter callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    param_handle_;

  // Actuator IDs
  std::vector<int> ids_;
  // Joint names corresponding to each actuator
  std::vector<std::string> joints_;
  // Actuator models
  std::vector<std::string> models_;
  // Joint state update period
  rclcpp::Duration joint_update_period_;
  // Actuator state update period
  rclcpp::Duration actuator_update_period_;
  // Time of last joint state update
  rclcpp::Time last_joint_update_;
  // Time of last actuator state update
  rclcpp::Time last_actuator_update_;
};

#endif  // CLIMB_ROBOT_DRIVER__ROBOT_DRIVER_NODE_HPP_
