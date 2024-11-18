#ifndef TELEOP_NODE_HPP
#define TELEOP_NODE_HPP

#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <climb_msgs/srv/key_input.hpp>
#include <climb_msgs/msg/joint_command.hpp>
#include <climb_msgs/msg/end_effector_command.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "climb_main/teleop/key_input_parser.hpp"
#include "climb_main/kinematics/kinematics_node.hpp"

using climb_msgs::srv::KeyInput;
using climb_msgs::msg::JointCommand;
using climb_msgs::msg::EndEffectorCommand;
using sensor_msgs::msg::JointState;
using geometry_msgs::msg::Twist;

/**
 * @brief ROS node that publishes teleop commands
 *
 * Services: key_input
 * Subscribers: joint_states, robot_description
 * Publishers: joint_commands, end_effector_commands
 */
class TeleopNode : public KinematicsNode
{
public:
  /**
   * @brief Constructor for TeleopNode
   */
  TeleopNode();

private:
  /**
   * @brief Add commands to the key input parser
   */
  void addCommands();

  /**
   * @brief Callback for key input service
   * @param request The key input request
   * @param response The key input response
   */
  void keyCallback(
    const std::shared_ptr<KeyInput::Request> request,
    std::shared_ptr<KeyInput::Response> response);

  /**
   * @brief Convert key press into a twist
   * @param key The key character (linear: wasdqe, angular: WASDQE)
   * @return A twist message
   */
  Twist getTwist(char key) const;

  /**
   * @brief Set a joint property
   * @param joint The joint name
   * @param property The property name (position, velocity, or effort)
   * @param value The property value (in rad, rad/s, or Nm)
   */
  void setJoint(
    const std::string & joint, const std::string & propery, double value);

  /**
   * @brief Move to a specific configuration
   * @param joints The joint names
   * @param positions The joint positions
   */
  void setConfiguration(
    const std::vector<std::string> & joints,
    const std::vector<double> & positions);

  void moveJoint(const std::string & joint, double displacement);

  void moveJoints(const Eigen::VectorXd & displacements);

  void moveEndEffector(const std::string & contact, Twist twist);

  void moveBody(Twist twist);

  rcl_interfaces::msg::SetParametersResult parameterCallback(
    const std::vector<rclcpp::Parameter> & parameters) override;

  // Key input parser
  KeyInputParser key_input_parser_;
  // Key input service
  rclcpp::Service<KeyInput>::SharedPtr key_input_service_;
  // Joint command publisher
  rclcpp::Publisher<JointCommand>::SharedPtr joint_cmd_pub_;
  // End effector command publisher
  rclcpp::Publisher<EndEffectorCommand>::SharedPtr ee_cmd_pub_;
  // Poses
  std::map<std::string, std::vector<double>> configurations_;
  // Joint setpoints
  Eigen::VectorXd joint_setpoints_;
  // Joint step in rad
  double joint_step_;
  // Twist linear step in m
  double linear_step_;
  // Twist angular step in rad
  double angular_step_;
};

#endif  // TELEOP_NODE_HPP
