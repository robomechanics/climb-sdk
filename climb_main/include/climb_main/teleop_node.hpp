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

class TeleopNode : public KinematicsNode
{
public:
  /**
   * @brief Constructor for TeleopNode
   */
  TeleopNode();

private:
  const std::vector<std::vector<std::string>> COMMANDS = {
    {"CONNECT"},
    {"set", "JOINT", "[position velocity effort]", "DOUBLE"},
    {"set", "[position velocity effort]", "DOUBLE"}
  };

  void addCommands();

  void keyCallback(
    const std::shared_ptr<KeyInput::Request> request,
    std::shared_ptr<KeyInput::Response> response);

  void setJoint(
    const std::string & joint, const std::string & propery, double value);

  // void setConfiguration(
  //   const std::vector<std::string> & joints,
  //   const std::vector<double> & positions);

  // void moveJoint(
  //   const std::string & joint, double displacement);

  // void moveEndEffector(const std::string & end_effector, Twist twist);

  // void moveBody(Twist twist);

  // Key input parser
  KeyInputParser key_input_parser_;
  // Key input service
  rclcpp::Service<KeyInput>::SharedPtr key_input_service_;
  // Joint command publisher
  rclcpp::Publisher<JointCommand>::SharedPtr joint_cmd_pub_;
  // End effector command publisher
  rclcpp::Publisher<EndEffectorCommand>::SharedPtr ee_cmd_pub_;
};

#endif  // TELEOP_NODE_HPP
