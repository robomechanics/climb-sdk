#ifndef TELEOP_NODE_HPP
#define TELEOP_NODE_HPP

#include "climb_teleop/key_input_parser.hpp"
#include <climb_kinematics/kinematics_node.hpp>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <climb_msgs/srv/key_input.hpp>
#include <climb_msgs/srv/actuator_enable.hpp>
#include <climb_msgs/srv/set_string.hpp>
#include <climb_msgs/msg/joint_command.hpp>
#include <climb_msgs/msg/end_effector_command.hpp>
#include <climb_msgs/msg/step_override_command.hpp>
#include <climb_msgs/msg/teleop_message.hpp>
#include <climb_msgs/action/step_command.hpp>
#include <Eigen/Dense>

using std_srvs::srv::Trigger;
using std_srvs::srv::SetBool;
using climb_msgs::action::StepCommand;
using climb_msgs::srv::KeyInput;
using climb_msgs::srv::ActuatorEnable;
using climb_msgs::srv::SetString;
using climb_msgs::msg::JointCommand;
using climb_msgs::msg::EndEffectorCommand;
using climb_msgs::msg::StepOverrideCommand;
using climb_msgs::msg::TeleopMessage;

/**
 * @brief ROS node that publishes teleop commands
 *
 * Services: key_input
 * Clients: actuator_enable, controller_enable
 * Action clients: step_command
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
   * @brief Enable or disable the actuator
   * @param tokens The command tokens
   * @return The response message
   */
  KeyInputParser::Response enableCommandCallback(
    const std::vector<std::string> & tokens);

  /**
   * @brief Enable or disable the controller
   * @param tokens The command tokens
   * @return The response message
   */
  KeyInputParser::Response controlCommandCallback(
    const std::vector<std::string> & tokens);

  /**
   * @brief Set a joint property
   * @param tokens The command tokens
   * @return The response message
   */
  KeyInputParser::Response setCommandCallback(
    const std::vector<std::string> & tokens);

  /**
   * @brief Go to a specific configuration
   * @param tokens The command tokens
   * @return The response message
   */
  KeyInputParser::Response gotoCommandCallback(
    const std::vector<std::string> & tokens);

  /**
   * @brief Move a joint
   * @param tokens The command tokens
   * @return The response message
   */
  KeyInputParser::Response moveCommandCallback(
    const std::vector<std::string> & tokens);

  /**
   * @brief Move the end effector
   * @param tokens The command tokens
   * @return The response message
   */
  KeyInputParser::Response twistCommandCallback(
    const std::vector<std::string> & tokens);

  /**
   * @brief Take a step
   * @param tokens The command tokens
   * @return The response message
   */
  KeyInputParser::Response stepCommandCallback(
    const std::vector<std::string> & tokens);

  /**
   * @brief Plan a sequence of footholds
   * @param tokens The command tokens
   * @return The response message
   */
  KeyInputParser::Response planCommandCallback(
    const std::vector<std::string> & tokens);

  /**
   * @brief Load a simulated point cloud
   * @param tokens The command tokens
   * @return The response message
   */
  KeyInputParser::Response simulateCommandCallback(
    const std::vector<std::string> & tokens);

  /**
   * @brief Convert key press into a twist
   * @param key The key character (linear: wasdqe, angular: WASDQE)
   * @return The twist vector (linear, angular)
   */
  Eigen::Vector<double, 6> getTwist(char key) const;

  /**
   * @brief Convert key press into a twist in a specific frame
   * @param key The key character (linear: wasdqe, angular: WASDQE)
   * @param frame The name of the twist frame
   * @return The twist vector (linear, angular)
   */
  Eigen::Vector<double, 6> getTwist(char key, const std::string & frame) const;

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

  /**
   * @brief Move a joint by a given displacement
   * @param joint The joint name
   * @param displacement The joint displacement
   */
  void moveJoint(const std::string & joint, double displacement);

  /**
   * @brief Move each joint by a given displacement
   * @param displacements The joint displacements
   */
  void moveJoints(const Eigen::VectorXd & displacements);

  /**
   * @brief Move the end effector by a given displacement
   * @param contact The contact frame name
   * @param twist The twist vector in the contact frame (linear, angular)
   */
  void moveEndEffector(
    const std::string & contact, const Eigen::Vector<double, 6> & twist);

  /**
   * @brief Move the body frame by a given displacement
   * @param twist The twist vector in the body frame (linear, angular)
   */
  void moveBody(const Eigen::Vector<double, 6> & twist);

  /**
   * @brief Set the end effector controller velocity setpoint
   * @param contact The contact frame name
   * @param twist The twist vector in the contact frame (linear, angular)
   */
  void controlEndEffector(
    const std::string & contact, const Eigen::Vector<double, 6> & twist);

  rcl_interfaces::msg::SetParametersResult parameterCallback(
    const std::vector<rclcpp::Parameter> & parameters) override;

  // Key input parser
  KeyInputParser key_input_parser_;
  // Key input service
  rclcpp::Service<KeyInput>::SharedPtr key_input_service_;
  // Asynchronous key response publisher
  rclcpp::Publisher<TeleopMessage>::SharedPtr key_output_pub_;
  // Joint command publisher
  rclcpp::Publisher<JointCommand>::SharedPtr joint_cmd_pub_;
  // End effector command publisher
  rclcpp::Publisher<EndEffectorCommand>::SharedPtr ee_cmd_pub_;
  // Step override command publisher
  rclcpp::Publisher<StepOverrideCommand>::SharedPtr step_override_cmd_pub_;
  // Actuator enable client
  rclcpp::Client<ActuatorEnable>::SharedPtr actuator_enable_client_;
  // Controller enable client
  rclcpp::Client<SetBool>::SharedPtr controller_enable_client_;
  // Plan service client
  rclcpp::Client<Trigger>::SharedPtr plan_client_;
  // Simulate point cloud service client
  rclcpp::Client<SetString>::SharedPtr simulate_client_;
  // Step command action client
  rclcpp_action::Client<StepCommand>::SharedPtr step_cmd_client_;
  // Command callback group
  rclcpp::CallbackGroup::SharedPtr command_callback_group_;
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
  // Controller enabled
  bool controller_enable_;
};

#endif  // TELEOP_NODE_HPP
