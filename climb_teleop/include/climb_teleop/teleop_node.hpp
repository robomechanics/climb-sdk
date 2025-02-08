#ifndef CLIMB_TELEOP__TELEOP_NODE_HPP_
#define CLIMB_TELEOP__TELEOP_NODE_HPP_

#include <Eigen/Geometry>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <climb_msgs/action/footstep_command.hpp>
#include <climb_msgs/msg/controller_command.hpp>
#include <climb_msgs/msg/footstep.hpp>
#include <climb_msgs/msg/footstep_plan.hpp>
#include <climb_msgs/msg/joint_command.hpp>
#include <climb_msgs/msg/footstep_update.hpp>
#include <climb_msgs/msg/teleop_output.hpp>
#include <climb_msgs/srv/actuator_enable.hpp>
#include <climb_msgs/srv/teleop_input.hpp>
#include <climb_msgs/srv/set_string.hpp>
#include <climb_kinematics/kinematics_node.hpp>
#include "climb_teleop/key_input_parser.hpp"

using std_srvs::srv::Trigger;
using std_srvs::srv::SetBool;
using climb_msgs::action::FootstepCommand;
using climb_msgs::srv::TeleopInput;
using climb_msgs::srv::ActuatorEnable;
using climb_msgs::srv::SetString;
using climb_msgs::msg::Footstep;
using climb_msgs::msg::FootstepPlan;
using climb_msgs::msg::JointCommand;
using climb_msgs::msg::ControllerCommand;
using climb_msgs::msg::FootstepUpdate;
using climb_msgs::msg::TeleopOutput;

/**
 * @brief ROS node that publishes teleop commands
 *
 * Services: teleop_input
 * Clients: actuator_enable, controller_enable
 * Action clients: footstep_command
 * Subscribers: joint_states, robot_description
 * Publishers: joint_commands, controller_commands
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
    const std::shared_ptr<TeleopInput::Request> request,
    std::shared_ptr<TeleopInput::Response> response);

  /**
   * @brief Callback for footstep plan message
   * @param msg The footstep plan message
   */
  void planCallback(const FootstepPlan::SharedPtr msg);

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
   * @brief Execute the most recent foothold plan
   * @param tokens The command tokens
   * @return The response message
   */
  KeyInputParser::Response executeCommandCallback(
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
   * @brief Convert FootstepCommand feedback state to display name
   * @param state The feedback state
   * @return The display name
   */
  std::string getStateName(int state) const;

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

  /**
   * @brief Execute a footstep command
   * @param footstep The footstep command
   * @param result_callback The callback function after completion/termination
   * @return The response message
   */
  KeyInputParser::Response takeStep(
    const Footstep & footstep,
    rclcpp_action::Client<FootstepCommand>::ResultCallback result_callback);

  rcl_interfaces::msg::SetParametersResult parameterCallback(
    const std::vector<rclcpp::Parameter> & parameters) override;

  // Key input parser
  KeyInputParser key_input_parser_;
  // Key input service
  rclcpp::Service<TeleopInput>::SharedPtr teleop_input_service_;
  // Asynchronous key response publisher
  rclcpp::Publisher<TeleopOutput>::SharedPtr teleop_output_pub_;
  // Joint command publisher
  rclcpp::Publisher<JointCommand>::SharedPtr joint_cmd_pub_;
  // End effector command publisher
  rclcpp::Publisher<ControllerCommand>::SharedPtr controller_cmd_pub_;
  // Step override command publisher
  rclcpp::Publisher<FootstepUpdate>::SharedPtr step_override_cmd_pub_;
  // Plan subscriber
  rclcpp::Subscription<FootstepPlan>::SharedPtr plan_sub_;
  // Actuator enable client
  rclcpp::Client<ActuatorEnable>::SharedPtr actuator_enable_client_;
  // Controller enable client
  rclcpp::Client<SetBool>::SharedPtr controller_enable_client_;
  // Plan service client
  rclcpp::Client<Trigger>::SharedPtr plan_client_;
  // Simulate point cloud service client
  rclcpp::Client<SetString>::SharedPtr simulate_client_;
  // Step command action client
  rclcpp_action::Client<FootstepCommand>::SharedPtr step_cmd_client_;
  // Command callback group
  rclcpp::CallbackGroup::SharedPtr command_callback_group_;
  // Most recent plan
  FootstepPlan::SharedPtr plan_;
  // Current step index of plan
  size_t step_index_ = 0;
  // Poses
  std::unordered_map<std::string, std::vector<double>> configurations_;
  // Joint setpoints
  Eigen::VectorXd joint_setpoints_;
  // Joint step in rad
  double joint_step_;
  // Twist linear step in m
  double linear_step_;
  // Twist angular step in rad
  double angular_step_;
  // Controller enabled
  bool controller_enable_ = false;
};

#endif  // CLIMB_TELEOP__TELEOP_NODE_HPP_
