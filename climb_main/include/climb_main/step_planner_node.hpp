#ifndef STEP_PLANNER_NODE_HPP
#define STEP_PLANNER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <climb_msgs/msg/contact_force.hpp>
#include <climb_msgs/msg/end_effector_command.hpp>
#include <climb_msgs/msg/step_override_command.hpp>
#include <climb_msgs/action/step_command.hpp>
#include "climb_main/kinematics/kinematics_node.hpp"
#include "climb_main/step_planner/step_planner.hpp"

using geometry_msgs::msg::PoseArray;
using climb_msgs::msg::ContactForce;
using climb_msgs::msg::EndEffectorCommand;
using climb_msgs::msg::StepOverrideCommand;
using climb_msgs::action::StepCommand;

/**
 * @brief ROS node that publishes controller setpoints to execute a step
 *
 * Subscribers: contact_forces, joint_states, robot_description
 * Publishers: end_effector_commands
 * Action services: step_command
 */
class StepPlannerNode : public KinematicsNode
{
public:
  /**
   * @brief Constructor for StepPlannerNode
   */
  StepPlannerNode();

private:
  /**
   * @brief Update step planner with latest contact forces
   * @param[in] msg Message containing end effector command
   */
  void contactForceCallback(const ContactForce::SharedPtr msg);

  /**
   * @brief Adjust current step progress and foothold location
   * @param[in] msg Message containing step override command
   */
  void stepOverrideCommandCallback(const StepOverrideCommand::SharedPtr msg);

  /**
   * @brief Callback for step command initial request
   */
  rclcpp_action::GoalResponse goalCallback(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const StepCommand::Goal> goal);

  /**
   * @brief Callback for step command cancel request
   */
  rclcpp_action::CancelResponse cancelCallback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<StepCommand>>
    goal_handle);

  /**
   * @brief Callback to begin step command
   */
  void acceptedCallback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<StepCommand>>
    goal_handle);

  rcl_interfaces::msg::SetParametersResult parameterCallback(
    const std::vector<rclcpp::Parameter> & parameters) override;

  // Step planner
  std::unique_ptr<StepPlanner> step_planner_;
  // Subscriber for contact forces
  rclcpp::Subscription<ContactForce>::SharedPtr contact_force_sub_;
  // Subscriber for step override commands
  rclcpp::Subscription<StepOverrideCommand>::SharedPtr step_override_cmd_sub_;
  // Publisher for end effector commands
  rclcpp::Publisher<EndEffectorCommand>::SharedPtr end_effector_cmd_pub_;
  // Publisher for current footholds
  rclcpp::Publisher<PoseArray>::SharedPtr foothold_pub_;
  // Action server for step commands
  rclcpp_action::Server<StepCommand>::SharedPtr step_cmd_srv_;
  // Goal handles for step commands
  std::unordered_map<std::string,
    std::shared_ptr<rclcpp_action::ServerGoalHandle<StepCommand>>>
  goal_handles_;
};

#endif  // STEP_PLANNER_NODE_HPP
