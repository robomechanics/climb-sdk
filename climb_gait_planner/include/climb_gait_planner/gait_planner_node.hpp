#ifndef CLIMB_GAIT_PLANNER__GAIT_PLANNER_NODE_HPP_
#define CLIMB_GAIT_PLANNER__GAIT_PLANNER_NODE_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <climb_msgs/msg/contact_force.hpp>
#include <climb_msgs/msg/controller_command.hpp>
#include <climb_msgs/msg/footstep_update.hpp>
#include <climb_msgs/action/footstep_command.hpp>
#include <climb_kinematics/kinematics_node.hpp>
#include "climb_gait_planner/gait_planner.hpp"

using geometry_msgs::msg::PoseArray;
using climb_msgs::msg::ContactForce;
using climb_msgs::msg::ControllerCommand;
using climb_msgs::msg::FootstepUpdate;
using climb_msgs::action::FootstepCommand;

/**
 * @brief ROS node that publishes controller setpoints to execute a step
 *
 * Subscribers: contact_forces, joint_states, robot_description
 * Publishers: controller_commands
 * Action services: footstep_command
 */
class GaitPlannerNode : public KinematicsNode
{
public:
  /**
   * @brief Constructor for GaitPlannerNode
   */
  GaitPlannerNode();

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
  void FootstepUpdateCallback(const FootstepUpdate::SharedPtr msg);

  /**
   * @brief Callback for step command initial request
   */
  rclcpp_action::GoalResponse goalCallback(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FootstepCommand::Goal> goal);

  /**
   * @brief Callback for step command cancel request
   */
  rclcpp_action::CancelResponse cancelCallback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<FootstepCommand>>
    goal_handle);

  /**
   * @brief Callback to begin step command
   */
  void acceptedCallback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<FootstepCommand>>
    goal_handle);

  /**
   * @brief Send state changes as feedback to the appropriate step command goal
   * handles
   */
  void processStateChanges();

  rcl_interfaces::msg::SetParametersResult parameterCallback(
    const std::vector<rclcpp::Parameter> & parameters) override;

  // Step planner
  std::unique_ptr<GaitPlanner> gait_planner_;
  // Subscriber for contact forces
  rclcpp::Subscription<ContactForce>::SharedPtr contact_force_sub_;
  // Subscriber for step override commands
  rclcpp::Subscription<FootstepUpdate>::SharedPtr step_override_cmd_sub_;
  // Publisher for end effector commands
  rclcpp::Publisher<ControllerCommand>::SharedPtr end_effector_cmd_pub_;
  // Publisher for current footholds
  rclcpp::Publisher<PoseArray>::SharedPtr foothold_pub_;
  // Action server for step commands
  rclcpp_action::Server<FootstepCommand>::SharedPtr step_cmd_srv_;
  // Goal handles for step commands
  std::unordered_map<std::string,
    std::shared_ptr<rclcpp_action::ServerGoalHandle<FootstepCommand>>>
  goal_handles_;
};

#endif  // CLIMB_GAIT_PLANNER__GAIT_PLANNER_NODE_HPP_
