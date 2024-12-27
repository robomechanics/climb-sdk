#include "climb_main/step_planner_node.hpp"
#include "climb_main/util/ros_utils.hpp"
#include <geometry_msgs/msg/pose.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using geometry_msgs::msg::Pose;
using GoalHandle = rclcpp_action::ServerGoalHandle<StepCommand>;

StepPlannerNode::StepPlannerNode()
: KinematicsNode("StepPlannerNode")
{
  // Initialize step planner
  step_planner_ = std::make_unique<StepPlanner>(robot_);

  // Declare parameters
  for (const auto & p : step_planner_->getParameters()) {
    this->declare_parameter(p.name, p.default_value, p.descriptor);
  }

  // Initialize subscribers and publishers
  contact_force_sub_ = this->create_subscription<ContactForce>(
    "contact_forces", 1,
    std::bind(&StepPlannerNode::contactForceCallback, this, std::placeholders::_1));
  step_override_cmd_sub_ = this->create_subscription<StepOverrideCommand>(
    "step_override_commands", 1,
    std::bind(&StepPlannerNode::stepOverrideCommandCallback, this, std::placeholders::_1));
  end_effector_cmd_pub_ = this->create_publisher<EndEffectorCommand>("end_effector_commands", 1);
  foothold_pub_ = this->create_publisher<PoseArray>("footholds", 1);
  step_cmd_srv_ = rclcpp_action::create_server<StepCommand>(
    this, "step_command",
    std::bind(&StepPlannerNode::goalCallback, this, _1, _2),
    std::bind(&StepPlannerNode::cancelCallback, this, _1),
    std::bind(&StepPlannerNode::acceptedCallback, this, _1));
  RCLCPP_INFO(this->get_logger(), "Step planner node initialized");
}

void StepPlannerNode::contactForceCallback(const ContactForce::SharedPtr msg)
{
  if (!robot_->isInitialized()) {
    return;
  }
  TransformStamped map_to_body = lookupMapToBodyTransform();
  if (map_to_body.header.frame_id.empty()) {
    return;
  }

  // Update step progress
  step_planner_->update(*msg, map_to_body);
  processStateChanges();

  // Publish end effector command
  auto cmd = step_planner_->getCommand();
  cmd.header.stamp = this->now();
  end_effector_cmd_pub_->publish(cmd);

  // Publish footholds
  PoseArray footholds;
  footholds.header.frame_id = name_ + "/map";
  footholds.header.stamp = this->now();
  for (const auto & frame : robot_->getContactFrames()) {
    footholds.poses.push_back(
      RosUtils::eigenToPose(step_planner_->getFoothold(frame)));
  }
  foothold_pub_->publish(footholds);
}

void StepPlannerNode::stepOverrideCommandCallback(
  const StepOverrideCommand::SharedPtr msg)
{
  Eigen::Vector<double, 6> twist = RosUtils::twistToEigen(msg->offset);
  if (twist.norm()) {
    step_planner_->moveFoothold(msg->header.frame_id, twist);
  }
  switch (msg->command) {
    case StepOverrideCommand::COMMAND_NONE:
      break;
    case StepOverrideCommand::COMMAND_RETRY:
      step_planner_->resume(msg->header.frame_id);
      step_planner_->retry(msg->header.frame_id);
      break;
    case StepOverrideCommand::COMMAND_ADVANCE:
      step_planner_->resume(msg->header.frame_id);
      step_planner_->advance(msg->header.frame_id);
      break;
    case StepOverrideCommand::COMMAND_STOP:
      step_planner_->cancel(msg->header.frame_id);
      break;
    case StepOverrideCommand::COMMAND_PAUSE:
      step_planner_->pause(msg->header.frame_id);
      break;
    case StepOverrideCommand::COMMAND_RESUME:
      step_planner_->resume(msg->header.frame_id);
      break;
  }
}

rclcpp_action::GoalResponse StepPlannerNode::goalCallback(
  [[maybe_unused]] const rclcpp_action::GoalUUID & uuid,
  [[maybe_unused]] std::shared_ptr<const StepCommand::Goal> goal)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse StepPlannerNode::cancelCallback(
  [[maybe_unused]] const std::shared_ptr<GoalHandle> goal_handle)
{
  auto goal = goal_handle->get_goal();
  goal_handles_.erase(goal->frame);
  step_planner_->cancel(goal->frame);
  processStateChanges();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void StepPlannerNode::acceptedCallback(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  auto goal = goal_handle->get_goal();
  goal_handles_[goal->frame] = goal_handle;
  if (goal->default_foothold) {
    step_planner_->step(goal->frame);
  } else {
    step_planner_->step(goal->frame, RosUtils::poseToEigen(goal->foothold));
  }
}

void StepPlannerNode::processStateChanges()
{
  auto changes = step_planner_->getStateChanges();
  while (!changes.empty()) {
    auto change = changes.front();
    changes.pop();
    auto handle = goal_handles_.find(change.contact);
    if (handle != goal_handles_.end()) {
      auto feedback = std::make_shared<StepCommand::Feedback>();
      feedback->state = static_cast<uint8_t>(change.state);
      handle->second->publish_feedback(feedback);
      if (change.state == StepPlanner::State::STANCE) {
        auto result = std::make_shared<StepCommand::Result>();
        result->success = true;
        handle->second->succeed(result);
        goal_handles_.erase(handle);
      }
    }
  }
}

rcl_interfaces::msg::SetParametersResult StepPlannerNode::parameterCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  auto result = KinematicsNode::parameterCallback(parameters);
  for (const auto & parameter : parameters) {
    step_planner_->setParameter(parameter, result);
  }
  return result;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StepPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
