#include "climb_gait_planner/gait_planner_node.hpp"

#include <geometry_msgs/msg/pose.hpp>

#include <climb_util/ros_utils.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using geometry_msgs::msg::Pose;
using GoalHandle = rclcpp_action::ServerGoalHandle<FootstepCommand>;

GaitPlannerNode::GaitPlannerNode()
: KinematicsNode("GaitPlannerNode"),
  gait_planner_(std::make_unique<GaitPlanner>(robot_))
{
  // Declare parameters
  for (const auto & p : gait_planner_->getParameters()) {
    declare_parameter(p.name, p.default_value, p.descriptor);
  }

  // Initialize subscribers and publishers
  contact_force_sub_ = create_subscription<ContactForce>(
    "contact_forces", 1,
    std::bind(&GaitPlannerNode::contactForceCallback, this, _1));
  step_override_cmd_sub_ = create_subscription<FootstepUpdate>(
    "footstep_updates", 1,
    std::bind(&GaitPlannerNode::footstepUpdateCallback, this, _1));
  controller_cmd_pub_ =
    create_publisher<ControllerCommand>("controller_commands", 1);
  foothold_pub_ = create_publisher<PoseArray>("footholds", 1);
  step_cmd_srv_ = rclcpp_action::create_server<FootstepCommand>(
    this, "footstep_command",
    std::bind(&GaitPlannerNode::goalCallback, this, _1, _2),
    std::bind(&GaitPlannerNode::cancelCallback, this, _1),
    std::bind(&GaitPlannerNode::acceptedCallback, this, _1));
  RCLCPP_INFO(get_logger(), "Step planner node initialized");
}

void GaitPlannerNode::contactForceCallback(const ContactForce::SharedPtr msg)
{
  if (!robot_->isInitialized()) {
    return;
  }

  // Update step progress
  gait_planner_->update(*msg, lookupTransform("/map", robot_->getBodyFrame()));
  processStateChanges();

  // Publish end effector command
  auto cmd = gait_planner_->getCommand();
  cmd.header.stamp = now();
  if (goal_handle_) {
    cmd.header.frame_id = goal_handle_->get_goal()->footstep.header.frame_id;
    cmd.body = goal_handle_->get_goal()->footstep.body;
    cmd.overrides = goal_handle_->get_goal()->footstep.overrides;
  } else {
    cmd.overrides.name.push_back("spine_joint");
    cmd.overrides.position.push_back(0.0);
  }
  controller_cmd_pub_->publish(cmd);

  // Publish footholds
  PoseArray footholds;
  footholds.header.frame_id = getPrefixedFrameId("/map");
  footholds.header.stamp = now();
  for (const auto & frame : robot_->getContactFrames()) {
    footholds.poses.push_back(
      RosUtils::eigenToPose(gait_planner_->getFoothold(frame)));
  }
  foothold_pub_->publish(footholds);
}

void GaitPlannerNode::footstepUpdateCallback(
  const FootstepUpdate::SharedPtr msg)
{
  Eigen::Vector<double, 6> twist = RosUtils::twistToEigen(msg->offset);
  if (twist.norm()) {
    gait_planner_->moveFoothold(msg->header.frame_id, twist);
  }
  switch (msg->command) {
    case FootstepUpdate::COMMAND_NONE:
      break;
    case FootstepUpdate::COMMAND_RETRY:
      gait_planner_->resume(msg->header.frame_id);
      gait_planner_->retry(msg->header.frame_id);
      break;
    case FootstepUpdate::COMMAND_ADVANCE:
      gait_planner_->resume(msg->header.frame_id);
      gait_planner_->advance(msg->header.frame_id);
      break;
    case FootstepUpdate::COMMAND_STOP:
      gait_planner_->cancel(msg->header.frame_id);
      break;
    case FootstepUpdate::COMMAND_PAUSE:
      gait_planner_->pause(msg->header.frame_id);
      break;
    case FootstepUpdate::COMMAND_RESUME:
      gait_planner_->resume(msg->header.frame_id);
      break;
  }
}

rclcpp_action::GoalResponse GaitPlannerNode::goalCallback(
  [[maybe_unused]] const rclcpp_action::GoalUUID & uuid,
  [[maybe_unused]] std::shared_ptr<const FootstepCommand::Goal> goal)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GaitPlannerNode::cancelCallback(
  [[maybe_unused]] const std::shared_ptr<GoalHandle> goal_handle)
{
  auto goal = goal_handle->get_goal();
  goal_handle_ = nullptr;
  for (const auto & frame : goal->footstep.frames) {
    gait_planner_->cancel(frame);
  }
  processStateChanges();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GaitPlannerNode::acceptedCallback(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  auto footstep = goal_handle->get_goal()->footstep;
  goal_handle_ = goal_handle;
  for (size_t i = 0; i < footstep.frames.size(); i++) {
    auto frame = footstep.frames[i];
    if (footstep.footholds.size() <= i) {
      gait_planner_->step(frame);
    } else {
      gait_planner_->step(frame, RosUtils::poseToEigen(footstep.footholds[i]));
    }
  }
}

void GaitPlannerNode::processStateChanges()
{
  auto changes = gait_planner_->getStateChanges();
  if (!goal_handle_ || changes.empty()) {
    return;
  }
  while (!changes.empty()) {
    auto change = changes.front();
    changes.pop();
    auto feedback = std::make_shared<FootstepCommand::Feedback>();
    feedback->state = static_cast<uint8_t>(change.state);
    goal_handle_->publish_feedback(feedback);
  }
  for (const auto & contact : goal_handle_->get_goal()->footstep.frames) {
    if (gait_planner_->getState(contact) != GaitPlanner::State::STANCE) {
      return;
    }
  }
  auto result = std::make_shared<FootstepCommand::Result>();
  result->success = true;
  goal_handle_->succeed(result);
  goal_handle_ = nullptr;
}

rcl_interfaces::msg::SetParametersResult GaitPlannerNode::parameterCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  auto result = KinematicsNode::parameterCallback(parameters);
  for (const auto & parameter : parameters) {
    gait_planner_->setParameter(parameter, result);
  }
  return result;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GaitPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
