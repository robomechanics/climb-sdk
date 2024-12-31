#include "climb_teleop/teleop_node.hpp"
#include <rclcpp/executors.hpp>
#include <fmt/core.h>
#include <climb_util/ros_utils.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

TeleopNode::TeleopNode()
: KinematicsNode("TeleopNode")
{
  // Load poses from parameters
  const auto & parameters =
    this->get_node_parameters_interface()->get_parameter_overrides();
  for (const auto & [name, parameter] : parameters) {
    std::string prefix = "configurations.";
    if (name.compare(0, prefix.size(), prefix) == 0) {
      configurations_[name.substr(prefix.size())] =
        parameter.get<std::vector<double>>();
    }
  }
  // Define parameters
  this->declare_parameter("joint_step", 0.02);    // rad
  this->declare_parameter("linear_step", 0.002);  // m
  this->declare_parameter("angular_step", 0.02);  // rad
  // Define commands
  addCommands();
  // Create separate callback group to handle key commands in their own thread
  command_callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  // Initialize service, clients, and publishers
  key_input_service_ = this->create_service<KeyInput>(
    "key_input", std::bind(&TeleopNode::keyCallback, this, _1, _2),
    rclcpp::ServicesQoS(), command_callback_group_);
  key_output_pub_ = this->create_publisher<TeleopMessage>("key_output", 1);
  actuator_enable_client_ =
    this->create_client<ActuatorEnable>("actuator_enable");
  controller_enable_client_ =
    this->create_client<SetBool>("controller_enable");
  plan_client_ = this->create_client<Trigger>("plan");
  joint_cmd_pub_ = this->create_publisher<JointCommand>("joint_commands", 1);
  ee_cmd_pub_ = this->create_publisher<EndEffectorCommand>(
    "end_effector_commands", 1);
  step_override_cmd_pub_ = this->create_publisher<StepOverrideCommand>(
    "step_override_commands", 1);
  step_cmd_client_ = rclcpp_action::create_client<StepCommand>(this, "step_command");
  RCLCPP_INFO(this->get_logger(), "Teleop node initialized");
}

void TeleopNode::addCommands()
{
  // Custom tokens
  key_input_parser_.defineToken(
    "JOINT", [this]() {
      return robot_->getJointNames();
    });
  key_input_parser_.defineToken(
    "BODY", [this]() {
      return std::vector<std::string>{robot_->getBodyFrame()};
    });
  key_input_parser_.defineToken(
    "CONTACT", [this]() {
      return robot_->getContactFrames();
    });
  key_input_parser_.defineToken(
    "CONFIGURATION", [this]() {
      std::vector<std::string> poses;
      poses.reserve(configurations_.size());
      for (const auto & [name, _] : configurations_) {
        poses.push_back(name);
      }
      return poses;
    });
  // Enable motors
  this->key_input_parser_.defineCommand(
    "[enable|disable]",
    std::bind(&TeleopNode::enableCommandCallback, this, _1));
  // Enable controller
  this->key_input_parser_.defineCommand(
    "control [on|off]",
    std::bind(&TeleopNode::controlCommandCallback, this, _1));
  // Set joint state
  key_input_parser_.defineCommand(
    "set JOINT [position|velocity|effort] DOUBLE",
    std::bind(&TeleopNode::setCommandCallback, this, _1));
  // Go to configuration
  key_input_parser_.defineCommand(
    "goto CONFIGURATION",
    std::bind(&TeleopNode::gotoCommandCallback, this, _1));
  // Move joint
  key_input_parser_.defineCommand(
    "move JOINT",
    std::bind(&TeleopNode::moveCommandCallback, this, _1));
  // Twist contact frame
  key_input_parser_.defineCommand(
    "twist [CONTACT|BODY]",
    std::bind(&TeleopNode::twistCommandCallback, this, _1));
  // Take a step
  key_input_parser_.defineCommand(
    "step CONTACT",
    std::bind(&TeleopNode::stepCommandCallback, this, _1));
  // Plan footsteps
  key_input_parser_.defineCommand(
    "plan",
    std::bind(&TeleopNode::planCommandCallback, this, _1));
}

void TeleopNode::keyCallback(
  const std::shared_ptr<KeyInput::Request> request,
  std::shared_ptr<KeyInput::Response> response)
{
  auto result = key_input_parser_.processKeyInput(
    request->input, request->realtime, request->autocomplete);
  response->response = result.response;
  response->realtime = result.realtime;
}

KeyInputParser::Response TeleopNode::enableCommandCallback(
  const std::vector<std::string> & tokens)
{
  auto request = std::make_shared<ActuatorEnable::Request>();
  request->enable = tokens.at(0) == "enable";
  auto result = actuator_enable_client_->async_send_request(request);
  auto status = result.wait_for(std::chrono::milliseconds(100));
  if (status == std::future_status::ready) {
    auto response = result.get();
    if (response->success) {
      return KeyInputParser::Response{
        request->enable ? "Enabled" : "Disabled", false};
    } else {
      return KeyInputParser::Response{response->message, false};
    }
  }
  return KeyInputParser::Response{"Robot driver node is not responding", false};
}

KeyInputParser::Response TeleopNode::controlCommandCallback(
  const std::vector<std::string> & tokens)
{
  auto request = std::make_shared<SetBool::Request>();
  request->data = tokens.at(1) == "on";
  auto result = controller_enable_client_->async_send_request(request);
  auto status = result.wait_for(std::chrono::milliseconds(100));
  if (status == std::future_status::ready) {
    auto response = result.get();
    if (response->success) {
      return KeyInputParser::Response{
        request->data ? "Control on" : "Control off", false};
    } else {
      return KeyInputParser::Response{response->message, false};
    }
  }
  return KeyInputParser::Response{"Controller node is not responding", false};
}

KeyInputParser::Response TeleopNode::setCommandCallback(
  const std::vector<std::string> & tokens)
{
  setJoint(tokens.at(1), tokens.at(2), std::stod(tokens.at(3)));
  auto response = fmt::format(
    "{}: {} = {}", tokens.at(1), tokens.at(2), tokens.at(3));
  return KeyInputParser::Response{response, false};
}

KeyInputParser::Response TeleopNode::gotoCommandCallback(
  const std::vector<std::string> & tokens)
{
  setConfiguration(
    robot_->getJointNames(), configurations_.at(tokens.at(1)));
  return KeyInputParser::Response{"Configuration: " + tokens.at(1), false};
}

KeyInputParser::Response TeleopNode::moveCommandCallback(
  const std::vector<std::string> & tokens)
{
  key_input_parser_.setInputCallback(
    [this, tokens](char key) {
      switch (key) {
        case ' ':
          return KeyInputParser::Response{"Stopped", false};
        case 'w':
          moveJoint(tokens.at(1), joint_step_);
          break;
        case 's':
          moveJoint(tokens.at(1), -joint_step_);
          break;
      }
      return KeyInputParser::Response{"", true};
    });
  joint_setpoints_ = robot_->getJointPosition();
  auto response = fmt::format(
    "{}: Move with w/s keys, press space to stop", tokens.at(1));
  return KeyInputParser::Response{response, true};
}

KeyInputParser::Response TeleopNode::twistCommandCallback(
  const std::vector<std::string> & tokens)
{
  StepOverrideCommand command;
  command.header.stamp = this->now();
  command.header.frame_id = tokens.at(1);
  command.command = StepOverrideCommand::COMMAND_PAUSE;
  step_override_cmd_pub_->publish(command);
  key_input_parser_.setInputCallback(
    [this, tokens](char key) {
      if (key == ' ') {
        if (controller_enable_) {
          controlEndEffector(tokens.at(1), Eigen::Vector<double, 6>::Zero());
        }
        StepOverrideCommand command;
        command.header.stamp = this->now();
        command.header.frame_id = tokens.at(1);
        command.command = StepOverrideCommand::COMMAND_RESUME;
        step_override_cmd_pub_->publish(command);
        return KeyInputParser::Response{"Stopped", false};
      }
      if (controller_enable_) {
        controlEndEffector(tokens.at(1), getTwist(key, tokens.at(1)));
      } else if (tokens.at(1) == robot_->getBodyFrame()) {
        moveBody(getTwist(key));
      } else {
        moveEndEffector(tokens.at(1), getTwist(key, tokens.at(1)));
      }
      return KeyInputParser::Response{"", true};
    });
  joint_setpoints_ = robot_->getJointPosition();
  auto response = fmt::format(
    "{}: Linear with w/a/s/d/q/e keys, angular with W/A/S/D/Q/E keys, "
    "press space to stop", tokens.at(1));
  return KeyInputParser::Response{response, true};
}

KeyInputParser::Response TeleopNode::stepCommandCallback(
  const std::vector<std::string> & tokens)
{
  auto goal = StepCommand::Goal();
  goal.frame = tokens.at(1);
  goal.header.stamp = this->now();
  goal.default_foothold = true;

  auto options = rclcpp_action::Client<StepCommand>::SendGoalOptions();
  options.feedback_callback =
    [this, goal](
    auto, const std::shared_ptr<const StepCommand::Feedback> feedback)
    {
      std::string state = "Invalid state";
      switch (feedback->state) {
        case StepCommand::Feedback::STATE_STANCE:
          state = "Stance";
          break;
        case StepCommand::Feedback::STATE_DISENGAGE:
          state = "Disengage";
          break;
        case StepCommand::Feedback::STATE_LIFT:
          state = "Lift";
          break;
        case StepCommand::Feedback::STATE_SWING:
          state = "Swing";
          break;
        case StepCommand::Feedback::STATE_PLACE:
          state = "Place";
          break;
        case StepCommand::Feedback::STATE_ENGAGE:
          state = "Engage";
          break;
        case StepCommand::Feedback::STATE_SNAG:
          state = "Snag";
          break;
        case StepCommand::Feedback::STATE_SLIP:
          state = "Slip";
          break;
        case StepCommand::Feedback::STATE_RETRY:
          state = "Retry";
          break;
        case StepCommand::Feedback::STATE_STOP:
          state = "Stop";
          break;
      }
      TeleopMessage msg;
      msg.response = fmt::format("{} state: {}", goal.frame, state);
      msg.realtime = true;
      key_output_pub_->publish(msg);
    };
  auto goal_handle_future = step_cmd_client_->async_send_goal(goal, options);
  auto goal_handle_status =
    goal_handle_future.wait_for(std::chrono::milliseconds(100));
  if (goal_handle_status != std::future_status::ready) {
    return KeyInputParser::Response{
      "Step planner node is not responding", false};
  }
  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    return KeyInputParser::Response{
      "Step planner node rejected the requested step", false};
  }
  auto result_future = step_cmd_client_->async_get_result(
    goal_handle, [this](const auto & result) {
      TeleopMessage msg;
      if (result.result->success) {
        msg.response = "Completed step";
      } else {
        msg.response = "Aborted step";
      }
      msg.realtime = false;
      key_output_pub_->publish(msg);
    });
  key_input_parser_.setInputCallback(
    [this, goal, goal_handle, result_future](char key) {
      StepOverrideCommand command;
      command.header.stamp = this->now();
      command.header.frame_id = goal.frame;
      if (key == ' ') {
        step_cmd_client_->async_cancel_goal(goal_handle);
        return KeyInputParser::Response{"", true};
      } else if (key == '.') {
        command.command = StepOverrideCommand::COMMAND_ADVANCE;
        step_override_cmd_pub_->publish(command);
        return KeyInputParser::Response{"Advancing Step", true};
      } else if (key == ',') {
        command.command = StepOverrideCommand::COMMAND_RETRY;
        step_override_cmd_pub_->publish(command);
        return KeyInputParser::Response{"Retrying Step", true};
      } else {
        command.offset = RosUtils::eigenToTwist(getTwist(key));
        step_override_cmd_pub_->publish(command);
        return KeyInputParser::Response{"", true};
      }
    });
  return KeyInputParser::Response{
    fmt::format("Taking step {}", goal.frame), true};
}

KeyInputParser::Response TeleopNode::planCommandCallback(
  const std::vector<std::string> & tokens [[maybe_unused]])
{
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto result = plan_client_->async_send_request(request);
  auto status = result.wait_for(std::chrono::milliseconds(100));
  if (status == std::future_status::ready) {
    auto response = result.get();
    if (response->success) {
      return KeyInputParser::Response{"Planned footsteps", false};
    } else {
      return KeyInputParser::Response{response->message, false};
    }
  }
  return KeyInputParser::Response{
    "Footstep planner node is not responding", false};
}

Eigen::Vector<double, 6> TeleopNode::getTwist(char key) const
{
  Eigen::Vector<double, 6> twist = Eigen::Vector<double, 6>::Zero();
  switch (key) {
    case 'w':
      twist[0] = linear_step_;
      break;
    case 's':
      twist[0] = -linear_step_;
      break;
    case 'a':
      twist[1] = linear_step_;
      break;
    case 'd':
      twist[1] = -linear_step_;
      break;
    case 'q':
      twist[2] = linear_step_;
      break;
    case 'e':
      twist[2] = -linear_step_;
      break;
    case 'W':
      twist[4] = angular_step_;
      break;
    case 'S':
      twist[4] = -angular_step_;
      break;
    case 'A':
      twist[3] = -angular_step_;
      break;
    case 'D':
      twist[3] = angular_step_;
      break;
    case 'Q':
      twist[5] = angular_step_;
      break;
    case 'E':
      twist[5] = -angular_step_;
      break;
  }
  return twist;
}

Eigen::Vector<double, 6> TeleopNode::getTwist(
  char key, const std::string & frame) const
{
  Eigen::Vector<double, 6> twist = getTwist(key);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = robot_->getTransform(frame).rotation().transpose();
  Eigen::MatrixXd Ad = robot_->getAdjoint(transform);
  return Ad * twist;
}

void TeleopNode::setJoint(
  const std::string & joint, const std::string & property, double value)
{
  JointCommand command;
  std::vector<std::string> joints =
    joint.empty() ? robot_->getJointNames() : std::vector<std::string>{joint};
  for (const auto & joint : joints) {
    command.name.push_back(joint);
    command.mode.push_back(JointCommand::MODE_POSITION);
    if (property == "position") {
      command.position.push_back(value);
    } else if (property == "velocity") {
      command.velocity.push_back(value);
    } else if (property == "effort") {
      command.effort.push_back(value);
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid property: %s", property.c_str());
      return;
    }
  }
  robot_->clampJointCommand(command);
  joint_cmd_pub_->publish(command);
}

void TeleopNode::setConfiguration(
  const std::vector<std::string> & joints,
  const std::vector<double> & positions)
{
  JointCommand command;
  command.name = joints;
  command.mode = std::vector<uint8_t>(joints.size(), JointCommand::MODE_POSITION);
  command.position = positions;
  robot_->clampJointCommand(command);
  joint_cmd_pub_->publish(command);
}

void TeleopNode::moveJoint(
  const std::string & joint, double displacement)
{
  auto j = robot_->getJointIndex(joint);
  joint_setpoints_[j] += displacement;
  JointCommand command;
  command.name.push_back(joint);
  command.mode.push_back(JointCommand::MODE_POSITION);
  command.position.push_back(joint_setpoints_[j]);
  robot_->clampJointCommand(command);
  joint_cmd_pub_->publish(command);
}

void TeleopNode::moveJoints(const Eigen::VectorXd & displacements)
{
  joint_setpoints_ += displacements;
  JointCommand command;
  for (auto j = 0; j < robot_->getNumJoints(); j++) {
    command.name.push_back(robot_->getJointNames().at(j));
    command.mode.push_back(JointCommand::MODE_POSITION);
    command.position.push_back(joint_setpoints_[j]);
  }
  robot_->clampJointCommand(command);
  joint_cmd_pub_->publish(command);
}

void TeleopNode::moveEndEffector(
  const std::string & contact, const Eigen::Vector<double, 6> & twist)
{
  Eigen::MatrixXd Jh = robot_->getHandJacobian();
  Eigen::MatrixXd B = robot_->getWrenchBasis(contact);
  Eigen::VectorXd all_twists =
    Eigen::VectorXd::Zero(robot_->getNumConstraints());
  int ind = 0;
  for (const auto & frame : robot_->getContactFrames()) {
    int c = robot_->getNumConstraints(frame);
    if (frame == contact) {
      all_twists.segment(ind, c) = B.transpose() * twist;
    }
    ind += c;
  }
  Eigen::VectorXd displacements =
    Jh.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(all_twists);
  moveJoints(displacements);
}

void TeleopNode::moveBody(const Eigen::Vector<double, 6> & twist)
{
  Eigen::MatrixXd Jh = robot_->getHandJacobian();
  Eigen::MatrixXd Gs = -robot_->getGraspMap();
  Eigen::VectorXd displacements =
    Jh.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(
    Gs.transpose() * twist);
  moveJoints(displacements);
}

void TeleopNode::controlEndEffector(
  const std::string & contact, const Eigen::Vector<double, 6> & twist)
{
  if (contact == robot_->getBodyFrame()) {
    return;
  }
  EndEffectorCommand command;
  command.header.stamp = this->now();
  command.frame = {contact};
  command.mode = {EndEffectorCommand::MODE_FREE};
  command.twist = {RosUtils::eigenToTwist(twist)};
  ee_cmd_pub_->publish(command);
}

rcl_interfaces::msg::SetParametersResult TeleopNode::parameterCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  auto result = KinematicsNode::parameterCallback(parameters);
  for (const auto & parameter : parameters) {
    if (parameter.get_name() == "joint_step") {
      joint_step_ = parameter.as_double();
    } else if (parameter.get_name() == "linear_step") {
      linear_step_ = parameter.as_double();
    } else if (parameter.get_name() == "angular_step") {
      angular_step_ = parameter.as_double();
    }
  }
  return result;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto teleop_node = std::make_shared<TeleopNode>();
  executor.add_node(teleop_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
