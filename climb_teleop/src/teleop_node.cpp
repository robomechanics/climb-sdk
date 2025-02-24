#include "climb_teleop/teleop_node.hpp"

#include <fmt/core.h>
#include <rclcpp/executors.hpp>

#include <climb_util/eigen_utils.hpp>
#include <climb_util/ros_utils.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;
using Response = KeyInputParser::Response;

TeleopNode::TeleopNode()
: KinematicsNode("TeleopNode")
{
  // Load poses from parameters
  const auto & parameters =
    get_node_parameters_interface()->get_parameter_overrides();
  for (const auto & [name, parameter] : parameters) {
    std::string prefix = "configurations.";
    if (name.compare(0, prefix.size(), prefix) == 0) {
      configurations_[name.substr(prefix.size())] =
        parameter.get<std::vector<double>>();
    }
  }
  // Define parameters
  declare_parameter("joint_step", 0.02);    // rad
  declare_parameter("linear_step", 0.002);  // m
  declare_parameter("angular_step", 0.02);  // rad
  // Define commands
  addCommands();
  // Create separate callback group to handle key commands in their own thread
  command_callback_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  // Initialize service, clients, and publishers
  teleop_input_service_ = create_service<TeleopInput>(
    "teleop_input", std::bind(&TeleopNode::keyCallback, this, _1, _2),
    rclcpp::ServicesQoS(), command_callback_group_);
  teleop_output_pub_ = create_publisher<TeleopOutput>("teleop_output", 1);
  plan_sub_ = create_subscription<FootstepPlan>(
    "footstep_plan", 1, std::bind(&TeleopNode::planCallback, this, _1));
  actuator_enable_client_ = create_client<ActuatorEnable>("actuator_enable");
  controller_enable_client_ = create_client<SetBool>("controller_enable");
  plan_client_ = create_client<Trigger>("plan");
  simulate_client_ = create_client<SetString>("simulate");
  joint_cmd_pub_ = create_publisher<JointCommand>("joint_commands", 1);
  controller_cmd_pub_ =
    create_publisher<ControllerCommand>("controller_commands", 1);
  step_override_cmd_pub_ =
    create_publisher<FootstepUpdate>("footstep_updates", 1);
  step_cmd_client_ =
    rclcpp_action::create_client<FootstepCommand>(this, "footstep_command");
  RCLCPP_INFO(get_logger(), "Teleop node initialized");
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
  key_input_parser_.defineCommand(
    "[enable|disable]",
    std::bind(&TeleopNode::enableCommandCallback, this, _1));
  // Enable controller
  key_input_parser_.defineCommand(
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
  key_input_parser_.defineCommand(
    "execute",
    std::bind(&TeleopNode::executeCommandCallback, this, _1));
  // Load simulated point cloud
  key_input_parser_.defineCommand(
    "simulate STRING",
    std::bind(&TeleopNode::simulateCommandCallback, this, _1));
  key_input_parser_.defineCommand(
    "align",
    std::bind(&TeleopNode::alignCommandCallback, this, _1));
}

void TeleopNode::keyCallback(
  const std::shared_ptr<TeleopInput::Request> request,
  std::shared_ptr<TeleopInput::Response> response)
{
  auto result = key_input_parser_.processKeyInput(
    request->command, request->realtime, request->autocomplete);
  response->response.message = result.message;
  response->response.realtime = result.realtime;
}

void TeleopNode::planCallback(const FootstepPlan::SharedPtr msg)
{
  plan_ = msg;
  step_index_ = 0;
}

Response TeleopNode::enableCommandCallback(
  const std::vector<std::string> & tokens)
{
  auto request = std::make_shared<ActuatorEnable::Request>();
  request->enable = tokens.at(0) == "enable";
  auto result = actuator_enable_client_->async_send_request(request);
  auto status = result.wait_for(100ms);
  if (status == std::future_status::ready) {
    auto response = result.get();
    if (response->success) {
      return Response{request->enable ? "Enabled" : "Disabled", false};
    } else {
      return Response{response->message, false};
    }
  }
  return Response{"Robot driver node is not responding", false};
}

Response TeleopNode::controlCommandCallback(
  const std::vector<std::string> & tokens)
{
  auto request = std::make_shared<SetBool::Request>();
  request->data = tokens.at(1) == "on";
  auto result = controller_enable_client_->async_send_request(request);
  auto status = result.wait_for(100ms);
  if (status == std::future_status::ready) {
    auto response = result.get();
    if (response->success) {
      controller_enable_ = request->data;
      return Response{request->data ? "Control on" : "Control off", false};
    } else {
      return Response{response->message, false};
    }
  }
  return Response{"Controller node is not responding", false};
}

Response TeleopNode::setCommandCallback(
  const std::vector<std::string> & tokens)
{
  setJoint(tokens.at(1), tokens.at(2), std::stod(tokens.at(3)));
  auto response = fmt::format(
    "{}: {} = {}", tokens.at(1), tokens.at(2), tokens.at(3));
  return Response{response, false};
}

Response TeleopNode::gotoCommandCallback(
  const std::vector<std::string> & tokens)
{
  setConfiguration(
    robot_->getJointNames(), configurations_.at(tokens.at(1)));
  return Response{"Configuration: " + tokens.at(1), false};
}

Response TeleopNode::moveCommandCallback(
  const std::vector<std::string> & tokens)
{
  key_input_parser_.setInputCallback(
    [this, tokens](char key) {
      switch (key) {
        case ' ':
          return Response{"Stopped", false};
        case 'w':
          moveJoint(tokens.at(1), joint_step_);
          break;
        case 's':
          moveJoint(tokens.at(1), -joint_step_);
          break;
      }
      return Response{"", true};
    });
  joint_setpoints_ = robot_->getJointPosition();
  auto response = fmt::format(
    "{}: Move with w/s keys, press space to stop", tokens.at(1));
  return Response{response, true};
}

Response TeleopNode::twistCommandCallback(
  const std::vector<std::string> & tokens)
{
  FootstepUpdate command;
  command.header.stamp = now();
  command.header.frame_id = tokens.at(1);
  command.command = FootstepUpdate::COMMAND_PAUSE;
  step_override_cmd_pub_->publish(command);
  key_input_parser_.setInputCallback(
    [this, tokens](char key) {
      if (key == ' ') {
        if (controller_enable_) {
          controlEndEffector(tokens.at(1), Eigen::Vector<double, 6>::Zero());
        }
        FootstepUpdate command;
        command.header.stamp = this->now();
        command.header.frame_id = tokens.at(1);
        command.command = FootstepUpdate::COMMAND_RESUME;
        step_override_cmd_pub_->publish(command);
        return Response{"Stopped", false};
      }
      if (controller_enable_) {
        controlEndEffector(tokens.at(1), getTwist(key, tokens.at(1)));
      } else if (tokens.at(1) == robot_->getBodyFrame()) {
        moveBody(getTwist(key));
      } else {
        moveEndEffector(tokens.at(1), getTwist(key, tokens.at(1)));
      }
      return Response{"", true};
    });
  joint_setpoints_ = robot_->getJointPosition();
  auto response = fmt::format(
    "{}: Linear with w/a/s/d/q/e keys, angular with W/A/S/D/Q/E keys, "
    "press space to stop", tokens.at(1));
  return Response{response, true};
}

Response TeleopNode::stepCommandCallback(
  const std::vector<std::string> & tokens)
{
  std::string frame = tokens.at(1);
  Footstep footstep;
  footstep.header.stamp = now();
  footstep.frames.push_back(frame);
  return takeStep(
    footstep, [this](const auto & result) {
      TeleopOutput msg;
      if (result.result->success) {
        msg.message = "Completed step";
      } else {
        msg.message = "Aborted step";
      }
      msg.realtime = false;
      teleop_output_pub_->publish(msg);
    });
}

Response TeleopNode::planCommandCallback(
  const std::vector<std::string> & tokens [[maybe_unused]])
{
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto result = plan_client_->async_send_request(request);
  auto status = result.wait_for(10s);
  if (status == std::future_status::ready) {
    auto response = result.get();
    if (response->success) {
      return Response{"Planned footsteps", false};
    } else {
      return Response{response->message, false};
    }
  }
  return Response{"Footstep planner node is not responding", false};
}

Response TeleopNode::executeCommandCallback(
  const std::vector<std::string> & tokens [[maybe_unused]])
{
  if (!plan_ || step_index_ >= plan_->steps.size()) {
    return Response{"No plan available", false};
  }
  return takeStep(
    plan_->steps.at(step_index_), [this](const auto & result) {
      // Start a new thread to avoid action client blocking, because
      // using a reentrant command group is not sufficient
      std::thread(
        [this, result]() {
          TeleopOutput msg;
          msg.realtime = false;
          if (result.result->success) {
            msg.message = "Completed step";
            if (result.result->success) {
              if (++step_index_ < plan_->steps.size()) {
                auto response = executeCommandCallback({});
                msg.message = response.message;
                msg.realtime = response.realtime;
              } else {
                msg.message = "Completed plan";
              }
            }
          } else {
            msg.message = "Aborted step";
          }
          teleop_output_pub_->publish(msg);
        }).detach();
    });
}

Response TeleopNode::simulateCommandCallback(
  const std::vector<std::string> & tokens)
{
  auto request = std::make_shared<SetString::Request>();
  request->data = tokens.at(1);
  auto result = simulate_client_->async_send_request(request);
  auto status = result.wait_for(1s);
  if (status == std::future_status::ready) {
    auto response = result.get();
    if (response->success) {
      return Response{"Loaded point cloud", false};
    } else {
      return Response{response->message, false};
    }
  }
  return Response{
    "Footstep planner node is not responding", false};
}

Response TeleopNode::alignCommandCallback(
  const std::vector<std::string> & tokens [[maybe_unused]])
{
  key_input_parser_.setInputCallback(
    [this](char key) {
      if (key == ' ') {
        return Response{"Stopped", false};
      }
      alignMap(getTwist(key));
      return Response{"", true};
    });
  auto response = fmt::format(
    "Align with w/a/s/d/q/e keys, press space to stop");
  return Response{response, true};
}

Eigen::Vector<double, 6> TeleopNode::getTwist(char key) const
{
  Eigen::Vector<double, 6> twist = Eigen::Vector<double, 6>::Zero();
  switch (key) {
    case 'w': twist[0] = linear_step_; break;
    case 's': twist[0] = -linear_step_; break;
    case 'a': twist[1] = linear_step_; break;
    case 'd': twist[1] = -linear_step_; break;
    case 'q': twist[2] = linear_step_; break;
    case 'e': twist[2] = -linear_step_; break;
    case 'W': twist[4] = angular_step_; break;
    case 'S': twist[4] = -angular_step_; break;
    case 'A': twist[3] = -angular_step_; break;
    case 'D': twist[3] = angular_step_; break;
    case 'Q': twist[5] = angular_step_; break;
    case 'E': twist[5] = -angular_step_; break;
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

std::string TeleopNode::getStateName(int state) const
{
  switch (state) {
    case FootstepCommand::Feedback::STATE_STANCE: return "Stance";
    case FootstepCommand::Feedback::STATE_DISENGAGE: return "Disengage";
    case FootstepCommand::Feedback::STATE_LIFT: return "Lift";
    case FootstepCommand::Feedback::STATE_SWING: return "Swing";
    case FootstepCommand::Feedback::STATE_PLACE: return "Place";
    case FootstepCommand::Feedback::STATE_ENGAGE: return "Engage";
    case FootstepCommand::Feedback::STATE_SNAG: return "Snag";
    case FootstepCommand::Feedback::STATE_SLIP: return "Slip";
    case FootstepCommand::Feedback::STATE_RETRY: return "Retry";
    case FootstepCommand::Feedback::STATE_STOP: return "Stop";
  }
  return "Invalid";
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
      RCLCPP_WARN(get_logger(), "Invalid property: %s", property.c_str());
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
  ControllerCommand command;
  command.header.stamp = now();
  command.frame = {contact};
  command.mode = {ControllerCommand::MODE_FREE};
  command.twist = {RosUtils::eigenToTwist(twist)};
  command.overrides.name.push_back("spine_joint");
  command.overrides.position.push_back(0.0);
  controller_cmd_pub_->publish(command);
}

void TeleopNode::alignMap(const Eigen::Vector<double, 6> & twist)
{
  auto transform = lookupTransform("/map", "/odom");
  auto T = RosUtils::transformToEigen(transform.transform);
  EigenUtils::applyTwistInPlace(T, twist);
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = now();
  msg.header.frame_id = "/map";
  msg.child_frame_id = "/odom";
  msg.transform = RosUtils::eigenToTransform(T);
  sendTransform(msg);
}

Response TeleopNode::takeStep(
  const Footstep & footstep,
  rclcpp_action::Client<FootstepCommand>::ResultCallback result_callback)
{
  if (footstep.frames.empty()) {
    return Response{"No contact frame specified", false};
  }
  FootstepCommand::Goal goal;
  goal.footstep = footstep;
  auto options = rclcpp_action::Client<FootstepCommand>::SendGoalOptions();
  options.feedback_callback =
    [this, footstep](
    auto, const std::shared_ptr<const FootstepCommand::Feedback> feedback)
    {
      TeleopOutput msg;
      msg.message = fmt::format(
        "{} state: {}", footstep.frames[0], getStateName(feedback->state));
      msg.realtime = true;
      teleop_output_pub_->publish(msg);
    };
  options.result_callback = result_callback;
  auto goal_handle_future = step_cmd_client_->async_send_goal(goal, options);
  auto goal_handle_status = goal_handle_future.wait_for(100ms);
  if (goal_handle_status != std::future_status::ready) {
    return Response{"Step planner node is not responding", false};
  }
  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    return Response{"Step planner node rejected the requested step", false};
  }
  key_input_parser_.setInputCallback(
    [this, footstep, goal_handle](char key) {
      FootstepUpdate command;
      command.header.stamp = this->now();
      command.header.frame_id = footstep.frames[0];
      if (key == ' ') {
        step_cmd_client_->async_cancel_goal(goal_handle);
        return Response{"", true};
      } else if (key == '.') {
        command.command = FootstepUpdate::COMMAND_ADVANCE;
        step_override_cmd_pub_->publish(command);
        return Response{"Advancing Step", true};
      } else if (key == ',') {
        command.command = FootstepUpdate::COMMAND_RETRY;
        step_override_cmd_pub_->publish(command);
        return Response{"Retrying Step", true};
      } else {
        auto transform = lookupTransform("/map", robot_->getBodyFrame());
        Eigen::Matrix3d rotation = RosUtils::transformToEigen(transform.transform).rotation();
        Eigen::Vector<double, 6> twist = EigenUtils::rotateTwist(getTwist(key), rotation);
        command.offset = RosUtils::eigenToTwist(twist);
        step_override_cmd_pub_->publish(command);
        return Response{"", true};
      }
    });
  return Response{fmt::format("Taking step {}", footstep.frames[0]), true};
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
