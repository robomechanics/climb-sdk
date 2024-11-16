#include "climb_main/teleop_node.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

TeleopNode::TeleopNode()
: KinematicsNode("TeleopNode")
{
  // Define commands
  addCommands();
  // Initialize service
  key_input_service_ = this->create_service<KeyInput>(
    "key_input", std::bind(&TeleopNode::keyCallback, this, _1, _2));
  // Initialize publishers
  joint_cmd_pub_ = this->create_publisher<JointCommand>("joint_commands", 1);
}

void TeleopNode::addCommands()
{
  key_input_parser_.defineToken(
    "JOINT", [this]() {
      return robot_->getJointNames();
    });
  key_input_parser_.defineToken(
    "FRAME", [this]() {
      auto frames = robot_->getContactFrames();
      frames.insert(frames.begin(), robot_->getBodyFrame());
      return frames;
    });
  key_input_parser_.defineCommand(
    "set JOINT [position|velocity] DOUBLE deg",
    [this](const std::vector<std::string> & tokens) {
      setJoint(
        tokens.at(1), tokens.at(2), std::stod(tokens.at(3)) * M_PI / 180);
      std::ostringstream ss;
      ss <<
        tokens.at(1) << ": " << tokens.at(2) << " = " << tokens.at(3) << "°";
      return KeyInputParser::Response{ss.str(), false};
    });
  key_input_parser_.defineCommand(
    "set JOINT [position|velocity|effort] DOUBLE",
    [this](const std::vector<std::string> & tokens) {
      setJoint(tokens.at(1), tokens.at(2), std::stod(tokens.at(3)));
      std::ostringstream ss;
      ss << tokens.at(1) << ": " << tokens.at(2) << " = " << tokens.at(3);
      return KeyInputParser::Response{ss.str(), false};
    });
  key_input_parser_.defineCommand(
    "set [position|velocity] DOUBLE deg",
    [this](const std::vector<std::string> & tokens) {
      setJoint("", tokens.at(1), std::stod(tokens.at(2)) * M_PI / 180);
      std::ostringstream ss;
      ss << "all joints: " << tokens.at(1) << " = " << tokens.at(2) << "°";
      return KeyInputParser::Response{ss.str(), false};
    });
  key_input_parser_.defineCommand(
    "set [position|velocity|effort] DOUBLE",
    [this](const std::vector<std::string> & tokens) {
      setJoint("", tokens.at(1), std::stod(tokens.at(2)));
      std::ostringstream ss;
      ss << "all joints: " << tokens.at(1) << " = " << tokens.at(2);
      return KeyInputParser::Response{ss.str(), false};
    });
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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeleopNode>());
  rclcpp::shutdown();
  return 0;
}
