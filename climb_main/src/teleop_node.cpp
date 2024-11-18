#include "climb_main/teleop_node.hpp"

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
  this->declare_parameter("joint_step", 0.02);
  this->declare_parameter("linear_step", 0.002);
  this->declare_parameter("angular_step", 0.02);
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
  // Custom tokens
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
  key_input_parser_.defineToken(
    "CONFIGURATION", [this]() {
      std::vector<std::string> poses;
      for (const auto & [name, _] : configurations_) {
        poses.push_back(name);
      }
      return poses;
    });
  // Set joint state
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
  // Go to configuration
  key_input_parser_.defineCommand(
    "goto CONFIGURATION",
    [this](const std::vector<std::string> & tokens) {
      setConfiguration(
        robot_->getJointNames(), configurations_.at(tokens.at(1)));
      return KeyInputParser::Response{"Configuration: " + tokens.at(1), false};
    });
  // Move joint
  key_input_parser_.defineCommand(
    "move JOINT",
    [this](const std::vector<std::string> & tokens) {
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
      std::ostringstream ss;
      ss << tokens.at(1) << ": Move with w/s keys, press space to stop";
      return KeyInputParser::Response{ss.str(), true};
    });
  // Twist contact frame
  key_input_parser_.defineCommand(
    "twist FRAME",
    [this](const std::vector<std::string> & tokens) {
      key_input_parser_.setInputCallback(
        [this, tokens](char key) {
          if (key == ' ') {
            return KeyInputParser::Response{"Stopped", false};
          }
          if (tokens.at(1) == robot_->getBodyFrame()) {
            moveBody(getTwist(key));
          } else {
            moveEndEffector(tokens.at(1), getTwist(key));
          }
          return KeyInputParser::Response{"", true};
        });
      joint_setpoints_ = robot_->getJointPosition();
      std::ostringstream ss;
      ss << tokens.at(
        1) << ": Linear with w/a/s/d/q/e keys, angular with W/A/S/D/Q/E keys, press space to stop";
      return KeyInputParser::Response{ss.str(), true};
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

Twist TeleopNode::getTwist(char key) const
{
  Twist twist;
  switch (key) {
    case 'w':
      twist.linear.x = linear_step_;
      break;
    case 's':
      twist.linear.x = -linear_step_;
      break;
    case 'a':
      twist.linear.y = linear_step_;
      break;
    case 'd':
      twist.linear.y = -linear_step_;
      break;
    case 'q':
      twist.linear.z = linear_step_;
      break;
    case 'e':
      twist.linear.z = -linear_step_;
      break;
    case 'W':
      twist.angular.y = angular_step_;
      break;
    case 'S':
      twist.angular.y = -angular_step_;
      break;
    case 'A':
      twist.angular.x = -angular_step_;
      break;
    case 'D':
      twist.angular.x = angular_step_;
      break;
    case 'Q':
      twist.angular.z = angular_step_;
      break;
    case 'E':
      twist.angular.z = -angular_step_;
      break;
  }
  return twist;
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

void TeleopNode::moveEndEffector(const std::string & contact, Twist twist)
{
  Eigen::MatrixXd Jh = robot_->getHandJacobian();
  Eigen::Matrix3d R = robot_->getTransform(contact).second;
  Eigen::MatrixXd Ad = robot_->getAdjoint({}, R.transpose());
  Eigen::MatrixXd B = robot_->getWrenchBasis(contact);
  Eigen::VectorXd twist_vec(6);
  twist_vec << twist.linear.x, twist.linear.y, twist.linear.z,
    twist.angular.x, twist.angular.y, twist.angular.z;
  Eigen::VectorXd contact_twist = B.transpose() * Ad * twist_vec;
  Eigen::VectorXd all_twists =
    Eigen::VectorXd::Zero(robot_->getNumConstraints());
  int ind = 0;
  for (const auto & frame : robot_->getContactFrames()) {
    int c = robot_->getNumConstraints(frame);
    if (frame == contact) {
      all_twists.segment(ind, c) = contact_twist;
    }
    ind += c;
  }
  Eigen::VectorXd displacements =
    Jh.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(all_twists);
  moveJoints(displacements);
}

void TeleopNode::moveBody(Twist twist)
{
  Eigen::MatrixXd Jh = robot_->getHandJacobian();
  Eigen::MatrixXd Gs = -robot_->getGraspMap();
  Eigen::VectorXd twist_vec(6);
  twist_vec << twist.linear.x, twist.linear.y, twist.linear.z,
    twist.angular.x, twist.angular.y, twist.angular.z;
  Eigen::VectorXd displacements =
    Jh.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(
      Gs.transpose() * twist_vec);
  moveJoints(displacements);
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
  rclcpp::spin(std::make_shared<TeleopNode>());
  rclcpp::shutdown();
  return 0;
}
