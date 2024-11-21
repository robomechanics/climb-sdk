#include "climb_main/controller_node.hpp"
#include "climb_main/kinematics/kdl_interface.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

ControllerNode::ControllerNode()
: KinematicsNode("ControllerNode")
{
  // Initialize kinematics model, estimators, and controller
  contact_estimator_ = std::make_unique<ContactEstimator>(robot_);
  force_estimator_ = std::make_unique<ForceEstimator>(robot_);
  force_controller_ = std::make_unique<ForceController>(robot_);

  // Declare parameters
  this->declare_parameter("debug", false);
  this->declare_parameter("effort_limit", 0.0);
  this->declare_parameter("sim_wrench", std::vector<double>());
  this->declare_parameter("default_configuration", "");
  for (const auto & p : contact_estimator_->getParameters()) {
    this->declare_parameter(p.name, p.default_value, p.descriptor);
  }
  for (const auto & p : force_estimator_->getParameters()) {
    this->declare_parameter(p.name, p.default_value, p.descriptor);
  }
  for (const auto & p : force_controller_->getParameters()) {
    this->declare_parameter(p.name, p.default_value, p.descriptor);
  }

  // Define publishers and subscribers
  ee_cmd_sub_ = this->create_subscription<EndEffectorCommand>(
    "end_effector_commands", 1,
    std::bind(&ControllerNode::endEffectorCmdCallback, this, _1));
  joint_cmd_pub_ = this->create_publisher<JointCommand>("joint_commands", 1);
  controller_enable_srv_ = this->create_service<ControllerEnable>(
    "controller_enable",
    std::bind(&ControllerNode::controllerEnableCallback, this, _1, _2));
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  RCLCPP_INFO(this->get_logger(), "Controller node initialized");
}

void ControllerNode::update()
{
  // Update contact frames
  std::vector<geometry_msgs::msg::TransformStamped> frames;
  frames = contact_estimator_->update(gravity_);
  robot_->updateContactFrames(frames);

  // Estimate contact forces
  Eigen::VectorXd forces = force_estimator_->update();

  // Adjust forces to match simulated wrench if specified
  if (sim_wrench_.size()) {
    Eigen::MatrixXd Gs = -robot_->getGraspMap();
    forces += Gs.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(
      sim_wrench_ - Gs * forces);
  }

  // Log current forces
  if (debug_) {
    RCLCPP_INFO_STREAM(
      this->get_logger(), "Current forces: " << std::fixed << std::setprecision(
        2) << forces.transpose());
  }

  if (enabled_) {
    // Update controller
    auto ground = contact_estimator_->getGroundPlane();
    force_controller_->setGroundConstraint(ground.normal, ground.distance);
    bool success = force_controller_->update(forces);

    // Publish joint commands
    if (success && force_controller_->getJointDisplacement().norm() > 1e-6) {
      // Initialize joint command message
      JointCommand cmd_msg;
      cmd_msg.header.stamp = this->get_clock()->now();
      cmd_msg.name = robot_->getJointNames();
      Eigen::VectorXd joint_position = force_controller_->getJointPosition();
      cmd_msg.position = std::vector<double>(
        joint_position.data(), joint_position.data() + joint_position.size());

      // Set expected joint efforts for dummy interface or maximum effort limit
      if (sim_wrench_.size()) {
        Eigen::VectorXd joint_effort = force_controller_->getJointEffort();
        cmd_msg.effort = std::vector<double>(
          joint_effort.data(), joint_effort.data() + joint_effort.size());
      } else {
        for (size_t i = 0; i < cmd_msg.name.size(); i++) {
          cmd_msg.effort.push_back(max_effort_);
        }
      }

      // Publish joint command
      robot_->clampJointCommand(cmd_msg);
      joint_cmd_pub_->publish(cmd_msg);

      // Log controller results
      if (debug_) {
        RCLCPP_INFO_STREAM(
          this->get_logger(),
          "Margin: " << force_controller_->getMargin() <<
            ", Error: " << force_controller_->getError());
        RCLCPP_INFO_STREAM(
          this->get_logger(),
          "Goal force: " << force_controller_->getContactForce().transpose());
        RCLCPP_INFO_STREAM(
          this->get_logger(),
          "Joint displacement: " <<
            force_controller_->getJointDisplacement().transpose());
        RCLCPP_INFO_STREAM(
          this->get_logger(),
          "Body displacement: " <<
            force_controller_->getBodyDisplacement().transpose());
        RCLCPP_INFO_STREAM(
          this->get_logger(),
          "Force displacement: " <<
            (force_controller_->getContactForce() - forces).transpose());
      }
    }
  }

  // Publish contact frames
  for (auto & frame : frames) {
    frame.header.stamp = this->get_clock()->now();
    frame.header.frame_id = name_ + "/" + frame.header.frame_id;
    frame.child_frame_id = name_ + "/" + frame.child_frame_id;
    tf_broadcaster_->sendTransform(frame);
  }

  // Publish contact forces
  std::vector<WrenchStamped> contact_forces =
    force_estimator_->forcesToMessages(forces, this->get_clock()->now(), name_);
  for (size_t i = 0; i < contact_forces.size(); i++) {
    if (contact_force_pubs_.size() == i) {
      if (i == contact_forces.size() - 1) {
        contact_force_pubs_.push_back(
          this->create_publisher<WrenchStamped>("gravity_force", 10));
      } else {
        contact_force_pubs_.push_back(
          this->create_publisher<WrenchStamped>(
            "contact_force_" + std::to_string(i + 1), 10));
      }
    }
    contact_force_pubs_.at(i)->publish(contact_forces[i]);
  }
  gravity_ <<
    contact_forces.back().wrench.force.x,
    contact_forces.back().wrench.force.y,
    contact_forces.back().wrench.force.z;
}

void ControllerNode::jointCallback(const JointState::SharedPtr msg)
{
  if (!robot_->isInitialized()) {
    return;
  }
  robot_->updateJointState(*msg);
  update();
}

void ControllerNode::endEffectorCmdCallback(
  const EndEffectorCommand::SharedPtr msg)
{
  force_controller_->setEndEffectorCommand(*msg);
}

void ControllerNode::controllerEnableCallback(
  const ControllerEnable::Request::SharedPtr request,
  ControllerEnable::Response::SharedPtr response)
{
  if (request->enable) {
    force_controller_->reset();
  }
  enabled_ = request->enable;
  response->success = true;
}

rcl_interfaces::msg::SetParametersResult ControllerNode::parameterCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  auto result = KinematicsNode::parameterCallback(parameters);
  for (const auto & param : parameters) {
    if (param.get_name() == "debug") {
      debug_ = param.as_bool();
    } else if (param.get_name() == "effort_limit") {
      max_effort_ = param.as_double();
    } else if (param.get_name() == "sim_wrench") {
      std::vector<double> wrench = param.as_double_array();
      if (wrench.size() == 0 || wrench.size() == 6) {
        sim_wrench_ = Eigen::VectorXd::Map(wrench.data(), wrench.size()).eval();
      } else {
        result.successful = false;
        result.reason = "Invalid wrench size (expected 0 or 6)";
      }
    } else if (param.get_name() == "default_configuration") {
      const auto & parameters =
        this->get_node_parameters_interface()->get_parameter_overrides();
      auto param_name = "configurations." + param.as_string();
      if (param.as_string().empty()) {
        force_controller_->setNominalConfiguration(
          Eigen::VectorXd::Zero(robot_->getNumJoints()));
      } else if (parameters.find(param_name) != parameters.end()) {
        auto config = parameters.at(param_name).get<std::vector<double>>();
        force_controller_->setNominalConfiguration(
          Eigen::VectorXd::Map(config.data(), config.size()));
      } else {
        result.successful = false;
        result.reason = "Configuration not found";
      }
    }
    contact_estimator_->setParameter(param, result);
    force_estimator_->setParameter(param, result);
    force_controller_->setParameter(param, result);
  }
  return result;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
