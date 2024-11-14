#include "climb_main/controller_node.hpp"
#include "climb_main/kinematics/kdl_interface.hpp"

using std::placeholders::_1;

ControllerNode::ControllerNode()
: Node("ControllerNode")
{
  // Initialize kinematics model, estimators, and controller
  robot_ = std::make_shared<KdlInterface>();
  contact_estimator_ = std::make_unique<ContactEstimator>(robot_);
  force_estimator_ = std::make_unique<ForceEstimator>(robot_);
  force_controller_ = std::make_unique<ForceController>(robot_);
  Eigen::VectorXd initial_pos(14);
  initial_pos << 0, 0, 0, 0, 0, 0, 0, 0, -0.5, -0.5, -0.5, -0.5, 0, 0;
  force_controller_->reset(initial_pos);

  // Subscribe to parameter changes
  param_handle_ = this->add_on_set_parameters_callback(
    std::bind(&ControllerNode::parameterCallback, this, _1));

  // Declare parameters
  this->declare_parameter("tf_prefix", "");
  this->declare_parameter("debug", false);
  this->declare_parameter("effort_limit", 0.0);
  this->declare_parameter("sim_wrench", std::vector<double>());
  for (const auto & p : robot_->getParameters()) {
    this->declare_parameter(p.name, p.default_value, p.descriptor);
  }
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
  description_sub_ = this->create_subscription<String>(
    "robot_description", rclcpp::QoS(1).transient_local(),
    std::bind(&ControllerNode::descriptionCallback, this, _1));
  joint_sub_ = this->create_subscription<JointState>(
    "joint_states", 1,
    std::bind(&ControllerNode::jointCallback, this, _1));
  ee_cmd_sub_ = this->create_subscription<EndEffectorCommand>(
    "end_effector_commands", 1,
    std::bind(&ControllerNode::endEffectorCmdCallback, this, _1));
  joint_cmd_pub_ = this->create_publisher<JointCommand>("joint_commands", 1);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  RCLCPP_INFO(this->get_logger(), "Controller node initialized");
}

void ControllerNode::update()
{
  // Update contact frames
  auto frames = contact_estimator_->update(-sim_wrench_.topRows(3));
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

  // Set controller command (temporary code for testing)
  EndEffectorCommand cmd;
  cmd.frame = {"gripper_1", "gripper_2", "gripper_3", "gripper_4", "tail_contact"};
  cmd.mode = {EndEffectorCommand::MODE_STANCE, EndEffectorCommand::MODE_STANCE,
    EndEffectorCommand::MODE_STANCE, EndEffectorCommand::MODE_STANCE,
    EndEffectorCommand::MODE_CONTACT};
  force_controller_->setEndEffectorCommand(cmd);

  // Update controller
  force_controller_->setGroundConstraint();
  bool success = force_controller_->update(forces);

  // Publish joint commands
  if (success && force_controller_->getJointDisplacement().norm() > 1e-6) {
    // Initialize joint command message
    JointCommand cmd_msg;
    cmd_msg.header.stamp = this->get_clock()->now();
    cmd_msg.name = robot_->getJointNames();
    cmd_msg.mode =
      std::vector<uint8_t>(robot_->getNumJoints(), JointCommand::MODE_POSITION);

    // Set commanded joint positions
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
    contact_force_pubs_[i]->publish(contact_forces[i]);
  }
}

void ControllerNode::descriptionCallback(const String::SharedPtr msg)
{
  std::string error_message;
  if (robot_->loadRobotDescription(msg->data, error_message)) {
    RCLCPP_INFO(this->get_logger(), "Robot description loaded");
    RCLCPP_INFO(this->get_logger(), "\tMass: %.3f kg", robot_->getMass());
    RCLCPP_INFO(this->get_logger(), "\tJoints: %d", robot_->getNumJoints());
    RCLCPP_INFO(
      this->get_logger(), "\tEnd effectors: %d", robot_->getNumContacts());
    RCLCPP_INFO(
      this->get_logger(), "\tConstraints: %d", robot_->getNumConstraints());
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to load robot description: %s", error_message.c_str());
    return;
  }
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

rcl_interfaces::msg::SetParametersResult ControllerNode::parameterCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "Success";
  for (const auto & param : parameters) {
    if (param.get_name() == "tf_prefix") {
      name_ = param.as_string();
    } else if (param.get_name() == "debug") {
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
    }
    bool initialized = robot_->isInitialized();
    robot_->setParameter(param, result);
    contact_estimator_->setParameter(param, result);
    force_estimator_->setParameter(param, result);
    force_controller_->setParameter(param, result);
    if (!initialized && robot_->isInitialized()) {
      RCLCPP_INFO(
        this->get_logger(),
        "Kinematics interface initialized");
    } else if (initialized && !robot_->isInitialized()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Kinematics interface disabled: %s", result.reason.c_str());
    }
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
