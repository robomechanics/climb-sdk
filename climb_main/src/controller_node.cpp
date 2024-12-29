#include "climb_main/controller_node.hpp"
#include "climb_main/kinematics/kdl_interface.hpp"
#include "climb_main/util/ros_utils.hpp"
#include "climb_main/util/eigen_utils.hpp"

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
  contact_force_pub_ =
    this->create_publisher<ContactForce>("contact_forces", 1);
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
  Imu imu;
  imu.linear_acceleration.x = -1.0;   // TODO: Read from IMU
  Eigen::VectorXd forces = force_estimator_->update(imu);

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

  Eigen::Vector<double, 6> body_twist = Eigen::Vector<double, 6>::Zero();
  if (enabled_) {
    // Update controller
    auto ground = contact_estimator_->getGroundPlane();
    force_controller_->setGroundConstraint(ground.normal, ground.distance);

    Eigen::Isometry3d nominal_pose = Eigen::Isometry3d::Identity();
    nominal_pose.translation() = ground.origin;
    // nominal_pose.linear() = Eigen::Quaterniond::FromTwoVectors(
    //   -Eigen::Vector3d::UnitZ(), ground.normal).toRotationMatrix();
    TransformStamped goal_pose;
    goal_pose.header.frame_id = name_ + "/" + robot_->getBodyFrame();
    goal_pose.header.stamp = this->get_clock()->now();
    goal_pose.child_frame_id = name_ + "/goal_pose";
    goal_pose.transform = RosUtils::eigenToTransform(nominal_pose);
    tf_broadcaster_->sendTransform(goal_pose);

    bool success = force_controller_->updateDecoupled(forces, nominal_pose);

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
      body_twist = force_controller_->getBodyDisplacement();

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
          "Goal effort: " << force_controller_->getJointEffort().transpose());
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
  const auto time = this->get_clock()->now();
  const std::string prefix = name_ + "/";
  for (auto & frame : frames) {
    frame.header.stamp = time;
    frame.header.frame_id.insert(0, prefix);
    frame.child_frame_id.insert(0, prefix);
    tf_broadcaster_->sendTransform(frame);
  }

  // Update map frame
  TransformStamped map_to_body = lookupMapToBodyTransform();
  if (!map_to_body.header.frame_id.empty()) {
    Eigen::Isometry3d transform =
      RosUtils::transformToEigen(map_to_body.transform);
    EigenUtils::applyChildTwistInPlace(transform, body_twist);
    geometry_msgs::msg::TransformStamped map_frame;
    map_frame.header.stamp = this->get_clock()->now();
    map_frame.header.frame_id = name_ + "/" + robot_->getBodyFrame();
    map_frame.child_frame_id = name_ + "/map";
    map_frame.transform = RosUtils::eigenToTransform(transform.inverse());
    tf_broadcaster_->sendTransform(map_frame);
  }

  // Publish contact forces
  auto contact_force =
    force_estimator_->getContactForceMessage(forces, this->get_clock()->now());
  contact_force_pub_->publish(contact_force);
  // Publish individual contact wrenches for RViz
  auto contact_wrenches =
    force_estimator_->splitContactForceMessage(contact_force, name_);
  for (size_t i = 0; i < contact_wrenches.size(); i++) {
    if (contact_force_pubs_.size() == i) {
      contact_force_pubs_.push_back(
        this->create_publisher<WrenchStamped>(
          "contact_force_" + std::to_string(i + 1), 10));
    }
    contact_force_pubs_.at(i)->publish(contact_wrenches[i]);
  }
  // Display gravity wrench for RViz
  auto gravity_wrench =
    force_estimator_->getGravityForceMessage(
    forces, this->get_clock()->now(), name_);
  if (contact_force_pubs_.size() == contact_wrenches.size()) {
    contact_force_pubs_.push_back(
      this->create_publisher<WrenchStamped>("gravity_force", 10));
  }
  contact_force_pubs_.back()->publish(gravity_wrench);
  // Store gravity force for next iteration
  gravity_ <<
    gravity_wrench.wrench.force.x,
    gravity_wrench.wrench.force.y,
    gravity_wrench.wrench.force.z;
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
      // Declared by force_controller
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
