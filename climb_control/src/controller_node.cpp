#include "climb_control/controller_node.hpp"

#include <climb_util/eigen_utils.hpp>
#include <climb_util/ros_utils.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

ControllerNode::ControllerNode()
: KinematicsNode("ControllerNode")
{
  // Initialize kinematics model, estimators, and controller
  contact_estimator_ = std::make_unique<ContactEstimator>(robot_);
  force_estimator_ = std::make_unique<ForceEstimator>(robot_);
  force_controller_ = std::make_unique<ForceController>(robot_);

  // Parameter to print debugging information for the controller
  declare_parameter("debug", false);
  // Parameter to set the nominal configuration for the controller
  declare_parameter("default_configuration", "");
  // Parameter to publish odometry estimates using dead reckoning
  declare_parameter("compute_odometry", false);
  // Parameter to provide initial estimate of gravity vector in the body frame
  declare_parameter("gravity_estimate", std::vector<double>());
  for (const auto & p : contact_estimator_->getParameters()) {
    declare_parameter(p.name, p.default_value, p.descriptor);
  }
  for (const auto & p : force_estimator_->getParameters()) {
    declare_parameter(p.name, p.default_value, p.descriptor);
  }
  for (const auto & p : force_controller_->getParameters()) {
    declare_parameter(p.name, p.default_value, p.descriptor);
  }

  // Initialize odometry
  gravity_covariance_ = Eigen::Matrix3d::Identity();
  if (compute_odometry_) {
    TransformStamped map_to_odom;
    map_to_odom.header.frame_id = "/map";
    map_to_odom.child_frame_id = "/odom";
    auto q = Eigen::Quaterniond::FromTwoVectors(
      -gravity_, Eigen::Vector3d::UnitZ());
    gravity_ = q * gravity_;
    map_to_odom.transform.rotation = RosUtils::eigenToQuaternion(q);
    map_to_odom.header.stamp = now();
    sendTransform(map_to_odom);
  }

  // Define publishers and subscribers
  imu_sub_ = create_subscription<Imu>(
    "imu", 1, std::bind(&ControllerNode::imuCallback, this, _1));
  ee_cmd_sub_ = create_subscription<EndEffectorCommand>(
    "end_effector_commands", 1,
    std::bind(&ControllerNode::endEffectorCmdCallback, this, _1));
  joint_cmd_pub_ = create_publisher<JointCommand>("joint_commands", 1);
  contact_force_pub_ = create_publisher<ContactForce>("contact_forces", 1);
  controller_enable_srv_ = create_service<SetBool>(
    "controller_enable",
    std::bind(&ControllerNode::controllerEnableCallback, this, _1, _2));
  RCLCPP_INFO(get_logger(), "Controller node initialized");
}

void ControllerNode::update()
{
  // Update contact frames
  TransformStamped body_to_map =
    lookupTransform(robot_->getBodyFrame(), "/map");
  Eigen::Matrix3d rotation = RosUtils::quaternionToEigen(
    body_to_map.transform.rotation).toRotationMatrix();
  auto frames = contact_estimator_->update(rotation * gravity_);
  robot_->updateContactFrames(frames);

  // Estimate contact forces
  Eigen::VectorXd forces;
  if (use_gravity_) {
    forces = force_estimator_->update(
      rotation * gravity_,
      rotation * gravity_covariance_ * rotation.transpose());
  } else {
    forces = force_estimator_->update();
  }

  // Log current forces
  if (debug_) {
    RCLCPP_INFO_STREAM(
      get_logger(), "Current forces: " << std::fixed << std::setprecision(
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
    goal_pose.header.frame_id = robot_->getBodyFrame();
    goal_pose.header.stamp = now();
    goal_pose.child_frame_id = "goal_pose";
    goal_pose.transform = RosUtils::eigenToTransform(nominal_pose);
    sendTransform(goal_pose);

    bool success = force_controller_->updateDecoupled(forces, nominal_pose);

    // Publish joint commands
    if (success && force_controller_->getJointDisplacement().norm() > 1e-6) {
      // Initialize joint command message
      JointCommand cmd_msg;
      cmd_msg.header.stamp = now();
      cmd_msg.name = robot_->getJointNames();
      Eigen::VectorXd joint_position = force_controller_->getJointPosition();
      cmd_msg.position = RosUtils::eigenToVector(joint_position);

      // Set expected joint efforts for dummy interface or maximum effort limit
      if (offline_) {
        Eigen::VectorXd joint_effort = force_controller_->getJointEffort();
        cmd_msg.effort = RosUtils::eigenToVector(joint_effort);
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
          get_logger(),
          "Margin: " << force_controller_->getMargin() <<
            ", Error: " << force_controller_->getError());
        RCLCPP_INFO_STREAM(
          get_logger(),
          "Goal force: " << force_controller_->getContactForce().transpose());
        RCLCPP_INFO_STREAM(
          get_logger(),
          "Goal effort: " << force_controller_->getJointEffort().transpose());
        RCLCPP_INFO_STREAM(
          get_logger(),
          "Joint displacement: " <<
            force_controller_->getJointDisplacement().transpose());
        RCLCPP_INFO_STREAM(
          get_logger(),
          "Body displacement: " <<
            force_controller_->getBodyDisplacement().transpose());
        RCLCPP_INFO_STREAM(
          get_logger(),
          "Force displacement: " <<
            (force_controller_->getContactForce() - forces).transpose());
      }
    }
  }

  // Publish contact frames
  for (auto & frame : frames) {
    frame.header.stamp = now();
    sendTransform(frame);
  }

  // Publish contact forces
  auto contact_force =
    force_estimator_->getContactForceMessage(forces, now());
  contact_force_pub_->publish(contact_force);
  // Publish individual contact wrenches for RViz
  auto contact_wrenches =
    force_estimator_->splitContactForceMessage(contact_force);
  for (size_t i = 0; i < contact_wrenches.size(); i++) {
    if (contact_force_pubs_.size() == i) {
      contact_force_pubs_.push_back(
        create_publisher<WrenchStamped>(
          "contact_force_" + std::to_string(i + 1), 10));
    }
    prefixFrameId(contact_wrenches[i].header.frame_id);
    contact_force_pubs_.at(i)->publish(contact_wrenches[i]);
  }
  // Display gravity wrench for RViz
  auto gravity_wrench =
    force_estimator_->getGravityForceMessage(forces, now());
  if (contact_force_pubs_.size() == contact_wrenches.size()) {
    contact_force_pubs_.push_back(
      create_publisher<WrenchStamped>("gravity_force", 10));
  }
  prefixFrameId(gravity_wrench.header.frame_id);
  contact_force_pubs_.back()->publish(gravity_wrench);

  if (compute_odometry_) {
    // Update odometry via dead reckoning
    TransformStamped odom_to_body =
      lookupTransform("/odom", robot_->getBodyFrame());
    Eigen::Isometry3d transform =
      RosUtils::transformToEigen(odom_to_body.transform);
    EigenUtils::applyChildTwistInPlace(transform, body_twist);
    odom_to_body.transform = RosUtils::eigenToTransform(transform);
    odom_to_body.header.stamp = now();
    sendTransform(odom_to_body);
    // Keep map transform alive
    TransformStamped map_to_odom = lookupTransform("/map", "/odom");
    map_to_odom.header.stamp = now();
    sendTransform(map_to_odom);
    // Update gravity estimate if IMU is disabled
    if (!use_gravity_) {
      gravity_ = rotation.inverse() *
        RosUtils::vector3ToEigen(gravity_wrench.wrench.force);
    }
  }
}

void ControllerNode::jointCallback(const JointState::SharedPtr msg)
{
  if (!robot_->isInitialized()) {
    return;
  }
  robot_->updateJointState(*msg);
  offline_ = msg->header.frame_id == "dummy";
  use_gravity_ = use_gravity_ || offline_;
  update();
}

void ControllerNode::imuCallback(const Imu::SharedPtr msg)
{
  TransformStamped transform;
  try {
    transform = lookupTransform("/map", "/" + msg->header.frame_id);
  } catch (const tf2::TransformException &) {
    return;
  }
  Eigen::Matrix3d rotation = RosUtils::quaternionToEigen(
    transform.transform.rotation).toRotationMatrix();
  gravity_ = -rotation * RosUtils::vector3ToEigen(msg->linear_acceleration);
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> covariance(
    msg->linear_acceleration_covariance.data());
  gravity_covariance_ = rotation * covariance * rotation.transpose();
  use_gravity_ = true;
}

void ControllerNode::endEffectorCmdCallback(
  const EndEffectorCommand::SharedPtr msg)
{
  force_controller_->setEndEffectorCommand(*msg);
}

void ControllerNode::controllerEnableCallback(
  const SetBool::Request::SharedPtr request,
  SetBool::Response::SharedPtr response)
{
  if (request->data) {
    force_controller_->reset();
  }
  enabled_ = request->data;
  response->success = true;
}

rcl_interfaces::msg::SetParametersResult ControllerNode::parameterCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  auto result = KinematicsNode::parameterCallback(parameters);
  for (const auto & param : parameters) {
    if (param.get_name() == "debug") {
      debug_ = param.as_bool();
    } else if (param.get_name() == "compute_odometry") {
      compute_odometry_ = param.as_bool();
    } else if (param.get_name() == "gravity_estimate") {
      if (param.as_double_array().size() == 3) {
        gravity_ = Eigen::Vector3d(param.as_double_array().data());
        use_gravity_ = true;
      } else if (param.as_double_array().empty()) {
        gravity_ = Eigen::Vector3d::UnitZ() * -9.81;
        use_gravity_ = false;
      } else {
        result.successful = false;
        result.reason = "Parameter must be length 3, or empty to disable";
      }
    } else if (param.get_name() == "effort_limit") {
      // Declared by force_controller
      max_effort_ = param.as_double();
    } else if (param.get_name() == "default_configuration") {
      const auto & parameters =
        get_node_parameters_interface()->get_parameter_overrides();
      auto param_name = "configurations." + param.as_string();
      if (param.as_string().empty()) {
        force_controller_->setNominalConfiguration(
          Eigen::VectorXd::Zero(robot_->getNumJoints()));
      } else if (parameters.find(param_name) != parameters.end()) {
        auto config = parameters.at(param_name).get<std::vector<double>>();
        force_controller_->setNominalConfiguration(
          RosUtils::vectorToEigen(config));
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
