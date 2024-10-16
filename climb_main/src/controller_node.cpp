#include "climb_main/controller_node.hpp"
#include "climb_main/kinematics/kdl_interface.hpp"

using std::placeholders::_1;

ControllerNode::ControllerNode()
: Node("ControllerNode")
{
  // Initialize kinematics model, estimators, and controller
  robot_ = std::make_shared<KdlInterface>();
  force_estimator_ = std::make_unique<ForceEstimator>(robot_);

  // Subscribe to parameter changes
  param_handle_ = this->add_on_set_parameters_callback(
    std::bind(&ControllerNode::parameterCallback, this, _1));

  // Declare parameters
  this->declare_parameter("tf_prefix", "");
  for (const auto & [param, default_value, description] :
    robot_->getParameters())
  {
    this->declare_parameter(param, default_value, description);
  }
  for (const auto & [param, default_value, description] :
    force_estimator_->getParameters())
  {
    this->declare_parameter(param, default_value, description);
  }

  // Define publishers and subscribers
  description_sub_ = this->create_subscription<String>(
    "robot_description", rclcpp::QoS(1).transient_local(),
    std::bind(&ControllerNode::descriptionCallback, this, _1));
  joint_sub_ = this->create_subscription<JointState>(
    "joint_states", 1,
    std::bind(&ControllerNode::jointCallback, this, _1));
  contact_cmd_sub_ = this->create_subscription<ContactCommand>(
    "contact_commands", 1,
    std::bind(&ControllerNode::contactCmdCallback, this, _1));
  joint_cmd_pub_ = this->create_publisher<JointCommand>("joint_commands", 1);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  RCLCPP_INFO(this->get_logger(), "Controller node initialized");
}

void ControllerNode::update()
{
  // Estimate contact forces
  Eigen::VectorXd forces = force_estimator_->update();

  // Update controller

  // Publish joint commands

  // Publish contact forces
  std::vector<WrenchStamped> contact_forces =
    force_estimator_->forcesToMessages(forces, this->get_clock()->now(), name_);
  for (size_t i = 0; i < contact_forces.size(); i++) {
    if (contact_force_pubs_.size() == i) {
      contact_force_pubs_.push_back(
        this->create_publisher<WrenchStamped>(
          "contact_force_" + std::to_string(i + 1), 10));
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

void ControllerNode::contactCmdCallback(const ContactCommand::SharedPtr msg)
{

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
    }
    bool initialized = robot_->isInitialized();
    robot_->setParameter(param, result);
    force_estimator_->setParameter(param, result);
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
