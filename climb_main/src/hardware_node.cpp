#include "climb_main/hardware_node.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

HardwareNode::HardwareNode()
: Node("HardwareNode")
{
  // Initialize robot interface
  interface_ = std::make_unique<DynamixelInterface>();

  // Subscribe to parameter changes
  param_handle_ = this->add_on_set_parameters_callback(
    std::bind(&HardwareNode::parameterCallback, this, _1));

  // Declare parameters
  this->declare_parameter("actuator_ids", std::vector<int>{});
  this->declare_parameter("actuator_joints", std::vector<std::string>{});
  this->declare_parameter("actuator_models", std::vector<std::string>{});
  if (ids_.empty()) {
    RCLCPP_WARN(this->get_logger(), "Robot configuration has no actuators");
  }

  // Define publishers and subscribers
  joint_pub_ = this->create_publisher<JointState>("joint_states", 1);
  actuator_pub_ = this->create_publisher<ActuatorState>("actuator_states", 1);
  joint_cmd_sub_ = this->create_subscription<JointCommand>(
    "joint_commands", 1,
    std::bind(&HardwareNode::jointCmdCallback, this, _1));
  actuator_cmd_srv_ = this->create_service<ActuatorCommand>(
    "actuator_command",
    std::bind(&HardwareNode::actuatorCmdCallback, this, _1, _2));

  RCLCPP_INFO(this->get_logger(), "Hardware node initialized");
}

void HardwareNode::init()
{
  interface_->declareParameters(shared_from_this());
  RCLCPP_INFO(this->get_logger(), "Parameters loaded; trying to connect...");
  if (interface_->connect()) {
    RCLCPP_INFO(this->get_logger(), "Connected to hardware interface");
    bool enabled = interface_->enable();
    if (enabled) {
      RCLCPP_INFO(this->get_logger(), "Enabled actuators");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to enable actuators");
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect to hardware interface");
  }
}

void HardwareNode::update()
{
  auto js = JointState();
  js.header.stamp = this->get_clock()->now();
  for (unsigned int i = 0; i < ids_.size(); i++) {
    js.name.push_back(joints_[i]);
    js.position.push_back(0);
  }
  this->joint_pub_->publish(js);

  auto ms = ActuatorState();
  this->actuator_pub_->publish(ms);

  // Implement reading joint/motor states
}

void HardwareNode::jointCmdCallback(const JointCommand::SharedPtr msg)
{
  // Implement sending joint commands
}

void HardwareNode::actuatorCmdCallback(
  const ActuatorCommand::Request::SharedPtr request,
  ActuatorCommand::Response::SharedPtr response)
{
  // Implement sending actuator commands
}

rcl_interfaces::msg::SetParametersResult HardwareNode::parameterCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "Success";
  for (const auto & param : parameters) {
    if (param.get_name() == "actuator_ids") {
      std::vector<int64_t> ids = param.as_integer_array();
      ids_.clear();
      ids_.reserve(ids.size());
      for (const auto & id : ids) {
        ids_.push_back(static_cast<int>(id));
      }
      updateActuators();
    } else if (param.get_name() == "actuator_joints") {
      joints_ = param.as_string_array();
      updateActuators();
    } else if (param.get_name() == "actuator_models") {
      models_ = param.as_string_array();
      updateActuators();
    } else {
      interface_->setParameter(param, result);
    }
  }
  return result;
}

void HardwareNode::updateActuators()
{
  interface_->removeActuators();
  if (ids_.empty() || joints_.empty() || models_.empty()) {
    return;
  }
  if (models_.size() == 1 && joints_.size() == ids_.size()) {
    interface_->addActuators(ids_, joints_, models_[0]);
  } else if (models_.size() == ids_.size() && joints_.size() == ids_.size()) {
    for (unsigned int i = 0; i < ids_.size(); i++) {
      interface_->addActuators({ids_[i]}, {joints_[i]}, models_[i]);
    }
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "Mismatched actuator IDs, joints, and models in robot configuration");
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HardwareNode>();
  node->init();

  while (rclcpp::ok()) {
    node->update();
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();
  return 0;
}
