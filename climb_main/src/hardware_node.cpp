#include "climb_main/hardware_node.hpp"
#include "climb_main/hardware/dynamixel_interface.hpp"
#include "climb_main/hardware/dummy_interface.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

HardwareNode::HardwareNode()
: Node("HardwareNode"),
  joint_update_period_(rclcpp::Duration::from_seconds(0)),
  actuator_update_period_(rclcpp::Duration::from_seconds(0)),
  last_joint_update_(0, 0, RCL_ROS_TIME),
  last_actuator_update_(0, 0, RCL_ROS_TIME)
{
  // Initialize robot interface
  interface_ = std::make_unique<DynamixelInterface>();

  // Subscribe to parameter changes
  param_handle_ = this->add_on_set_parameters_callback(
    std::bind(&HardwareNode::parameterCallback, this, _1));

  // Declare parameters
  this->declare_parameter("joint_names", std::vector<std::string>{});
  this->declare_parameter("actuator_ids", std::vector<int>{});
  this->declare_parameter("actuator_models", std::vector<std::string>{});
  this->declare_parameter("joint_update_freq", 0.0);
  this->declare_parameter("actuator_update_freq", 0.0);
  for (const auto & [param, default_value, description] :
    interface_->getParameters())
  {
    this->declare_parameter(param, default_value, description);
  }

  // Validate configuration
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

  // Attempt connection to robot
  RCLCPP_INFO(
    this->get_logger(),
    "Hardware node initialized; connecting to hardware interface...");
  if (interface_->connect()) {
    RCLCPP_INFO(this->get_logger(), "Connected to hardware interface");
    bool enabled = interface_->enable();
    if (enabled) {
      RCLCPP_INFO(this->get_logger(), "Enabled actuators");
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to enable actuators");
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "Failed to connect to hardware interface");
    interface_ = std::make_unique<DummyInterface>();
    updateActuators();
    interface_->connect();
    interface_->enable();
    RCLCPP_INFO(this->get_logger(), "Switching to dummy interface");
  }
}

void HardwareNode::update()
{
  auto now = this->get_clock()->now();
  if (now - last_joint_update_ >= joint_update_period_) {
    last_joint_update_ = now;
    auto js = interface_->readJointState();
    js.header.stamp = this->get_clock()->now();
    this->joint_pub_->publish(js);
  }

  now = this->get_clock()->now();
  if (now - last_actuator_update_ >= actuator_update_period_) {
    last_actuator_update_ = now;
    auto as = interface_->readActuatorState();
    as.header.stamp = this->get_clock()->now();
    this->actuator_pub_->publish(as);
  }
}

void HardwareNode::jointCmdCallback(const JointCommand::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received joint command");
  if (!interface_->validateJointCommand(*msg)) {
    RCLCPP_WARN(this->get_logger(), "Invalid joint command");
    return;
  }
  bool success = interface_->writeJointCommand(*msg);
  if (!success) {
    RCLCPP_WARN(this->get_logger(), "Failed to send joint command");
  } else {
    RCLCPP_INFO(this->get_logger(), "Sent joint command");
  }
}

void HardwareNode::actuatorCmdCallback(
  const ActuatorCommand::Request::SharedPtr request,
  ActuatorCommand::Response::SharedPtr response)
{
  std::vector<int> ids;
  if (request->id.empty()) {
    if (request->joint.empty()) {
      ids = interface_->getIds();
    } else {
      for (const auto & joint : request->joint) {
        for (int id : interface_->getId(joint)) {
          ids.push_back(id);
        }
      }
    }
  } else {
    for (int id : request->id) {
      ids.push_back(id);
    }
  }
  if (request->enable) {
    response->success = interface_->enable(ids);
  } else {
    response->success = interface_->disable(ids);
  }
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
    } else if (param.get_name() == "joint_names") {
      joints_ = param.as_string_array();
      updateActuators();
    } else if (param.get_name() == "actuator_models") {
      models_ = param.as_string_array();
      updateActuators();
    } else if (param.get_name() == "joint_update_freq") {
      double freq = param.as_double();
      joint_update_period_ =
        rclcpp::Duration::from_seconds((freq > 0) ? (1.0 / freq) : 0);
    } else if (param.get_name() == "actuator_update_freq") {
      double freq = param.as_double();
      actuator_update_period_ =
        rclcpp::Duration::from_seconds((freq > 0) ? (1.0 / freq) : 0);
    }
    interface_->setParameter(param, result);
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
    for (size_t i = 0; i < ids_.size(); i++) {
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

  while (rclcpp::ok()) {
    node->update();
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();
  return 0;
}
