#include "climb_robot_driver/robot_driver_node.hpp"

#include "climb_robot_driver/interfaces/dummy_interface.hpp"
#include "climb_robot_driver/interfaces/dynamixel_interface.hpp"

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
  param_handle_ = add_on_set_parameters_callback(
    std::bind(&HardwareNode::parameterCallback, this, _1));

  // Declare parameters
  declare_parameter("joint_names", std::vector<std::string>{});
  declare_parameter("actuator_ids", std::vector<int>{});
  declare_parameter("actuator_models", std::vector<std::string>{});
  declare_parameter("joint_update_freq", 0.0);
  declare_parameter("actuator_update_freq", 0.0);
  for (const auto & p : interface_->getParameters()) {
    declare_parameter(p.name, p.default_value, p.descriptor);
  }

  // Validate configuration
  if (ids_.empty()) {
    RCLCPP_WARN(get_logger(), "Robot configuration has no actuators");
  }

  // Define publishers and subscribers
  joint_pub_ = create_publisher<JointState>("joint_states", 1);
  actuator_pub_ = create_publisher<ActuatorState>("actuator_states", 1);
  joint_cmd_sub_ = create_subscription<JointCommand>(
    "joint_commands", 1,
    std::bind(&HardwareNode::jointCmdCallback, this, _1));
  actuator_enable_srv_ = create_service<ActuatorEnable>(
    "actuator_enable",
    std::bind(&HardwareNode::actuatorEnableCallback, this, _1, _2));

  // Attempt connection to robot
  RCLCPP_INFO(
    get_logger(),
    "Hardware node initialized; connecting to hardware interface...");
  if (interface_->connect()) {
    RCLCPP_INFO(get_logger(), "Connected to hardware interface");
    bool enabled = interface_->enable();
    if (enabled) {
      RCLCPP_INFO(get_logger(), "Enabled actuators");
    } else {
      RCLCPP_WARN(get_logger(), "Failed to enable actuators");
    }
  } else {
    RCLCPP_WARN(get_logger(), "Failed to connect to hardware interface");
    interface_ = std::make_unique<DummyInterface>();
    updateActuators();
    for (const auto & p : interface_->getParameters()) {
      declare_parameter(p.name, p.default_value, p.descriptor);
    }
    interface_->connect();
    interface_->enable();
    RCLCPP_INFO(get_logger(), "Switching to dummy interface");
  }
}

void HardwareNode::update()
{
  if (now() - last_joint_update_ >= joint_update_period_) {
    last_joint_update_ = now();
    auto js = interface_->readJointState();
    js.header.stamp = now();
    joint_pub_->publish(js);
  }

  if (now() - last_actuator_update_ >= actuator_update_period_) {
    last_actuator_update_ = now();
    auto as = interface_->readActuatorState();
    as.header.stamp = now();
    actuator_pub_->publish(as);
  }
}

void HardwareNode::jointCmdCallback(const JointCommand::SharedPtr msg)
{
  bool success = interface_->writeJointCommand(*msg);
  if (!success) {
    RCLCPP_WARN(get_logger(), "Failed to send joint command");
  }
}

void HardwareNode::actuatorEnableCallback(
  const ActuatorEnable::Request::SharedPtr request,
  ActuatorEnable::Response::SharedPtr response)
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
  if (!response->success) {
    response->message = request->enable ?
      "Failed to enable actuators" : "Failed to disable actuators";
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
      get_logger(),
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
