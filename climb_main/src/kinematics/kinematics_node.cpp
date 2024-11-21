#include "climb_main/kinematics/kinematics_node.hpp"
#include "climb_main/kinematics/kdl_interface.hpp"

using std::placeholders::_1;

KinematicsNode::KinematicsNode(
  const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  // Initialize kinematics model
  robot_ = std::make_shared<KdlInterface>();

  // Subscribe to parameter changes
  param_handle_ = this->add_on_set_parameters_callback(
    std::bind(&KinematicsNode::parameterCallback, this, _1));
  this->declare_parameter("tf_prefix", "");
  for (const auto & p : robot_->getParameters()) {
    this->declare_parameter(p.name, p.default_value, p.descriptor);
  }

  // Define publishers and subscribers
  description_sub_ = this->create_subscription<String>(
    "robot_description", rclcpp::QoS(1).transient_local(),
    std::bind(&KinematicsNode::descriptionCallback, this, _1));
  joint_sub_ = this->create_subscription<JointState>(
    "joint_states", 1,
    std::bind(&KinematicsNode::jointCallback, this, _1));
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void KinematicsNode::descriptionCallback(const String::SharedPtr msg)
{
  std::string error_message;
  if (robot_->loadRobotDescription(msg->data, error_message)) {
    RCLCPP_INFO(this->get_logger(), "Robot description loaded");
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to load robot description: %s", error_message.c_str());
    return;
  }
}

void KinematicsNode::jointCallback(const JointState::SharedPtr msg)
{
  if (!robot_->isInitialized()) {
    return;
  }
  robot_->updateJointState(*msg);
  std::vector<TransformStamped> transforms;
  for (auto i = 0; i < robot_->getNumContacts(); i++) {
    try {
      auto ee_frame = robot_->getEndEffectorFrames().at(i);
      auto contact_frame = robot_->getContactFrames().at(i);
      auto transform = tf_buffer_->lookupTransform(
        name_ + "/" + ee_frame, name_ + "/" + contact_frame,
        tf2::TimePointZero);
      transform.header.frame_id = ee_frame;
      transform.child_frame_id = contact_frame;
      transforms.push_back(transform);
    } catch (const tf2::TransformException & ex) {
    }
  }
  robot_->updateContactFrames(transforms);
}

rcl_interfaces::msg::SetParametersResult KinematicsNode::parameterCallback(
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
