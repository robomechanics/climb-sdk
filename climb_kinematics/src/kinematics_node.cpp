#include "climb_kinematics/kinematics_node.hpp"

#include "climb_kinematics/interfaces/kdl_interface.hpp"

using std::placeholders::_1;

KinematicsNode::KinematicsNode(
  const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  // Initialize kinematics model
  robot_ = std::make_shared<KdlInterface>();

  // Subscribe to parameter changes
  param_handle_ = add_on_set_parameters_callback(
    std::bind(&KinematicsNode::parameterCallback, this, _1));
  for (const auto & p : robot_->getParameters()) {
    declare_parameter(p.name, p.default_value, p.descriptor);
  }

  // Define publishers and subscribers
  description_sub_ = create_subscription<String>(
    "robot_description", rclcpp::QoS(1).transient_local(),
    std::bind(&KinematicsNode::descriptionCallback, this, _1));
  joint_sub_ = create_subscription<JointState>(
    "joint_states", 1,
    std::bind(&KinematicsNode::jointCallback, this, _1));
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void KinematicsNode::descriptionCallback(const String::SharedPtr msg)
{
  std::string error_message;
  if (!robot_->loadRobotDescription(msg->data, error_message)) {
    RCLCPP_ERROR(
      get_logger(),
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
      transforms.emplace_back(
        lookupTransform(
          robot_->getEndEffectorFrames().at(i),
          robot_->getContactFrames().at(i)));
    } catch (const tf2::TransformException & ex) {
    }
  }
  robot_->updateContactFrames(transforms);
}

std::string KinematicsNode::getPrefixedFrameId(const std::string & frame_id)
{
  if (frame_id.empty()) {
    throw std::invalid_argument("TF frame name cannot be empty");
  }
  return frame_id.at(0) == '/' ?
         frame_id.substr(1) :
         std::string(get_namespace()).substr(1) + "/" + frame_id;
}

void KinematicsNode::prefixFrameId(std::string & frame_id)
{
  frame_id = getPrefixedFrameId(frame_id);
}

TransformStamped KinematicsNode::lookupTransform(
  const std::string & parent_frame, const std::string & child_frame,
  const rclcpp::Time & time)
{
  std::string parent = getPrefixedFrameId(parent_frame);
  std::string child = getPrefixedFrameId(child_frame);
  TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(parent, child, time);
  } catch (const tf2::TransformException & ex) {
    // Fallback to body frame if global map and odom are not yet published
    if (parent_frame == "/odom" || parent_frame == "/map") {
      transform = lookupTransform(robot_->getBodyFrame(), child_frame);
    } else if (child_frame == "/odom" || child_frame == "/map") {
      transform = lookupTransform(parent_frame, robot_->getBodyFrame());
    } else {
      throw(ex);
    }
  }
  transform.header.frame_id = parent_frame;
  transform.child_frame_id = child_frame;
  return transform;
}

void KinematicsNode::sendTransform(TransformStamped transform)
{
  prefixFrameId(transform.header.frame_id);
  prefixFrameId(transform.child_frame_id);
  tf_broadcaster_->sendTransform(transform);
}

void KinematicsNode::sendIdentityTransform(
  const std::string & parent_frame, const std::string & child_frame)
{
  TransformStamped transform;
  transform.header.frame_id = parent_frame;
  transform.child_frame_id = child_frame;
  transform.transform.rotation.w = 1;
  transform.header.stamp = now();
  sendTransform(transform);
}

rcl_interfaces::msg::SetParametersResult KinematicsNode::parameterCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "Success";
  for (const auto & param : parameters) {
    bool initialized = robot_->isInitialized();
    robot_->setParameter(param, result);
    if (!initialized && robot_->isInitialized()) {
      RCLCPP_INFO(get_logger(), "Kinematics interface initialized");
    } else if (initialized && !robot_->isInitialized()) {
      RCLCPP_WARN(
        get_logger(),
        "Kinematics interface disabled: %s", result.reason.c_str());
    }
  }
  return result;
}
