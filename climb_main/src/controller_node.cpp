#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "climb_msgs/msg/contact_state.hpp"

#include "climb_main/kinematics/kinematics_interface.hpp"
#include "climb_main/kinematics/kdl_interface.hpp"

using std::placeholders::_1;
using std_msgs::msg::String;
using sensor_msgs::msg::JointState;
using climb_msgs::msg::ContactState;

/**
 * Computes optimal joint displacements given current state, desired contact
 * mode, and desired free-space motion.
 */
class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode()
  : Node("ControllerNode")
  {
    this->declare_parameter("tf_prefix", "");
    name_ = this->get_parameter("tf_prefix").as_string();

    robot_ = std::make_unique<KdlInterface>();
    description_sub_ = this->create_subscription<String>(
      "robot_description", 1,
      std::bind(&ControllerNode::description_callback, this, _1));
    joint_sub_ = this->create_subscription<JointState>(
      "joint_states", 1,
      std::bind(&ControllerNode::joint_callback, this, _1));
    joint_cmd_pub_ = this->create_publisher<JointState>("joint_commands", 1);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    RCLCPP_INFO(this->get_logger(), "Controller node initialized");
  }

private:
  std::unique_ptr<KinematicsInterface> robot_;

  rclcpp::Subscription<String>::SharedPtr description_sub_;
  rclcpp::Subscription<JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<ContactState>::SharedPtr contact_cmd_sub_;

  rclcpp::Publisher<JointState>::SharedPtr joint_cmd_pub_;
  rclcpp::Publisher<ContactState>::SharedPtr contact_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string name_;

  /**
   * @brief the kinematics model using the provided robot description
   * @param[in] msg Message containing robot description in URDF format
   */
  void description_callback(const String::SharedPtr msg)
  {
    robot_->loadRobotDescription(msg->data);
  }

  /**
   * @brief Publish robot state corresponding to the provided joint states
   * @param[in] msg Message containing joint states
   */
  void joint_callback(const JointState::SharedPtr msg)
  {
    // ContactState contact_state;
    // this->contact_pub_->publish(contact_state);

    // geometry_msgs::msg::TransformStamped t;
    // t.header.stamp = this->get_clock()->now();
    // t.header.frame_id = "world";
    // t.child_frame_id = "world";
    // tf_broadcaster_->sendTransform(t);
  }

  /**
   * @brief Publish joint commands to achieve the provided contact commands
   * @param[in] msg Message containing contact commands
   */
  void contact_cmd_callback(const ContactState::SharedPtr msg)
  {
    JointState joint_cmd;
    this->joint_cmd_pub_->publish(joint_cmd);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_unique<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
