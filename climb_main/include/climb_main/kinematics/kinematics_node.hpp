#ifndef KINEMATICS_NODE_HPP
#define KINEMATICS_NODE_HPP

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "climb_main/kinematics/kinematics_interface.hpp"

using std_msgs::msg::String;
using sensor_msgs::msg::JointState;

/**
 * @brief Parent class for nodes that use the kinematics interface
 *
 * Subscribers: joint_states, robot_description
 * Parameters: tf_prefix
 */
class KinematicsNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for KinematicsNode
   */
  KinematicsNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  /**
   * @brief Update the kinematics model with the provided robot description
   * @param[in] msg Message containing robot description
   */
  virtual void descriptionCallback(const String::SharedPtr msg);

  /**
   * @brief Update joint state with the latest data
   * @param[in] msg Message containing joint state
   */
  virtual void jointCallback(const JointState::SharedPtr msg);

  /**
   * @brief Update modified parameters
   * @param[in] parameters Modified parameter values
   * @return Result of the parameter update
   */
  virtual rcl_interfaces::msg::SetParametersResult parameterCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  // TF prefix
  std::string name_;
  // Kinematics interface
  std::shared_ptr<KinematicsInterface> robot_;
  // Tf buffer
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

private:
  // Robot description subscriber handle
  rclcpp::Subscription<String>::SharedPtr description_sub_;
  // Joint state subscriber handle
  rclcpp::Subscription<JointState>::SharedPtr joint_sub_;
  // Tf listener
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  // Parameter callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    param_handle_;
};

#endif  // KINEMATICS_NODE_HPP
