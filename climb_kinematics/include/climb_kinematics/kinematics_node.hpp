#ifndef CLIMB_KINEMATICS__KINEMATICS_NODE_HPP_
#define CLIMB_KINEMATICS__KINEMATICS_NODE_HPP_

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include "climb_kinematics/interfaces/kinematics_interface.hpp"

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
   * @brief Apply prefix to TF frame ID unless frame ID leads with a slash
   * @param frame_id TF frame ID to prefix with TF namespace
   * @return TF frame ID with prefix applied or with leading slash removed
   */
  std::string getPrefixedFrameId(const std::string & frame_id);

  /**
   * @brief Apply prefix to TF frame ID in place unless frame ID leads with a slash
   * @param[in, out] frame_id TF frame ID
   */
  void prefixFrameId(std::string & frame_id);

  /**
   * @brief Lookup most recent transform in TF buffer with TF prefix applied
   * to frame IDs (unless prefixed with a slash)
   * @param[in] parent_frame Parent frame of transform
   * @param[in] child_frame Child frame of transform
   * @param[in] time Requested time of transform
   * @return Transform from parent to child frame
   * @throws tf2::TransformException if transform is unavailable
   */
  TransformStamped lookupTransform(
    const std::string & parent_frame, const std::string & child_frame,
    const rclcpp::Time & time = rclcpp::Time(0));

  /**
   * @brief Send a TF transform with TF prefix applied to frame IDs
   * (unless prefixed with a slash)
   */
  void sendTransform(TransformStamped transform);

  /**
   * @brief Send an identity TF transform with TF prefix applied to frame IDs
   * @param[in] parent_frame Parent frame of transform
   * @param[in] child_frame Child frame of transform
   */
  void sendIdentityTransform(
    const std::string & parent_frame, const std::string & child_frame);

  /**
   * @brief Update modified parameters
   * @param[in] parameters Modified parameter values
   * @return Result of the parameter update
   */
  virtual rcl_interfaces::msg::SetParametersResult parameterCallback(
    const std::vector<rclcpp::Parameter> & parameters);

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
  // TF broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  // Parameter callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    param_handle_;
};

#endif  // CLIMB_KINEMATICS__KINEMATICS_NODE_HPP_
