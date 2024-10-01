#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "climb_msgs/msg/actuator_state.hpp"
#include "climb_msgs/msg/joint_command.hpp"
#include "climb_msgs/srv/actuator_command.hpp"

#include "climb_main/hardware/hardware_interface.hpp"
#include "climb_main/hardware/dynamixel_interface.hpp"

using sensor_msgs::msg::JointState;
using climb_msgs::msg::JointCommand;
using climb_msgs::msg::ActuatorState;
using climb_msgs::srv::ActuatorCommand;

/**
 * @brief ROS node that interfaces with the robot's physical actuators
 *
 * Subscribers: joint_commands
 * Publishers: joint_states, actuator_states
 * Services: actuator_command
 */
class HardwareNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for HardwareNode
   */
  HardwareNode();

  /**
   * @brief Declare additional parameters
   */
  void init();

  /**
   * @brief Read and publish current joint and actuator values
   */
  void update();

private:
  // Hardware interface
  std::unique_ptr<HardwareInterface> interface_;
  // Joint state publisher
  rclcpp::Publisher<JointState>::SharedPtr joint_pub_;
  // Actuator state publisher
  rclcpp::Publisher<ActuatorState>::SharedPtr actuator_pub_;
  // Joint command subscriber
  rclcpp::Subscription<JointCommand>::SharedPtr joint_cmd_sub_;
  // Actuator command service (enable/disable)
  rclcpp::Service<ActuatorCommand>::SharedPtr actuator_cmd_srv_;
  // Parameter callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    param_handle_;

  // Actuator IDs
  std::vector<int> ids_;
  // Joint names corresponding to each actuator
  std::vector<std::string> joints_;
  // Actuator models
  std::vector<std::string> models_;

  /**
   * @brief Send commanded values to the joint actuators
   * @param[in] msg Message containing joint commands
   */
  void jointCmdCallback(const JointCommand::SharedPtr msg);

  /**
   * @brief Send commands to the actuators
   * @param[in] request Request containing actuator commands
   * @param[out] response Response containing command results
   */
  void actuatorCmdCallback(
    const ActuatorCommand::Request::SharedPtr request,
    ActuatorCommand::Response::SharedPtr response);

  /**
   * @brief Update modified parameters
   * @param[in] parameters Modified parameter values
   * @return Result of the parameter update
   */
  rcl_interfaces::msg::SetParametersResult parameterCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  /**
   * @brief Update actuators in the hardware interface
   */
  void updateActuators();
};
