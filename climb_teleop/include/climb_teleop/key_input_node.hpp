#ifndef CLIMB_TELEOP__KEY_INPUT_NODE_HPP_
#define CLIMB_TELEOP__KEY_INPUT_NODE_HPP_

#include <chrono>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <keyboard_handler/keyboard_handler.hpp>

#include <climb_msgs/msg/teleop_output.hpp>
#include <climb_msgs/srv/teleop_input.hpp>

using climb_msgs::msg::TeleopOutput;
using climb_msgs::srv::TeleopInput;

/**
 * @brief ROS node that parses keyboard input and displays the response
 *
 * Clients: teleop_input
 */
class KeyInputNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for KeyInputNode
   */
  KeyInputNode();

private:
  /**
   * @brief Callback for keyboard input
   * @param key_code The key code of the pressed key
   * @param key_modifiers The modifiers of the pressed key
   */
  void keyCallback(
    KeyboardHandler::KeyCode key_code,
    KeyboardHandler::KeyModifiers key_modifiers);

  /**
   * @brief Send input string to service
   * @param input The input string
   */
  void sendTeleopInput(const std::string & input);

  /**
   * @brief Auto-complete input string
   * @param input The incomplete input string
   */
  void autoComplete(const std::string & input);

  /**
   * @brief Callback for asynchronous responses
   * @param msg The response message
   */
  void teleopOutputCallback(const TeleopOutput::SharedPtr msg);

  // Keyboard handler
  KeyboardHandler keyboard_handler_;
  // Input string
  std::string input_;
  // Index in input string
  size_t index_ = 0;
  // Buffer of previous inputs
  std::vector<std::string> history_;
  // Index in history buffer
  size_t history_index_ = 0;
  // Send every key press individually
  bool realtime_ = false;
  // Key input service client
  rclcpp::Client<TeleopInput>::SharedPtr teleop_input_client_;
  // Async response subscriber
  rclcpp::Subscription<TeleopOutput>::SharedPtr async_response_sub_;
  // Timeout for service requests
  std::chrono::milliseconds timeout_;
};

#endif  // CLIMB_TELEOP__KEY_INPUT_NODE_HPP_
