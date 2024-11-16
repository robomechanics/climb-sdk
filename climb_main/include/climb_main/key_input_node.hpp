#ifndef KEY_INPUT_NODE_HPP
#define KEY_INPUT_NODE_HPP

#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <climb_msgs/srv/key_input.hpp>
#include <keyboard_handler/keyboard_handler.hpp>

using climb_msgs::srv::KeyInput;

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
  void sendKeyInput(const std::string & input);

  /**
   * @brief Auto-complete input string
   * @param input The incomplete input string
   */
  void autoComplete(const std::string & input);

  // Keyboard handler
  KeyboardHandler keyboard_handler_;
  // Input string
  std::string input_;
  // Index in input string
  size_t index_;
  // Buffer of previous inputs
  std::vector<std::string> history_;
  // Index in history buffer
  size_t history_index_;
  // Send every key press individually
  bool realtime_;
  // Key input service
  rclcpp::Client<KeyInput>::SharedPtr key_input_client_;
  // Timeout for service requests
  std::chrono::milliseconds timeout_;
};

#endif  // KEY_INPUT_NODE_HPP
