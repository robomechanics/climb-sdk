#include "climb_teleop/key_input_node.hpp"

using KeyCode = KeyboardHandler::KeyCode;
using KeyModifiers = KeyboardHandler::KeyModifiers;
using std::placeholders::_1;

KeyInputNode::KeyInputNode()
: Node("KeyInputNode"), history_({""})
{
  // Subscribe to all key events
  for (
    auto key = KeyCode::UNKNOWN; key != KeyCode::END_OF_KEY_CODE_ENUM; ++key)
  {
    keyboard_handler_.add_key_press_callback(
      [this](KeyCode key_code, KeyModifiers key_modifiers) {
        this->keyCallback(key_code, key_modifiers);
      }, key);
    keyboard_handler_.add_key_press_callback(
      [this](KeyCode key_code, KeyModifiers key_modifiers) {
        this->keyCallback(key_code, key_modifiers);
      }, key, KeyboardHandler::KeyModifiers::SHIFT);
  }

  // Get service name as parameter
  auto service_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  service_param_desc.description = "Name of TeleopInput service";
  service_param_desc.read_only = true;
  declare_parameter("service", "teleop_input", service_param_desc);
  std::string service = get_parameter("service").as_string();

  // Get topic name as parameter
  auto topic_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  topic_param_desc.description =
    "Name of TeleopOutput topic for asynchronous responses";
  topic_param_desc.read_only = true;
  declare_parameter("topic", "teleop_output", topic_param_desc);
  std::string topic = get_parameter("topic").as_string();

  // Timeout for service requests
  auto timeout_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  timeout_param_desc.description = "Timeout for service requests in ms";
  timeout_param_desc.read_only = true;
  declare_parameter("timeout", 1000, timeout_param_desc);
  timeout_ = std::chrono::milliseconds(get_parameter("timeout").as_int());

  // Initialize service and subscriber
  teleop_input_client_ = create_client<TeleopInput>(service);
  async_response_sub_ = create_subscription<TeleopOutput>(
    topic, 1, std::bind(&KeyInputNode::teleopOutputCallback, this, _1));
}

void KeyInputNode::keyCallback(KeyCode key_code, KeyModifiers key_modifiers)
{
  if (realtime_) {
    auto input = keyboard_handler_.get_terminal_sequence(key_code);
    if (key_modifiers == KeyModifiers::SHIFT) {
      std::transform(input.begin(), input.end(), input.begin(), ::toupper);
    }
    if (!input.empty()) {
      sendTeleopInput(input);
    }
    return;
  }
  switch (key_code) {
    case KeyCode::ENTER:          // Send input
      if (!input_.empty()) {
        sendTeleopInput(input_);
      }
      if (history_index_ > 0) {
        history_.erase(history_.begin() + history_index_);
      }
      history_.front() = input_;
      history_.insert(history_.begin(), "");
      input_.clear();
      index_ = 0;
      history_index_ = 0;
      break;
    case KeyCode::BACK_SPACE:     // Delete previous character
      if (index_ > 0) {
        input_.erase(index_ - 1, 1);
        --index_;
      }
      break;
    case KeyCode::DELETE_KEY:     // Delete next character
      if (index_ < input_.size()) {
        input_.erase(index_, 1);
      }
      break;
    case KeyCode::CURSOR_LEFT:    // Move cursor left
      if (index_ > 0) {
        --index_;
      }
      break;
    case KeyCode::CURSOR_RIGHT:   // Move cursor right
      if (index_ < input_.size()) {
        ++index_;
      } else {
        autoComplete(input_);
      }
      break;
    case KeyCode::HOME:           // Move cursor to start
      index_ = 0;
      break;
    case KeyCode::END:            // Move cursor to end
      index_ = input_.size();
      break;
    case KeyCode::PG_UP:          // Move to first input
      history_[history_index_] = input_;
      history_index_ = history_.size() - 1;
      input_ = history_[history_index_];
      index_ = input_.size();
      break;
    case KeyCode::PG_DOWN:        // Move to last input
      history_[history_index_] = input_;
      history_index_ = 0;
      input_ = history_[history_index_];
      index_ = input_.size();
      break;
    case KeyCode::CURSOR_UP:      // Move to previous input
      if (history_index_ < history_.size() - 1) {
        history_[history_index_] = input_;
        ++history_index_;
        input_ = history_[history_index_];
        index_ = input_.size();
      }
      break;
    case KeyCode::CURSOR_DOWN:    // Move to next input
      if (history_index_ > 0) {
        history_[history_index_] = input_;
        --history_index_;
        input_ = history_[history_index_];
        index_ = input_.size();
      }
      break;
    default:                      // Insert character
      auto input = keyboard_handler_.get_terminal_sequence(key_code);
      if (key_modifiers == KeyModifiers::SHIFT) {
        std::transform(input.begin(), input.end(), input.begin(), ::toupper);
      }
      input_.insert(index_, input);
      index_ += input.size();
      history_[history_index_] = input_;
  }
  std::cout << "\33[2K\r" << input_ << "\33[" << index_ + 1 << "G" << std::flush;
}

void KeyInputNode::sendTeleopInput(const std::string & input)
{
  std::cout << "\33[2K\r" << std::flush;
  auto request = std::make_shared<TeleopInput::Request>();
  request->header.stamp = now();
  request->command = input;
  request->realtime = realtime_;
  auto result = teleop_input_client_->async_send_request(request);
  auto status = result.wait_for(timeout_);
  if (status == std::future_status::ready) {
    auto response = result.get()->response;
    realtime_ = response.realtime;
    if (!response.message.empty()) {
      std::cout << response.message << std::endl;
    }
  } else {
    std::cout << "Server is not responding" << std::endl;
  }
}

void KeyInputNode::autoComplete(const std::string & input)
{
  auto request = std::make_shared<TeleopInput::Request>();
  request->header.stamp = now();
  request->command = input;
  request->realtime = realtime_;
  request->autocomplete = true;
  auto result = teleop_input_client_->async_send_request(request);
  auto status = result.wait_for(std::chrono::seconds(1));
  if (status == std::future_status::ready) {
    input_ = result.get()->response.message;
    index_ = input_.size();
  }
}

void KeyInputNode::teleopOutputCallback(const TeleopOutput::SharedPtr msg)
{
  realtime_ = msg->realtime;
  if (!msg->message.empty()) {
    std::cout << msg->message << std::endl;
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyInputNode>());
  rclcpp::shutdown();
  return 0;
}
