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
  service_param_desc.description = "Name of KeyInput service";
  service_param_desc.read_only = true;
  this->declare_parameter("service", "key_input", service_param_desc);
  std::string service = this->get_parameter("service").as_string();

  // Get topic name as parameter
  auto topic_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  topic_param_desc.description =
    "Name of KeyInput topic for asynchronous responses";
  topic_param_desc.read_only = true;
  this->declare_parameter("topic", "key_output", topic_param_desc);
  std::string topic = this->get_parameter("topic").as_string();

  // Timeout for service requests
  auto timeout_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  timeout_param_desc.description = "Timeout for service requests in ms";
  timeout_param_desc.read_only = true;
  this->declare_parameter("timeout", 1000, timeout_param_desc);
  timeout_ = std::chrono::milliseconds(
    this->get_parameter("timeout").as_int());

  // Initialize service and subscriber
  key_input_client_ = this->create_client<KeyInput>(service);
  async_response_sub_ = this->create_subscription<TeleopMessage>(
    topic, 1, std::bind(&KeyInputNode::keyOutputCallback, this, _1));
}

void KeyInputNode::keyCallback(KeyCode key_code, KeyModifiers key_modifiers)
{
  if (realtime_) {
    auto input = keyboard_handler_.get_terminal_sequence(key_code);
    if (key_modifiers == KeyModifiers::SHIFT) {
      std::transform(input.begin(), input.end(), input.begin(), ::toupper);
    }
    if (!input.empty()) {
      sendKeyInput(input);
    }
    return;
  }
  switch (key_code) {
    case KeyCode::ENTER:          // Send input
      if (!input_.empty()) {
        sendKeyInput(input_);
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

void KeyInputNode::sendKeyInput(const std::string & input)
{
  std::cout << "\33[2K\r" << std::flush;
  auto request = std::make_shared<KeyInput::Request>();
  request->header.stamp = this->now();
  request->input = input;
  request->realtime = realtime_;
  auto result = key_input_client_->async_send_request(request);
  auto status = result.wait_for(timeout_);
  if (status == std::future_status::ready) {
    auto response = result.get();
    realtime_ = response->realtime;
    if (!response->response.empty()) {
      std::cout << response->response << std::endl;
    }
  } else {
    std::cout << "Server is not responding" << std::endl;
  }
}

void KeyInputNode::autoComplete(const std::string & input)
{
  auto request = std::make_shared<KeyInput::Request>();
  request->header.stamp = this->now();
  request->input = input;
  request->realtime = realtime_;
  request->autocomplete = true;
  auto result = key_input_client_->async_send_request(request);
  auto status = result.wait_for(std::chrono::seconds(1));
  if (status == std::future_status::ready) {
    input_ = result.get()->response;
    index_ = input_.size();
  }
}

void KeyInputNode::keyOutputCallback(const TeleopMessage::SharedPtr msg)
{
  realtime_ = msg->realtime;
  if (!msg->response.empty()) {
    std::cout << msg->response << std::endl;
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyInputNode>());
  rclcpp::shutdown();
  return 0;
}
