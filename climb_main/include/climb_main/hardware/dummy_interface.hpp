#ifndef DUMMY_INTERFACE_H
#define DUMMY_INTERFACE_H

#include <climb_main/hardware/hardware_interface.hpp>

/**
 * @brief Interface for communicating with a simulated robot for testing
 */
class DummyInterface : public HardwareInterface
{
public:
  bool connect() override;
  void disconnect() override;
  bool isConnected() override;
  bool enable(std::vector<int> ids) override;
  bool disable(std::vector<int> ids) override;

  inline void declareParameters(
    [[maybe_unused]] const rclcpp::Node::SharedPtr node) override {}
  inline void setParameter(
    [[maybe_unused]] const rclcpp::Parameter & param,
    [[maybe_unused]] rcl_interfaces::msg::SetParametersResult & result)
  override {}

  ActuatorState readActuatorState() override;
  JointState readJointState() override;
  bool writeJointCommand(JointCommand command) override;

private:
  bool connected_ = false;          // Flag for connection status
  std::map<int, bool> enabled_;     // Enable status by ID
  std::map<int, double> position_;  // Position by ID
  std::map<int, double> velocity_;  // Velocity by ID
  std::map<int, double> effort_;    // Effort by ID
};

#endif  // DUMMY_INTERFACE_H
