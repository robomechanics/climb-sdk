#ifndef DUMMY_INTERFACE_HPP
#define DUMMY_INTERFACE_HPP

#include <climb_main/hardware/hardware_interface.hpp>

/**
 * @brief Interface for communicating with a simulated robot for testing
 */
class DummyInterface : public HardwareInterface
{
public:
  void addActuators(
    std::vector<int> ids, std::vector<std::string> joints,
    std::string model, double ratio) override;
  void removeActuators(std::vector<int> ids) override;
  bool connect() override;
  void disconnect() override;
  bool isConnected() override;
  bool enable(std::vector<int> ids) override;
  bool disable(std::vector<int> ids) override;

  void declareParameters() override;
  void setParameter(
    const Parameter & param, SetParametersResult & result) override;
  using Parameterized::setParameter;

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

#endif  // DUMMY_INTERFACE_HPP
