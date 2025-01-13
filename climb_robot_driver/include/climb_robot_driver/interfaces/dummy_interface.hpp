#ifndef CLIMB_ROBOT_DRIVER__INTERFACES__DUMMY_INTERFACE_HPP_
#define CLIMB_ROBOT_DRIVER__INTERFACES__DUMMY_INTERFACE_HPP_

#include <map>
#include <string>
#include <vector>

#include "climb_robot_driver/interfaces/hardware_interface.hpp"

/**
 * @brief Interface for communicating with a simulated robot for testing
 */
class DummyInterface : public HardwareInterface
{
public:
  void addActuators(
    const std::vector<int> & ids, const std::vector<std::string> & joints,
    const std::string & model, double ratio) override;
  void removeActuators(const std::vector<int> & ids) override;
  bool connect() override;
  void disconnect() override;
  bool isConnected() override;
  bool enable(const std::vector<int> & ids) override;
  bool disable(const std::vector<int> & ids) override;

  void declareParameters() override;
  void setParameter(
    const Parameter & param, SetParametersResult & result) override;
  using Parameterized::setParameter;

  ActuatorState readActuatorState() override;
  JointState readJointState() override;
  bool writeJointCommand(const JointCommand & command) override;

private:
  bool connected_ = false;          // Flag for connection status
  std::map<int, bool> enabled_;     // Enable status by ID
  std::map<int, double> position_;  // Position by ID
  std::map<int, double> velocity_;  // Velocity by ID
  std::map<int, double> effort_;    // Effort by ID
};

#endif  // CLIMB_ROBOT_DRIVER__INTERFACES__DUMMY_INTERFACE_HPP_
