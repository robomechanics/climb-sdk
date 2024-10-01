#ifndef DXL_INTERFACE_H
#define DXL_INTERFACE_H

#include <dynamixel_sdk/dynamixel_sdk.h>
#include "climb_main/hardware/hardware_interface.hpp"

/**
 * @brief Interface for communicating with a robot composed of Dynamixel motors
 */
class DynamixelInterface : public HardwareInterface
{
public:
  ~DynamixelInterface();

  bool connect() override;
  void disconnect() override;
  bool isConnected() override;
  bool enable(std::vector<int> ids) override;
  void disable(std::vector<int> ids) override;

  void declareParameters(const rclcpp::Node::SharedPtr node) override;
  void setParameter(
    const rclcpp::Parameter & param,
    rcl_interfaces::msg::SetParametersResult & result) override;

  ActuatorState readActuatorState() override;
  JointState readJointState() override;
  void writeJointState(JointCommand command) override;

  std::vector<double> readPosition(std::vector<int> ids);
  std::vector<double> readVelocity(std::vector<int> ids);
  std::vector<double> readEffort(std::vector<int> ids);
  std::vector<double> readTemperature(std::vector<int> ids);
  std::vector<double> readVoltage(std::vector<int> ids);
  std::vector<uint8_t> readError(std::vector<int> ids);

  void writePosition(std::vector<int> ids, std::vector<double> positions);
  void writeVelocity(
    std::vector<int> ids, std::vector<double> velocities, bool limit = false);
  void writeEffort(
    std::vector<int> ids, std::vector<double> efforts, bool limit = false);

private:
  std::shared_ptr<dynamixel::PortHandler> port_handler_;      // Port handler
  std::shared_ptr<dynamixel::PacketHandler> packet_handler_;  // Packet handler
  std::shared_ptr<dynamixel::GroupSyncWrite> group_write_;    // Sync write
  std::string port_name_;       // Serial port name
  int baud_rate_;               // Communication baud rate
  bool connected_ = false;      // Flag indicating if the port is open
};

#endif  // DXL_INTERFACE_H
