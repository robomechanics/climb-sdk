#ifndef DXL_INTERFACE_H
#define DXL_INTERFACE_H

#include <dynamixel_sdk/dynamixel_sdk.h>
#include "climb_main/hardware/hardware_interface.hpp"

/**
 * @brief Interface for communicating with a robot composed of Dynamixel motors
 *
 * TODO: Currently only supports XM430-W350-T actuators. Additional XM series
 * actuators must specify the correct torque constant in Nm/mA. Other series
 * must additionally specify the appropriate Dynamixel protocol (1.0 or 2.0)
 * and control table indices.
 */
class DynamixelInterface : public HardwareInterface
{
public:
  ~DynamixelInterface();

  void addActuators(
    std::vector<int> ids,
    std::vector<std::string> joints, std::string model, double ratio);
  bool connect() override;
  void disconnect() override;
  bool isConnected() override;
  bool enable(std::vector<int> ids) override;
  bool disable(std::vector<int> ids) override;

  void declareParameters(const rclcpp::Node::SharedPtr node) override;
  void setParameter(
    const rclcpp::Parameter & param,
    rcl_interfaces::msg::SetParametersResult & result)
  override;

  ActuatorState readActuatorState() override;
  JointState readJointState() override;
  void writeJointState(JointCommand command) override;

  /**
   * @brief Sync read data from the control table of a set of Dynamixels
   * @param[in] ids Actuator IDs
   * @param[in] item Index and length of Dynamixel control table item
   * @param[in] protocol Dynamixel protocol version (1.0 or 2.0)
   * @return Vector of data received (or empty vector if read failed)
   */
  std::vector<double> read(
    std::vector<int> ids, std::tuple<int, int> item,
    float protocol = 2.0);

  /**
   * @brief Sync write data to the control table of a set of Dynamixels
   * @param[in] ids Actuator IDs
   * @param[in] item Index and length of Dynamixel control table item
   * @param[in] data Data to write (one value per actuator or one value for all)
   * @param[in] protocol Dynamixel protocol version (1.0 or 2.0)
   * @return True if the write was successful
   */
  bool write(
    std::vector<int> ids, std::tuple<int, int> item,
    std::vector<double> data, float protocol = 2.0);

  /**
   * @brief Convert a double to a vector of bytes for dynamixel write
   * @param[in] value Double value to convert
   * @param[in] length Number of bytes
   * @return Vector of bytes
   */
  std::vector<uint8_t> toBytes(double value, int length);

  /**
   * @brief Get a bit from an integer
   * @param[in] value Integer value
   * @param[in] bit Bit index
   * @return True if the bit is set
   */
  bool getBit(int value, int bit);

  std::vector<double> readPosition(std::vector<int> ids);
  std::vector<double> readVelocity(std::vector<int> ids);
  std::vector<double> readEffort(std::vector<int> ids);
  std::vector<double> readTemperature(std::vector<int> ids);
  std::vector<double> readVoltage(std::vector<int> ids);
  std::vector<uint8_t> readError(std::vector<int> ids);

  bool writePosition(std::vector<int> ids, std::vector<double> position);
  bool writeVelocity(
    std::vector<int> ids, std::vector<double> velocity, bool limit = false);
  bool writeEffort(
    std::vector<int> ids, std::vector<double> effort, bool limit = false);

private:
  // Dynamixel serial port handler
  std::shared_ptr<dynamixel::PortHandler> port_handler_;
  // Dynamixel serial packet handler for protocol 1.0
  std::shared_ptr<dynamixel::PacketHandler> packet_handler_1_;
  // Dynamixel serial packet handler for protocol 2.0
  std::shared_ptr<dynamixel::PacketHandler> packet_handler_2_;
  // Group sync write instances for each control table item (protocol 1.0)
  std::map<std::tuple<int, int>, std::shared_ptr<dynamixel::GroupSyncWrite>>
  group_write_1_;
  // Group sync write instances for each control table item (protocol 2.0)
  std::map<std::tuple<int, int>, std::shared_ptr<dynamixel::GroupSyncWrite>>
  group_write_2_;
  // Group sync read instances for each control table item (protocol 1.0)
  std::map<std::tuple<int, int>, std::shared_ptr<dynamixel::GroupSyncRead>>
  group_read_1_;
  // Group sync read instances for each control table item (protocol 2.0)
  std::map<std::tuple<int, int>, std::shared_ptr<dynamixel::GroupSyncRead>>
  group_read_2_;
  // Error status types not returned by the actuator (resets on enable)
  std::map<int, int> error_status_;

  // Parameters
  std::string port_name_;       // Serial port name
  int baud_rate_;               // Communication baud rate
  bool connected_ = false;      // Flag indicating if the port is open
};

#endif  // DXL_INTERFACE_H
