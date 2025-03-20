#ifndef CLIMB_ROBOT_DRIVER__INTERFACES__DYNAMIXEL_INTERFACE_HPP_
#define CLIMB_ROBOT_DRIVER__INTERFACES__DYNAMIXEL_INTERFACE_HPP_

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "climb_robot_driver/interfaces/hardware_interface.hpp"

/**
 * @brief Hash function for std::pair
 */
struct pair_hash
{
  template<class T1, class T2>
  std::size_t operator()(const std::pair<T1, T2> & pair) const
  {
    std::size_t h1 = std::hash<T1>{}(pair.first);
    std::size_t h2 = std::hash<T2>{}(pair.second);
    return h1 ^ (h2 << 1);
  }
};

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
    const std::vector<int> & ids, const std::vector<std::string> & joints,
    const std::string & model, double ratio) override;
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

  /**
   * @brief Sync read data from the control table of a set of Dynamixels
   * @param[in] ids Actuator IDs
   * @param[in] item Index and length of Dynamixel control table item
   * @param[in] protocol Dynamixel protocol version (1.0 or 2.0)
   * @return Vector of data received (or empty vector if read failed)
   */
  std::vector<double> read(
    const std::vector<int> & ids, std::pair<int, int> item,
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
    const std::vector<int> & ids, std::pair<int, int> item,
    const std::vector<double> & data, float protocol = 2.0);

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

  std::vector<double> readPosition(const std::vector<int> & ids);
  std::vector<double> readVelocity(const std::vector<int> & ids);
  std::vector<double> readEffort(const std::vector<int> & ids);
  std::vector<double> readTemperature(const std::vector<int> & ids);
  std::vector<double> readVoltage(const std::vector<int> & ids);
  std::vector<bool> readEnabled(const std::vector<int> & ids);
  std::vector<uint8_t> readError(const std::vector<int> & ids);

  bool writePosition(
    const std::vector<int> & ids, const std::vector<double> & position);
  bool writeVelocity(
    const std::vector<int> & ids, const std::vector<double> & velocity,
    bool limit = false);
  bool writeEffort(
    const std::vector<int> & ids, const std::vector<double> & effort,
    bool limit = false);

private:
  // Dynamixel serial port handler
  std::shared_ptr<dynamixel::PortHandler> port_handler_;
  // Dynamixel serial packet handler for protocol 1.0
  std::shared_ptr<dynamixel::PacketHandler> packet_handler_1_;
  // Dynamixel serial packet handler for protocol 2.0
  std::shared_ptr<dynamixel::PacketHandler> packet_handler_2_;
  // Group sync write instances for each control table item (protocol 1.0)
  std::unordered_map<std::pair<int, int>,
    std::shared_ptr<dynamixel::GroupSyncWrite>, pair_hash> group_write_1_;
  // Group sync write instances for each control table item (protocol 2.0)
  std::unordered_map<std::pair<int, int>,
    std::shared_ptr<dynamixel::GroupSyncWrite>, pair_hash> group_write_2_;
  // Group sync read instances for each control table item (protocol 1.0)
  std::unordered_map<std::pair<int, int>,
    std::shared_ptr<dynamixel::GroupSyncRead>, pair_hash> group_read_1_;
  // Group sync read instances for each control table item (protocol 2.0)
  std::unordered_map<std::pair<int, int>,
    std::shared_ptr<dynamixel::GroupSyncRead>, pair_hash> group_read_2_;
  // Error status types not returned by the actuator (resets on enable)
  std::unordered_map<int, int> error_status_;

  // Parameters
  std::string port_name_;       // Serial port name
  int baud_rate_;               // Communication baud rate
  bool connected_ = false;      // Flag indicating if the port is open
};

#endif  // CLIMB_ROBOT_DRIVER__INTERFACES__DYNAMIXEL_INTERFACE_HPP_
