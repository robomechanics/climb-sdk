#include "climb_main/hardware/dynamixel_interface.hpp"

constexpr double PI = 3.14159;
const std::tuple<int, int> OPERATING_MODE_XM = {11, 1};
const std::tuple<int, int> TORQUE_ENABLE_XM = {64, 1};
const std::tuple<int, int> PRESENT_POSITION_XM = {132, 4};
const std::tuple<int, int> PRESENT_VELOCITY_XM = {128, 4};
const std::tuple<int, int> PRESENT_CURRENT_XM = {126, 2};
const std::tuple<int, int> GOAL_POSITION_XM = {116, 4};
const std::tuple<int, int> PROFILE_VELOCITY_XM = {112, 4};
const std::tuple<int, int> GOAL_CURRENT_XM = {102, 2};
const std::tuple<int, int> PRESENT_TEMPERATURE_XM = {146, 1};
const std::tuple<int, int> PRESENT_INPUT_VOLTAGE_XM = {144, 2};
const std::tuple<int, int> HARDWARE_ERROR_STATUS_XM = {70, 1};

DynamixelInterface::~DynamixelInterface()
{
  disconnect();
}

void DynamixelInterface::addActuators(
  std::vector<int> ids,
  std::vector<std::string> joints, std::string model, double ratio)
{
  HardwareInterface::addActuators(ids, joints, model, ratio);
  for (int id : ids) {
    error_status_[id] = ActuatorState::ERROR_NONE;
  }
}

bool DynamixelInterface::connect()
{
  port_handler_ =
    std::shared_ptr<dynamixel::PortHandler>(
    dynamixel::PortHandler::getPortHandler(
      port_name_.c_str()));
  if (!port_handler_->openPort()) {
    return connected_ = false;
  }
  if (!port_handler_->setBaudRate(baud_rate_)) {
    return connected_ = false;
  }
  packet_handler_1_ = std::shared_ptr<dynamixel::PacketHandler>(
    dynamixel::PacketHandler::getPacketHandler(1.0));
  packet_handler_2_ = std::shared_ptr<dynamixel::PacketHandler>(
    dynamixel::PacketHandler::getPacketHandler(2.0));
  return connected_ = true;
}

void DynamixelInterface::disconnect()
{
  if (isConnected()) {
    port_handler_->closePort();
  }
  connected_ = false;
}

bool DynamixelInterface::isConnected()
{
  return connected_;
}

std::vector<double> DynamixelInterface::read(
  std::vector<int> ids,
  std::tuple<int, int> item, float protocol)
{
  // Abort if not connected or no IDs provided
  if (!isConnected() || ids.empty()) {
    return {};
  }
  // Select protocol version
  auto & group_read = (protocol == 1.0) ? group_read_1_ : group_read_2_;
  auto & handler = (protocol == 1.0) ? packet_handler_1_ : packet_handler_2_;
  auto [index, length] = item;
  // Create or reset group read instance
  if (group_read.find(item) == group_read.end()) {
    group_read[item] = std::shared_ptr<dynamixel::GroupSyncRead>(
      new dynamixel::GroupSyncRead(
        port_handler_.get(), handler.get(), index, length));
  } else {
    group_read[item]->clearParam();
  }
  // Read data
  for (int id : ids) {
    group_read[item]->addParam(id);
  }
  int dxl_comm_result = group_read[item]->txRxPacket();
  // If communication fails, close connection and abort
  if (dxl_comm_result != COMM_SUCCESS) {
    disconnect();
    return {};
  }
  // Return read data (-1 for failed reads)
  std::vector<double> data;
  for (int id : ids) {
    if (group_read[item]->isAvailable(id, index, length)) {
      data.push_back(group_read[item]->getData(id, index, length));
    } else {
      data.push_back(-1);
    }
  }
  return data;
}

bool DynamixelInterface::write(
  std::vector<int> ids,
  std::tuple<int, int> item, std::vector<double> data, float protocol)
{
  // Abort if not connected, no IDs provided, or data size mismatch
  if (!isConnected() || ids.empty()) {
    return false;
  } else if (data.size() != ids.size() && data.size() != 1) {
    for (int id : ids) {
      error_status_[id] = ActuatorState::ERROR_BAD_COMMAND;
    }
    disable(ids);
    return false;
  }
  // Select protocol version
  auto & group_write = (protocol == 1.0) ? group_write_1_ : group_write_2_;
  auto & handler = (protocol == 1.0) ? packet_handler_1_ : packet_handler_2_;
  auto [index, length] = item;
  // Create or reset group write instance
  if (group_write.find(item) == group_write.end()) {
    group_write[item] = std::shared_ptr<dynamixel::GroupSyncWrite>(
      new dynamixel::GroupSyncWrite(
        port_handler_.get(), handler.get(), index, length));
  } else {
    group_write[item]->clearParam();
  }
  // Write data
  for (unsigned int i = 0; i < ids.size(); i++) {
    auto bytes = toBytes((data.size() > 1) ? data[i] : data[0], length);
    group_write[item]->addParam(ids[i], bytes.data());
  }
  int dxl_comm_result = group_write[item]->txPacket();
  // If communication fails, close connection
  if (dxl_comm_result != COMM_SUCCESS) {
    disconnect();
    return false;
  }
  return true;
}

std::vector<uint8_t> DynamixelInterface::toBytes(double value, int length)
{
  std::vector<uint8_t> bytes;
  bytes.push_back(DXL_LOBYTE(DXL_LOWORD(value)));
  if (length > 1) {
    bytes.push_back(DXL_HIBYTE(DXL_LOWORD(value)));
  }
  if (length > 2) {
    bytes.push_back(DXL_LOBYTE(DXL_HIWORD(value)));
  }
  if (length > 3) {
    bytes.push_back(DXL_HIBYTE(DXL_HIWORD(value)));
  }
  return bytes;
}

bool DynamixelInterface::getBit(int value, int bit)
{
  return (value & (1 << bit)) != 0;
}

void DynamixelInterface::declareParameters(const rclcpp::Node::SharedPtr node)
{
  declareParameter(node, "port_name", "/dev/ttyUSB0", "Serial port name");
  declareParameter(node, "baud_rate", 1000000, "Communication baud rate");
}

void DynamixelInterface::setParameter(
  const rclcpp::Parameter & param,
  [[maybe_unused]] rcl_interfaces::msg::SetParametersResult & result)
{
  if (param.get_name() == "port_name") {
    port_name_ = param.as_string();
    if (isConnected()) {
      connect();
    }
  } else if (param.get_name() == "baud_rate") {
    baud_rate_ = param.as_int();
    if (isConnected()) {
      port_handler_->setBaudRate(baud_rate_);
    }
  }
}

bool DynamixelInterface::enable(std::vector<int> ids)
{
  for (int id : ids) {
    error_status_[id] = ActuatorState::ERROR_NONE;
  }
  if (!write(ids, OPERATING_MODE_XM, {5}, 2.0)) {
    return false;
  }
  return write(ids, TORQUE_ENABLE_XM, {1}, 2.0);
}

bool DynamixelInterface::disable(std::vector<int> ids)
{
  return write(ids, TORQUE_ENABLE_XM, {0}, 2.0);
}

std::vector<double> DynamixelInterface::readPosition(std::vector<int> ids)
{
  std::vector<double> position = read(ids, PRESENT_POSITION_XM, 2.0);
  for (unsigned int i = 0; i < position.size(); i++) {
    if (position[i] == -1) {
      position[i] = 0;
    } else {
      // Convert from Dynamixel units (1/4095 revolutions) to radians
      position[i] = position[i] / 4095.0 * 2 * PI / ratios_by_id_[ids[i]] - PI;
    }
  }
  return position;
}

std::vector<double> DynamixelInterface::readVelocity(std::vector<int> ids)
{
  std::vector<double> velocity = read(ids, PRESENT_VELOCITY_XM, 2.0);
  for (unsigned int i = 0; i < velocity.size(); i++) {
    if (velocity[i] == -1) {
      velocity[i] = 0;
    } else {
      if (velocity[i] >= 32768) {
        velocity[i] -= 32768 * 2;
      }
      // Convert from Dynamixel units (0.229 rpm) to rad/s
      velocity[i] = velocity[i] * 0.229 * 2 * PI / 60 / ratios_by_id_[ids[i]];
    }
  }
  return velocity;
}

std::vector<double> DynamixelInterface::readEffort(std::vector<int> ids)
{
  std::vector<double> effort = read(ids, PRESENT_CURRENT_XM, 2.0);
  for (unsigned int i = 0; i < effort.size(); i++) {
    if (effort[i] == -1) {
      effort[i] = 0;
    } else {
      if (effort[i] >= 32768) {
        effort[i] -= 32768 * 2;
      }
      // Convert from Dynamixel units (2.69 mA) to Nm
      effort[i] = effort[i] * 2.69 * 4.1 / 2.3 * ratios_by_id_[ids[i]];
    }
  }
  return effort;
}

std::vector<double> DynamixelInterface::readTemperature(std::vector<int> ids)
{
  std::vector<double> temperature = read(ids, PRESENT_TEMPERATURE_XM, 2.0);
  for (unsigned int i = 0; i < temperature.size(); i++) {
    if (temperature[i] == -1) {
      temperature[i] = 0;
    }
    // Dynamixel units are deg C
  }
  return temperature;
}

std::vector<double> DynamixelInterface::readVoltage(std::vector<int> ids)
{
  std::vector<double> voltage = read(ids, PRESENT_INPUT_VOLTAGE_XM, 2.0);
  for (unsigned int i = 0; i < voltage.size(); i++) {
    if (voltage[i] == -1) {
      voltage[i] = 0;
    }
    // Convert from Dynamixel units (0.1 V) to V
    voltage[i] = voltage[i] / 10.0;
  }
  return voltage;
}

std::vector<uint8_t> DynamixelInterface::readError(std::vector<int> ids)
{
  std::vector<double> error = read(ids, HARDWARE_ERROR_STATUS_XM, 2.0);
  std::vector<uint8_t> code(error.size());
  for (unsigned int i = 0; i < error.size(); i++) {
    if (error[i] == -1) {
      code[i] = ActuatorState::ERROR_COMMUNICATION;
    } else if (error[i] == 0) {
      code[i] = error_status_[ids[i]];
    } else if (getBit(error[i], 0)) {
      code[i] = ActuatorState::ERROR_VOLTAGE;
    } else if (getBit(error[i], 2)) {
      code[i] = ActuatorState::ERROR_TEMPERATURE;
    } else if (getBit(error[i], 3)) {
      code[i] = ActuatorState::ERROR_POSITION_SENSOR;
    } else if (getBit(error[i], 4)) {
      code[i] = ActuatorState::ERROR_ELECTRICAL;
    } else if (getBit(error[i], 5)) {
      code[i] = ActuatorState::ERROR_OVERLOAD;
    } else {
      code[i] = ActuatorState::ERROR_UNDEFINED;
    }
  }
  return code;
}

bool DynamixelInterface::writePosition(
  std::vector<int> ids, std::vector<double> position)
{
  for (unsigned int i = 0; i < position.size(); i++) {
    // Convert from radians to Dynamixel units (1/4095 revolutions)
    position[i] =
      (position[i] + PI) / (2 * PI) * 4095.0 * ratios_by_id_[ids[i]];
  }
  return write(ids, GOAL_POSITION_XM, position, 2.0);
}

bool DynamixelInterface::writeVelocity(
  std::vector<int> ids, std::vector<double> velocity, bool limit)
{
  for (unsigned int i = 0; i < velocity.size(); i++) {
    // Convert from rad/s to Dynamixel units (0.229 rpm)
    velocity[i] = velocity[i] * 60 / (2 * PI) / 0.229 * ratios_by_id_[ids[i]];
  }
  // TODO: velocity mode not yet supported
  if (!limit) {
    for (int id : ids) {
      error_status_[id] = ActuatorState::ERROR_BAD_MODE;
    }
    disable(ids);
    return false;
  }
  return write(ids, PROFILE_VELOCITY_XM, velocity, 2.0);
}

bool DynamixelInterface::writeEffort(
  std::vector<int> ids, std::vector<double> effort, bool limit)
{
  for (unsigned int i = 0; i < effort.size(); i++) {
    // Convert from Nm to Dynamixel units (2.69 mA)
    effort[i] = effort[i] * 2.3 / 4.1 / 2.69 / ratios_by_id_[ids[i]];
  }
  // TODO: effort mode not yet supported
  if (!limit) {
    for (int id : ids) {
      error_status_[id] = ActuatorState::ERROR_BAD_MODE;
    }
    disable(ids);
    return false;
  }
  return write(ids, GOAL_CURRENT_XM, effort, 2.0);
}

ActuatorState DynamixelInterface::readActuatorState()
{
  ActuatorState state;
  state.id = ids_;
  for (int i : ids_) {
    state.joint.push_back(joints_by_id_.at(i));
  }
  state.temperature = readTemperature(ids_);
  state.voltage = readVoltage(ids_);
  state.error = readError(ids_);
  return state;
}

JointState DynamixelInterface::readJointState()
{
  JointState state;
  for (int i : ids_) {
    state.name.push_back(joints_by_id_.at(i));
  }
  state.position = readPosition(ids_);
  state.velocity = readVelocity(ids_);
  state.effort = readEffort(ids_);
  return state;
}

void DynamixelInterface::writeJointState(JointCommand command)
{
  // Group actuators by operating mode
  std::vector<int> ids_pos, ids_vel, ids_eff;
  std::vector<double> p_pos;
  std::vector<double> v_pos, v_vel;
  std::vector<double> e_pos, e_vel, e_eff;
  for (unsigned int j = 0; j < command.name.size(); j++) {
    if (command.mode[j] == JointCommand::MODE_POSITION) {
      for (int i : ids_by_joint_[command.name[j]]) {
        ids_pos.push_back(i);
        p_pos.push_back(command.position[i]);
        v_pos.push_back(command.velocity[i]);
        e_pos.push_back(
          command.effort[i] / ids_by_joint_[command.name[j]].size());
      }
    } else if (command.mode[j] == JointCommand::MODE_VELOCITY) {
      for (unsigned int i : ids_by_joint_[command.name[j]]) {
        ids_pos.push_back(i);
        v_pos.push_back(command.velocity[i]);
        e_pos.push_back(
          command.effort[i] / ids_by_joint_[command.name[j]].size());
      }
    } else if (command.mode[j] == JointCommand::MODE_EFFORT) {
      for (unsigned int i : ids_by_joint_[command.name[j]]) {
        ids_pos.push_back(i);
        e_pos.push_back(
          command.effort[i] / ids_by_joint_[command.name[j]].size());
      }
    }
  }

  // Write to actuators in position mode
  writePosition(ids_pos, p_pos);
  writeVelocity(ids_pos, v_pos, true);
  writeEffort(ids_pos, e_pos, true);

  // Write to actuators in velocity mode
  writeVelocity(ids_vel, v_vel);
  writeEffort(ids_vel, e_vel, true);

  // Write to actuators in effort mode
  writeEffort(ids_eff, e_eff);
}
