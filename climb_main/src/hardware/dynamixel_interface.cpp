#include "climb_main/hardware/dynamixel_interface.hpp"

DynamixelInterface::~DynamixelInterface()
{
  disconnect();
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
  packet_handler_ = std::shared_ptr<dynamixel::PacketHandler>(
    dynamixel::PacketHandler::getPacketHandler(2.0));
  group_write_ = std::shared_ptr<dynamixel::GroupSyncWrite>(
    new dynamixel::GroupSyncWrite(port_handler_.get(), packet_handler_.get(), 64, 1));
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

bool DynamixelInterface::enable(std::vector<int> ids)
{
  uint8_t param[1] = {1};
  bool dxl_add_result = group_write_->addParam(1, param);
  int dxl_comm_result = group_write_->txPacket();

  return dxl_comm_result == COMM_SUCCESS;
}

void DynamixelInterface::disable(std::vector<int> ids)
{
  // TODO: disable motors
}

void DynamixelInterface::declareParameters(const rclcpp::Node::SharedPtr node)
{
  declareParameter(node, "port_name", "/dev/ttyUSB0", "Serial port name");
  declareParameter(node, "baud_rate", 1000000, "Communication baud rate");
}

void DynamixelInterface::setParameter(
  const rclcpp::Parameter & param,
  rcl_interfaces::msg::SetParametersResult & result)
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

std::vector<double> DynamixelInterface::readPosition(std::vector<int> ids)
{
  // Read positions
}

std::vector<double> DynamixelInterface::readVelocity(std::vector<int> ids)
{
  // Read velocities
}

std::vector<double> DynamixelInterface::readEffort(std::vector<int> ids)
{
  // Read efforts
}

std::vector<double> DynamixelInterface::readTemperature(std::vector<int> ids)
{
  // Read temperatures
}

std::vector<double> DynamixelInterface::readVoltage(std::vector<int> ids)
{
  // Read voltages
}

std::vector<uint8_t> DynamixelInterface::readError(std::vector<int> ids)
{
  // Read errors
}

void DynamixelInterface::writePosition(
  std::vector<int> ids, std::vector<double> positions)
{
  // Write positions
}

void DynamixelInterface::writeVelocity(
  std::vector<int> ids, std::vector<double> velocities, bool limit)
{
  // Write velocities
}

void DynamixelInterface::writeEffort(
  std::vector<int> ids, std::vector<double> efforts, bool limit)
{
  // Write efforts
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
    if (command.mode[j] == JointCommand::POSITION_MODE) {
      for (int i : ids_by_joint_[command.name[j]]) {
        ids_pos.push_back(i);
        p_pos.push_back(command.position[i]);
        v_pos.push_back(command.velocity[i]);
        e_pos.push_back(
          command.effort[i] / ids_by_joint_[command.name[j]].size());
      }
    } else if (command.mode[j] == JointCommand::VELOCITY_MODE) {
      for (unsigned int i : ids_by_joint_[command.name[j]]) {
        ids_pos.push_back(i);
        v_pos.push_back(command.velocity[i]);
        e_pos.push_back(
          command.effort[i] / ids_by_joint_[command.name[j]].size());
      }
    } else if (command.mode[j] == JointCommand::EFFORT_MODE) {
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
