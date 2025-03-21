#include "climb_robot_driver/interfaces/dummy_interface.hpp"

void DummyInterface::addActuators(
  const std::vector<int> & ids, const std::vector<std::string> & joints,
  const std::string & model, double ratio)
{
  HardwareInterface::addActuators(ids, joints, model, ratio);
  for (const auto id : ids) {
    enabled_[id] = false;
    position_[id] = 0;
    velocity_[id] = 0;
    effort_[id] = 0;
  }
}

void DummyInterface::removeActuators(const std::vector<int> & ids)
{
  HardwareInterface::removeActuators(ids);
  for (const auto id : ids) {
    enabled_.erase(id);
    position_.erase(id);
    velocity_.erase(id);
    effort_.erase(id);
  }
}

bool DummyInterface::connect()
{
  connected_ = true;
  return true;
}

void DummyInterface::disconnect()
{
  connected_ = false;
}

bool DummyInterface::isConnected()
{
  return connected_;
}

bool DummyInterface::enable(const std::vector<int> & ids)
{
  for (const auto id : ids) {
    enabled_[id] = true;
  }
  return true;
}

bool DummyInterface::disable(const std::vector<int> & ids)
{
  for (const auto id : ids) {
    enabled_[id] = false;
  }
  return true;
}

ActuatorState DummyInterface::readActuatorState()
{
  ActuatorState state;
  for (const auto id : ids_) {
    state.id.push_back(id);
    state.joint.push_back(getJoint(id));
    if (isConnected()) {
      state.enabled.push_back(enabled_[id]);
      state.voltage.push_back(0);
      state.temperature.push_back(0);
      state.error.push_back(0);
    }
  }
  state.header.frame_id = "dummy";
  return state;
}

JointState DummyInterface::readJointState()
{
  JointState state;
  std::unordered_map<std::string, double> pos, vel, eff;
  for (int id : ids_) {
    auto joint = getJoint(id);
    pos[joint] += position_[id] / getId(joint).size();
    vel[joint] += velocity_[id] / getId(joint).size();
    eff[joint] += effort_[id];
    if (std::find(state.name.begin(), state.name.end(), joint) ==
      state.name.end())
    {
      state.name.push_back(joint);
    }
  }
  if (isConnected()) {
    for (const auto & joint : state.name) {
      state.position.push_back(pos[joint]);
      state.velocity.push_back(vel[joint]);
      state.effort.push_back(eff[joint]);
    }
  }
  state.header.frame_id = "dummy";
  return state;
}

bool DummyInterface::writeJointCommand(const JointCommand & command)
{
  if (!isConnected()) {
    return false;
  }
  for (size_t j = 0; j < command.name.size(); j++) {
    size_t num_actuators = getId(command.name[j]).size();
    for (const auto id : getId(command.name[j])) {
      if (j >= command.mode.size() ||
        command.mode[j] == JointCommand::MODE_POSITION)
      {
        if (j < command.position.size()) {
          position_[id] = command.position[j];
        }
        if (j < command.velocity.size()) {
          velocity_[id] = command.velocity[j];
        }
        if (j < command.effort.size()) {
          effort_[id] = command.effort[j] / num_actuators;
        }
      } else if (command.mode[j] == JointCommand::MODE_VELOCITY) {
        velocity_[id] = command.velocity[j];
        effort_[id] = (command.effort.size() ? command.effort[j] : 0) /
          getId(command.name[j]).size();
      } else if (command.mode[j] == JointCommand::MODE_EFFORT) {
        effort_[id] = command.effort[j] / getId(command.name[j]).size();
      }
    }
  }
  return true;
}

void DummyInterface::declareParameters()
{
  declareParameter(
    "initial_position", std::vector<double>(),
    "Initial joint angles in rad or m");
  declareParameter(
    "initial_velocity", std::vector<double>(),
    "Initial joint velocities in rad/s or m/s");
  declareParameter(
    "initial_effort", std::vector<double>(),
    "Initial joint efforts in Nm or N");
}

void DummyInterface::setParameter(
  const Parameter & param, [[maybe_unused]] SetParametersResult & result)
{
  if (param.get_name() == "initial_position") {
    std::vector<double> positions = param.as_double_array();
    for (size_t i = 0; i < positions.size() && i < ids_.size(); i++) {
      position_[ids_[i]] = positions[i];
    }
  } else if (param.get_name() == "initial_velocity") {
    std::vector<double> velocities = param.as_double_array();
    for (size_t i = 0; i < velocities.size() && i < ids_.size(); i++) {
      velocity_[ids_[i]] = velocities[i];
    }
  } else if (param.get_name() == "initial_effort") {
    std::vector<double> efforts = param.as_double_array();
    for (size_t i = 0; i < efforts.size() && i < ids_.size(); i++) {
      effort_[ids_[i]] = efforts[i];
    }
  }
}
