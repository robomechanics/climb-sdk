#include "climb_main/hardware/dummy_interface.hpp"

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

bool DummyInterface::enable(std::vector<int> ids)
{
  for (int id : ids) {
    enabled_[id] = true;
  }
  return true;
}

bool DummyInterface::disable(std::vector<int> ids)
{
  for (int id : ids) {
    enabled_[id] = false;
  }
  return true;
}

ActuatorState DummyInterface::readActuatorState()
{
  ActuatorState state;
  for (int id : ids_) {
    state.id.push_back(id);
    for (int i : ids_) {
      state.joint.push_back(joints_by_id_.at(i));
    }
    if (isConnected()) {
      state.enabled.push_back(enabled_[id]);
      state.voltage.push_back(0);
      state.temperature.push_back(0);
      state.error.push_back(0);
    }
  }
  return state;
}

JointState DummyInterface::readJointState()
{
  JointState state;
  double pos, vel, eff;
  for (const auto & item : ids_by_joint_) {
    pos = vel = eff = 0;
    for (int i : item.second) {
      pos += position_[i] / getId(item.first).size();
      vel += velocity_[i] / getId(item.first).size();
      eff += effort_[i];
    }
    state.name.push_back(item.first);
    if (isConnected()) {
      state.position.push_back(pos);
      state.velocity.push_back(vel);
      state.effort.push_back(eff);
    }
  }
  return state;
}

bool DummyInterface::writeJointCommand(JointCommand command)
{
  if (!isConnected() || !validateJointCommand(command)) {
    return false;
  }
  for (unsigned int j = 0; j < command.name.size(); j++) {
    for (int i : getId(command.name[j])) {
      if (command.mode[j] == JointCommand::MODE_POSITION) {
        position_[i] = command.position[j];
        velocity_[i] = command.velocity.size() ? command.velocity[j] : 0;
        effort_[i] = (command.effort.size() ? command.effort[j] : 0) /
          getId(command.name[j]).size();
      } else if (command.mode[j] == JointCommand::MODE_VELOCITY) {
        velocity_[i] = command.velocity[j];
        effort_[i] = (command.effort.size() ? command.effort[j] : 0) /
          getId(command.name[j]).size();
      } else if (command.mode[j] == JointCommand::MODE_EFFORT) {
        effort_[i] = command.effort[j] / getId(command.name[j]).size();
      }
    }
  }
  return true;
}
