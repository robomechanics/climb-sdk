#include "climb_main/hardware/hardware_interface.hpp"

void HardwareInterface::addActuators(
  std::vector<int> ids, std::vector<std::string> joints,
  std::string model, double ratio)
{
  for (size_t i = 0; i < ids.size(); i++) {
    if (ids[i] == -1) {
      continue;   // Passive joint
    }
    ids_.push_back(ids[i]);
    models_by_id_[ids[i]] = model;
    joints_by_id_[ids[i]] = joints[i];
    ratios_by_id_[ids[i]] = ratio;
    ids_by_joint_[joints[i]].push_back(ids[i]);
  }
}

void HardwareInterface::removeActuators(std::vector<int> ids)
{
  disable(ids);
  for (int id : ids) {
    auto & j = ids_by_joint_[joints_by_id_[id]];
    j.erase(std::remove(j.begin(), j.end(), id), j.end());
    ids_.erase(std::remove(ids_.begin(), ids_.end(), id), ids_.end());
    models_by_id_.erase(id);
    joints_by_id_.erase(id);
    ratios_by_id_.erase(id);
  }
}

void HardwareInterface::removeJoints(std::vector<std::string> joints)
{
  std::vector<int> ids;
  for (const auto & j : joints) {
    for (int id : ids_by_joint_[j]) {
      ids.push_back(id);
    }
  }
  removeActuators(ids);
}

void HardwareInterface::setRatios(std::vector<int> ids, double ratio)
{
  for (int id : ids) {
    if (ratios_by_id_.find(id) != ratios_by_id_.end()) {
      ratios_by_id_[id] = ratio;
    }
  }
}

bool HardwareInterface::validateJointCommand(const JointCommand & command)
{
  size_t name_size = command.name.size();
  size_t mode_size = command.mode.size();
  size_t position_size = command.position.size();
  size_t velocity_size = command.velocity.size();
  size_t effort_size = command.effort.size();
  bool has_position_mode = std::find(
    command.mode.begin(), command.mode.end(),
    JointCommand::MODE_POSITION) != command.mode.end();
  bool has_velocity_mode = std::find(
    command.mode.begin(), command.mode.end(),
    JointCommand::MODE_VELOCITY) != command.mode.end();
  bool has_effort_mode = std::find(
    command.mode.begin(), command.mode.end(),
    JointCommand::MODE_EFFORT) != command.mode.end();
  if (!name_size ||
    (mode_size && mode_size != name_size) ||
    (has_position_mode && name_size != position_size) ||
    (has_velocity_mode && name_size != velocity_size) ||
    (has_effort_mode && name_size != effort_size) ||
    (has_position_mode && velocity_size && velocity_size != name_size) ||
    (has_position_mode && effort_size && effort_size != name_size) ||
    (has_velocity_mode && effort_size && effort_size != name_size))
  {
    return false;
  }
  for (auto j : command.name) {
    if (getId(j).empty()) {
      return false;
    }
  }
  return true;
}
