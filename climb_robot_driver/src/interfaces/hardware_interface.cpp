#include "climb_robot_driver/interfaces/hardware_interface.hpp"

void HardwareInterface::addActuators(
  const std::vector<int> & ids, const std::vector<std::string> & joints,
  const std::string & model, double ratio)
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

void HardwareInterface::removeActuators(const std::vector<int> & ids)
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

void HardwareInterface::removeJoints(const std::vector<std::string> & joints)
{
  std::vector<int> ids;
  for (const auto & j : joints) {
    for (int id : ids_by_joint_[j]) {
      ids.push_back(id);
    }
  }
  removeActuators(ids);
}

void HardwareInterface::setRatios(const std::vector<int> & ids, double ratio)
{
  for (int id : ids) {
    if (ratios_by_id_.find(id) != ratios_by_id_.end()) {
      ratios_by_id_[id] = ratio;
    }
  }
}
