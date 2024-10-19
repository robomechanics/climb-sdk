#include "grasp_system.hpp"

#include <gz/plugin/Register.hh>

IGNITION_ADD_PLUGIN(
  grasp_system::GraspSystem,
  gz::sim::System,
  grasp_system::GraspSystem::ISystemPreUpdate,
  grasp_system::GraspSystem::ISystemPostUpdate)

using namespace grasp_system;

GraspSystem::GraspSystem()
{
}

GraspSystem::~GraspSystem()
{
}

void GraspSystem::PreUpdate(
  const gz::sim::UpdateInfo & _info,
  gz::sim::EntityComponentManager & _ecm)
{
  
}

void GraspSystem::PostUpdate(
  const gz::sim::UpdateInfo & _info,
  const gz::sim::EntityComponentManager & _ecm)
{
  
}
