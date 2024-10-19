#ifndef GRASP_SYSTEM_HPP
#define GRASP_SYSTEM_HPP

#include <gz/sim/System.hh>

namespace grasp_system
{
class GraspSystem :
  public gz::sim::System,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate
{
public:
  GraspSystem();

  ~GraspSystem() override;

  void PreUpdate(
    const gz::sim::UpdateInfo & _info,
    gz::sim::EntityComponentManager & _ecm) override;

  void PostUpdate(
    const gz::sim::UpdateInfo & _info,
    const gz::sim::EntityComponentManager & _ecm) override;
};
}

#endif  // GRASP_SYSTEM_HPP
