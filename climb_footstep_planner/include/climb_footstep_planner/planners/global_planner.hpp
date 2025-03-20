#ifndef CLIMB_FOOTSTEP_PLANNER__PLANNERS__GLOBAL_PLANNER_HPP_
#define CLIMB_FOOTSTEP_PLANNER__PLANNERS__GLOBAL_PLANNER_HPP_

#include <memory>
#include <string>
#include <climb_footstep_planner/planners/planner.hpp>

class GlobalPlanner : public Planner
{
public:
  GlobalPlanner(
    std::shared_ptr<KinematicsInterface> robot,
    std::unique_ptr<Planner> local_planner);

  bool isInitialized(std::string & message) override;

  void update(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud,
    const Eigen::Isometry3d & viewpoint,
    const Eigen::Vector3d & gravity) override;

  Plan plan(const Step & start, const Eigen::Isometry3d & goal) override;

  pcl::PointCloud<pcl::PointXYZI>::Ptr getCostmap() override;

  void declareParameters() override;

  void setParameter(
    const Parameter & param, SetParametersResult & result) override;

  using Parameterized::setParameter;

private:
  std::unique_ptr<Planner> local_planner_;
  bool debug_;
  std::string algorithm_;
  double runtime_;
  double search_radius_;
  double global_incline_radius_;
  double global_incline_cost_;
  bool vertices_only_;
};

#endif  // CLIMB_FOOTSTEP_PLANNER__PLANNERS__GLOBAL_PLANNER_HPP_
