#ifndef CLIMB_FOOTSTEP_PLANNER__PLANNERS__DUMMY_PLANNER_HPP_
#define CLIMB_FOOTSTEP_PLANNER__PLANNERS__DUMMY_PLANNER_HPP_

#include <memory>
#include "climb_footstep_planner/planners/planner.hpp"

class DummyPlanner : public Planner
{
public:
  explicit DummyPlanner(std::shared_ptr<KinematicsInterface> robot)
  : Planner(robot) {}

  Plan plan(const Step & start, const Eigen::Isometry3d & goal) override
  {
    Step step = start;
    step.pose = goal;
    step.cost = (start.pose.translation() - goal.translation()).norm();
    if (step.cost > 0.2) {
      step.cost = INFINITY;
    }
    return {step};
  }

  void declareParameters() override {}

  void setParameter(
    const Parameter &, SetParametersResult &) override {}
};

#endif  // CLIMB_FOOTSTEP_PLANNER__PLANNERS__DUMMY_PLANNER_HPP_
