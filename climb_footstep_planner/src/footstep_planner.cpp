#include "climb_footstep_planner/footstep_planner.hpp"
#include <climb_kinematics/kinematics_interfaces/kdl_interface.hpp>

FootstepPlanner::FootstepPlanner()
{
  robot_ = std::make_unique<KdlInterface>();
}

bool FootstepPlanner::initialize(const std::string & robot_description)
{
  std::string err;
  return robot_->loadRobotDescription(robot_description, err);
}

std::vector<FootstepPlanner::Stance> FootstepPlanner::plan(
  Stance start, Eigen::Isometry3d goal)
{
  // TODO: implement planner
  goal_ = goal;
  plan_.clear();
  plan_.push_back(start);
  return plan_;
}

std::vector<FootstepPlanner::Stance> FootstepPlanner::replan(Stance start)
{
  return plan(start, goal_);
}
