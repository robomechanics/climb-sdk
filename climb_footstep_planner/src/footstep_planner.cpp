#include "climb_footstep_planner/footstep_planner.hpp"
#include <climb_kinematics/kinematics_interfaces/kdl_interface.hpp>

FootstepPlanner::FootstepPlanner()
{
  robot_ = std::make_unique<KdlInterface>();
}

bool FootstepPlanner::initialize(const std::string & robot_description, std::string & message)
{
  return robot_->loadRobotDescription(robot_description, message);
}

bool FootstepPlanner::isInitialized(std::string & message)
{
  if (!robot_->isInitialized()) {
    message = "Waiting for robot description...";
    return false;
  }
  if (!map_cloud_.size()) {
    message = "Waiting for point cloud...";
    return false;
  }
  return true;
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

void FootstepPlanner::declareParameters()
{
  for (const auto & param : robot_->getParameters()) {
    parameters_.push_back(param);
  }
}

void FootstepPlanner::setParameter(
  const Parameter & param, SetParametersResult & result)
{
  robot_->setParameter(param, result);
}
