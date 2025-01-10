#include "climb_footstep_planner/footstep_planner.hpp"
#include <climb_kinematics/kinematics_interfaces/kdl_interface.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>

using namespace std::chrono_literals;

FootstepPlanner::FootstepPlanner()
{
  robot_ = std::make_unique<KdlInterface>();
  map_ = std::make_shared<PointCloud>();
  normals_ = std::make_shared<NormalCloud>();
  curvatures_ = std::make_shared<CurvatureCloud>();
  cost_ = std::make_shared<CostCloud>();
  kdtree_ = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>(false);
  gravity_ = Eigen::Vector3d(0.0, 0.0, 1.0);
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
  if (!map_->size()) {
    message = "Waiting for point cloud...";
    return false;
  }
  return true;
}

void FootstepPlanner::update(
  const PointCloud::Ptr & map_cloud, const Eigen::Isometry3d & viewpoint)
{
  map_ = map_cloud;
  viewpoint_ = viewpoint;
  normals_->clear();
  cost_->clear();
  kdtree_->setInputCloud(map_);
}

void FootstepPlanner::updateGravity(const Eigen::Quaterniond & orientation)
{
  gravity_ = orientation * Eigen::Vector3d(0.0, 0.0, -1.0);
}

std::vector<FootstepPlanner::Stance> FootstepPlanner::plan(
  Stance start, Eigen::Isometry3d goal)
{
  // TODO: implement planner
  goal_ = goal;
  plan_.clear();
  plan_.push_back(start);

  // Compute normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(map_);
  ne.setSearchMethod(kdtree_);
  ne.setRadiusSearch(0.1);
  Eigen::Vector3d viewpoint = viewpoint_.translation();
  ne.setViewPoint(viewpoint[0], viewpoint[1], viewpoint[2]);
  ne.compute(*normals_);
  n_ = normals_->getMatrixXfMap(3, 8, 0);

  // Compute curvature
  pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> pc;
  pc.setInputCloud(map_);
  pc.setInputNormals(normals_);
  pc.setSearchMethod(kdtree_);
  pc.setRadiusSearch(0.1);
  pc.compute(*curvatures_);

  t_ = curvatures_->getMatrixXfMap().block(0, 0, 3, curvatures_->getMatrixXfMap().cols());
  k_ = curvatures_->getMatrixXfMap().block(3, 0, 2, curvatures_->getMatrixXfMap().cols());

  // Compute cost
  p_ = map_->getMatrixXfMap(3, 4, 0);
  c_ = gravity_.cast<float>().transpose() * n_;   // Incline-based cost
  // c_ = k_.colwise().sum();                        // Curvature-based cost
  cost_->resize(map_->size());
  cost_->getMatrixXfMap(4, 8, 0) = map_->getMatrixXfMap(4, 4, 0);
  cost_->getMatrixXfMap(1, 8, 4) = c_;

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
