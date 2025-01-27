#include "climb_footstep_planner/footstep_planner.hpp"

#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>

#include <climb_kinematics/interfaces/kdl_interface.hpp>
#include <climb_util/geometry_utils.hpp>
#include <climb_util/ros_utils.hpp>

using geometry_utils::Polytope;

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

void FootstepPlanner::processCloud()
{
  // Compute normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(map_);
  ne.setSearchMethod(kdtree_);
  ne.setRadiusSearch(0.1);
  Eigen::Vector3d viewpoint = viewpoint_.translation();
  ne.setViewPoint(viewpoint(0), viewpoint(1), viewpoint(2));
  ne.compute(*normals_);
  n_ = normals_->getMatrixXfMap(3, 8, 0);

  // Compute curvature
  pcl::PrincipalCurvaturesEstimation<
    pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> pc;
  pc.setInputCloud(map_);
  pc.setInputNormals(normals_);
  pc.setSearchMethod(kdtree_);
  pc.setRadiusSearch(0.1);
  pc.compute(*curvatures_);
  t_ = curvatures_->getMatrixXfMap().block(
    0, 0, 3, curvatures_->getMatrixXfMap().cols());
  k_ = curvatures_->getMatrixXfMap().block(
    3, 0, 2, curvatures_->getMatrixXfMap().cols());

  // Compute cost
  p_ = map_->getMatrixXfMap(3, 4, 0);
  c_ = gravity_.cast<float>().transpose() * n_;   // Incline-based cost
  // c_ = k_.colwise().sum();                        // Curvature-based cost
  cost_->resize(map_->size());
  cost_->getMatrixXfMap(4, 8, 0) = map_->getMatrixXfMap(4, 4, 0);
  cost_->getMatrixXfMap(1, 8, 4) = c_;
}

std::vector<FootstepPlanner::Stance> FootstepPlanner::plan(
  const Stance & start, const Eigen::Isometry3d & goal)
{
  processCloud();
  goal_ = goal;
  plan_ = {start};
  auto local_plan = localPlan(start, goal);
  plan_.insert(plan_.end(), local_plan.begin(), local_plan.end());
  return plan_;
}

std::vector<FootstepPlanner::Stance> FootstepPlanner::replan(
  const Stance & start)
{
  return plan(start, goal_);
}

std::vector<FootstepPlanner::Stance> FootstepPlanner::localPlan(
  const Stance & start, const Eigen::Isometry3d & goal)
{
  std::vector<Stance> local_plan;
  auto stance = start;
  Eigen::Vector3d direction = goal.translation() - start.pose.translation();
  double dist = 0;
  double goal_dist = direction.norm();
  direction.normalize();
  while (dist < goal_dist) {
    stance = step(stance, goal);
    if (stance.swing_foot.empty()) {break;}
    local_plan.push_back(stance);
    dist = direction.dot(stance.pose.translation() - start.pose.translation());
  }
  if (stance.swing_foot.empty()) {
    std::cout << "Plan failed" << std::endl;
  } else {
    std::cout << "Plan succeeded" << std::endl;
  }
  return local_plan;
}

FootstepPlanner::Stance FootstepPlanner::step(
  const Stance & start, const Eigen::Isometry3d & goal)
{
  Stance stance = start;
  stance.swing_foot = "";
  Eigen::Vector3d p0_body = start.pose.translation();
  Eigen::Vector3d direction = goal.translation() - p0_body;
  direction.normalize();
  // TODO: Temporarily hardcoded workspaces for each end effector
  auto W1 = Polytope::createBox(
    workspace_min_limits_, workspace_max_limits_);
  std::unordered_map<std::string, Polytope> workspaces = {
    {"gripper_1", W1},
    {"gripper_2", W1.scaled(Eigen::Vector3d{1.0, -1.0, 1.0})},
    {"gripper_3", W1.scaled(Eigen::Vector3d{-1.0, 1.0, 1.0})},
    {"gripper_4", W1.scaled(Eigen::Vector3d{-1.0, -1.0, 1.0})}};

  // Select swing foot closest to rear edge of workspace (in body frame)
  Eigen::Isometry3d map_to_body = start.pose.inverse();
  std::string swing;
  double min_margin = INFINITY;
  int n = 0;
  for (const auto & [foot, pose] : start.footholds) {
    if (robot_->getContactType(foot) == ContactType::TAIL) {continue;}
    ++n;
    double margin = workspaces.at(foot).distance(
      map_to_body * pose.translation(), map_to_body * -direction);
    if (margin < min_margin) {
      swing = foot;
      min_margin = margin;
    }
  }

  // Compute swing foot workspace (in body frame)
  Eigen::Vector3d p_mean = Eigen::Vector3d::Zero();
  auto W_swing = workspaces.at(swing);
  auto W_body = Polytope::createBox(
    Eigen::Vector3d::Constant(-INFINITY),
    Eigen::Vector3d::Constant(INFINITY));
  for (const auto & [foot, W_stance] : workspaces) {
    if (robot_->getContactType(foot) == ContactType::TAIL) {continue;}
    if (foot != swing) {
      Eigen::VectorXd p_stance =
        map_to_body * start.footholds.at(foot).translation();
      p_mean += p_stance / (n - 1);
      W_body.intersect(p_stance - W_stance);
    }
  }
  auto W_lim = W_body + W_swing;
  auto W = W_swing * n / (n - 1) + p_mean;
  W.intersect(W_lim);

  // Convert workspace from body to map frame
  W_swing = start.pose * W_swing;
  W_body = start.pose * W_body;
  W_lim = start.pose * W_lim;
  W = start.pose * W;
  p_mean = start.pose * p_mean;

  // Sample footholds in workspace
  double radius = (workspace_max_limits_ - workspace_min_limits_).norm();
  Eigen::VectorXd p0_swing = start.footholds.at(swing).translation();
  Eigen::Vector3f p_search = (p0_swing + direction * radius).cast<float>();
  pcl::PointXYZ p_search_pcl{p_search.x(), p_search.y(), p_search.z()};
  std::vector<int> indices;
  std::vector<float> distances;
  kdtree_->radiusSearch(p_search_pcl, radius, indices, distances);
  Eigen::MatrixXd p_workspace(3, indices.size());
  Eigen::Vector3d p_i;
  int j = 0;
  c_.setConstant(0);
  for (auto i : indices) {
    p_i = p_.col(i).cast<double>();
    if (W_lim.contains(p_i)) {c_(i) = 3000;}
    if (W.contains(p_i)) {
      p_workspace.col(j++) = p_i;
      c_(i) = 2000;
    }
    if (W_swing.contains(p_i)) {c_(i) = 1000;}
  }
  p_workspace.conservativeResize(3, j);
  cost_->getMatrixXfMap(1, 8, 4) = c_;
  if (!j) {
    std::cout << "Initial: " << W_swing.b.transpose() << std::endl;
    std::cout << "Kinematic: " << W_lim.b.transpose() << std::endl;
    std::cout << "Centered: " << W.b.transpose() << std::endl;
    return stance;
  }

  // Select optimal foothold from workspace
  Eigen::VectorXd distance =
    W_swing.distanceAll(p_workspace, -direction, workspace_angular_tol_);
  distance.array() -= W_swing.distance(p0_swing, -direction);
  Eigen::Index index;
  if (distance.maxCoeff(&index) < min_step_length_) {return stance;}
  Eigen::Vector3d p_swing = p_workspace.col(index);

  // Compute body pose
  Eigen::Vector3d p_body = p_mean * (n - 1) / n + p_swing / n;
  p_body = W_body.clip(p_body, p_body - p0_body);

  stance.pose.translation() = p_body;
  stance.footholds.at(swing).translation() = p_swing;
  stance.swing_foot = swing;
  return stance;
}

void FootstepPlanner::declareParameters()
{
  for (const auto & param : robot_->getParameters()) {
    parameters_.push_back(param);
  }
  declareParameter(
    "min_step_length", 0.02,
    "Minimum body displacment in direction of goal per step", 0.0);
  declareParameter(
    "workspace_angular_tol", 0.0,
    "Angular tolerance for distance from workspace edge in radians");
  declareParameter(
    "workspace_min_limits", std::vector<double>{0.0, 0.0, 0.0},
    "Lower bounds of front left end effector workspace in body frame");
  declareParameter(
    "workspace_max_limits", std::vector<double>{0.0, 0.0, 0.0},
    "Upper bounds of front left end effector workspace in body frame");
}

void FootstepPlanner::setParameter(
  const Parameter & param, SetParametersResult & result)
{
  if (param.get_name() == "min_step_length") {
    min_step_length_ = param.as_double();
  } else if (param.get_name() == "workspace_angular_tol") {
    workspace_angular_tol_ = param.as_double();
  } else if (param.get_name() == "workspace_min_limits") {
    if (param.as_double_array().size() == 3) {
      workspace_min_limits_ = RosUtils::vectorToEigen(param.as_double_array());
    } else {
      result.successful = false;
      result.reason = "Parameter must be length 3";
    }
  } else if (param.get_name() == "workspace_max_limits") {
    if (param.as_double_array().size() == 3) {
      workspace_max_limits_ = RosUtils::vectorToEigen(param.as_double_array());
    } else {
      result.successful = false;
      result.reason = "Parameter must be length 3";
    }
  }
  robot_->setParameter(param, result);
}
