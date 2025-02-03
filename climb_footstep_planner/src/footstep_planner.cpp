#include "climb_footstep_planner/footstep_planner.hpp"

#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>

#include <climb_kinematics/interfaces/kdl_interface.hpp>
#include <climb_util/eigen_utils.hpp>
#include <climb_util/ros_utils.hpp>
#include "climb_footstep_planner/terrain_generator.hpp"

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
  n_ = normals_->getMatrixXfMap(3, 8, 0).cast<double>();

  // Compute curvature
  pcl::PrincipalCurvaturesEstimation<
    pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> pc;
  pc.setInputCloud(map_);
  pc.setInputNormals(normals_);
  pc.setSearchMethod(kdtree_);
  pc.setRadiusSearch(0.1);
  pc.compute(*curvatures_);
  t_ = curvatures_->getMatrixXfMap().block(
    0, 0, 3, curvatures_->getMatrixXfMap().cols()).cast<double>();
  k_ = curvatures_->getMatrixXfMap().block(
    3, 0, 2, curvatures_->getMatrixXfMap().cols()).cast<double>();

  // Compute cost
  p_ = map_->getMatrixXfMap(3, 4, 0).cast<double>();
  c_ = gravity_.transpose() * n_;       // Incline-based cost
  // c_ = k_.colwise().sum();                        // Curvature-based cost
  cost_->resize(map_->size());
  cost_->getMatrixXfMap(4, 8, 0) = map_->getMatrixXfMap(4, 4, 0);
  cost_->getMatrixXfMap(1, 8, 4).row(0) = c_.cast<float>();
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
    if (!local_plan.empty() &&
      stance.swing_foot == local_plan.back().swing_foot)
    {
      local_plan.back() = stance;
    } else {
      local_plan.push_back(stance);
    }
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
  Stance result = start;
  result.swing_foot = "";
  Eigen::Vector3d p0_body = start.pose.translation();
  Eigen::Vector3d direction = goal.translation() - p0_body;
  direction.normalize();
  // TODO: Temporarily hardcoded workspaces for each end effector
  auto W_1 = Polytope::createBox(
    workspace_min_limits_, workspace_max_limits_);
  std::unordered_map<std::string, Polytope> W_i = {
    {"gripper_1", W_1},
    {"gripper_2", W_1.scaled(Eigen::Vector3d{1.0, -1.0, 1.0})},
    {"gripper_3", W_1.scaled(Eigen::Vector3d{-1.0, 1.0, 1.0})},
    {"gripper_4", W_1.scaled(Eigen::Vector3d{-1.0, -1.0, 1.0})}};

  // Select swing foot closest to rear edge of workspace
  Eigen::Isometry3d map_to_body = start.pose.inverse();
  std::string swing;
  std::vector<std::string> stance;
  double min_margin = INFINITY;
  for (const auto & [foot, pose] : start.footholds) {
    if (robot_->getContactType(foot) == ContactType::TAIL) {continue;}
    stance.push_back(foot);
    double margin = W_i.at(foot).distance(
      map_to_body * pose.translation(), map_to_body.rotation() * -direction);
    if (margin < min_margin) {
      swing = foot;
      min_margin = margin;
    }
  }
  stance.erase(std::remove(stance.begin(), stance.end(), swing), stance.end());
  size_t n = stance.size() + 1;
  std::cout << "Foot: " << swing << std::endl;

  // Compute workspace
  auto W_swing = Polytope::createBox(3);
  Eigen::Vector3d p0_swing = start.footholds.at(swing).translation();
  Eigen::Matrix3Xd p_stance = Eigen::Matrix3Xd::Zero(3, n);
  for (std::size_t i = 0; i < n - 1; ++i) {
    auto foot = stance.at(i);
    p_stance.col(i) = start.footholds.at(foot).translation();
    W_swing.intersect(
      W_i.at(swing) + -W_i.at(foot) + map_to_body * p_stance.col(i));
  }
  Eigen::Vector3d p_mid = (W_swing.b.tail(3) - W_swing.b.head(3)) / 2;
  W_swing = start.pose * W_swing;
  auto W_select = W_swing;
  W_select.addFacet(-direction, -direction.dot(p0_swing) - min_step_length_);
  auto W_centered =
    (start.pose.rotation() * W_i.at(swing) * n + p_stance.rowwise().sum()) / (n - 1);

  // Sample footholds in area ahead of current swing foothold
  Eigen::Vector3f p_search = (start.pose * p_mid).cast<float>();
  double radius = (W_swing.b.tail(3) + W_swing.b.head(3)).norm() / 2;
  pcl::PointXYZ p_search_pcl{p_search.x(), p_search.y(), p_search.z()};
  std::vector<int> indices;
  std::vector<float> distances;
  kdtree_->radiusSearch(p_search_pcl, radius, indices, distances);
  Eigen::VectorXi samples =
    Eigen::Map<Eigen::VectorXi>(indices.data(), indices.size());
  Eigen::Matrix3Xd p = p_(Eigen::all, samples);

  // Filter reachable footholds
  Eigen::VectorXi reachable = W_select.containsAll(p);
  reachable = samples(EigenUtils::maskToIndex(reachable));
  if (reachable.size() == 0) {
    drawBox(W_swing, 0);
    for (const auto & foot : stance) {
      drawBox(start.pose * W_i.at(foot), 250);
    }
    drawBox(start.pose * W_i.at(swing), 250);
    drawBox(W_centered, -500);
    cost_->getMatrixXfMap(1, 8, 4)(0, samples).setConstant(-500);
    cost_->getMatrixXfMap(1, 8, 4)(0, reachable).setConstant(1000);
    return result;
  }
  p = p_(Eigen::all, reachable);
  // Filter footholds with centered body if possible
  Eigen::VectorXi centered = W_centered.containsAll(p);
  if (centered.sum() > 0) {
    reachable = reachable(EigenUtils::maskToIndex(centered)).eval();
    p = p_(Eigen::all, reachable);
  } else {
    std::cout << "Warning: no centered footholds" << std::endl;
  }

  // Select foothold with lowest cost
  Eigen::VectorXd distance = (start.pose * W_i[swing]).distanceAll(
    p, -direction, workspace_angular_tol_);
  distance.array() -= min_margin;
  Eigen::Index index;
  (distance + c_(reachable) * 0.01).maxCoeff(&index);
  Eigen::Vector3d p_swing = p.col(index);

  // Update body pose
  Eigen::Vector3d p_body = p_stance.rowwise().sum() / n + p_swing / n;
  Polytope W_body = map_to_body * p_swing - W_i.at(swing);  // For debugging
  for (size_t i = 0; i < n - 1; ++i) {
    W_body.intersect(map_to_body * p_stance.col(i) - W_i.at(stance.at(i)));
  }
  W_body = start.pose * W_body;

  // Update body rotation
  Eigen::Matrix3Xd p_feet(3, n);
  p_feet.leftCols(p_stance.cols()) = p_stance;  // Assign first N columns
  p_feet.rightCols(1) = p_swing;
  p_feet.colwise() -= p_body;
  Eigen::Vector3d n0_body = start.pose.rotation().col(2);
  Eigen::Vector3d n_body = (p_feet * p_feet.transpose()).jacobiSvd(
    Eigen::ComputeFullU | Eigen::ComputeFullV).matrixU().col(2);
  if (n_body.dot(n0_body) < 0) {
    n_body *= -1;
  }
  Eigen::Matrix3d R =
    Eigen::Quaterniond::FromTwoVectors(n0_body, n_body).matrix();
  if ((R * W_swing).contains(p_swing)) {
    result.pose.prerotate(R);
  }

  result.pose.translation() = p_body;
  result.footholds.at(swing).translation() = p_swing;
  result.swing_foot = swing;
  return result;
}

void FootstepPlanner::drawBox(const Polytope & box, double value)
{
  assert(box.A.rows() == 6 && box.A.cols() == 3);
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  if (!box.box) {
    T.linear() = box.A.block(3, 0, 3, 3).transpose();
  }
  T.translation() = T.rotation() * (box.b.tail(3) - box.b.head(3)) / 2;
  Eigen::Vector3d size = box.b.tail(3) + box.b.head(3);
  if (size.minCoeff() <= 0) {
    return;
  }
  PointCloud pc = terrain::box(T, size(0), size(1), size(2), 0.005);
  CostCloud cc;
  cc.resize(pc.size());
  cc.getMatrixXfMap(4, 8, 0) = pc.getMatrixXfMap(4, 4, 0);
  cc.getMatrixXfMap(1, 8, 4).setConstant(value);
  *cost_ += cc;
  c_.conservativeResize(c_.cols() + pc.size());
  c_.tail(pc.size()).setConstant(value);
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
