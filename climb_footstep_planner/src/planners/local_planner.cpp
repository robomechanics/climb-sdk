#include "climb_footstep_planner/planners/local_planner.hpp"

#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>

#include <climb_util/eigen_utils.hpp>
#include <climb_util/ros_utils.hpp>
#include "climb_footstep_planner/terrain_generator.hpp"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZI> CostCloud;
using geometry_utils::Polytope;

LocalPlanner::LocalPlanner(std::shared_ptr<KinematicsInterface> robot)
: Planner(robot),
  kdtree_(new pcl::search::KdTree<pcl::PointXYZ>(false)) {}

void LocalPlanner::update(
  const PointCloud::Ptr map_cloud,
  const Eigen::Isometry3d & viewpoint,
  const Eigen::Vector3d & gravity)
{
  Planner::update(map_cloud, viewpoint, gravity);
  kdtree_->setInputCloud(map_);
  processed_cloud = false;
}

void LocalPlanner::processCloud()
{
  // Compute normals
  computeNormals(n_, incline_radius_);
  orientNormals(n_, flip_normals_nn_);
  computeCurvatures(t_, k_, n_, curvature_radius_);

  // Compute cost
  p_ = map_->getMatrixXfMap(3, 4, 0).cast<double>();
  c_ = gravity_.transpose() * n_;       // Incline-based cost
  Eigen::Array<bool, Eigen::Dynamic, 1> n_mask =
    c_.array() <= -cos(incline_limit_);
  Eigen::Array<bool, Eigen::Dynamic, 1> k_mask =
    k_.colwise().sum().array() <= curvature_limit_;
  c_ = (n_mask && k_mask).select(c_, INFINITY);
  costmap_->resize(map_->size());
  costmap_->getMatrixXfMap(4, 8, 0) = map_->getMatrixXfMap(4, 4, 0);
  costmap_->getMatrixXfMap(1, 8, 4).row(0) = c_.cast<float>();

  processed_cloud = true;
}

Plan LocalPlanner::plan(const Step & start, const Eigen::Isometry3d & goal)
{
  if (!processed_cloud) {
    processCloud();
  }
  Plan local_plan;
  auto step = start;
  Eigen::Vector3d n;
  Eigen::Vector3d d = {INFINITY, INFINITY, INFINITY};
  while (d.norm() > min_step_length_) {
    step = planStep(step, goal);
    if (step.swing_foot.empty()) {
      local_plan.cost = INFINITY;
      if (debug_) {
        std::cout << "Plan failed" << std::endl;
      }
      return local_plan;
    }
    if (!local_plan.empty() &&
      step.swing_foot == local_plan.back().swing_foot)
    {
      local_plan.back() = step;
    } else {
      local_plan.push_back(step);
    }
    n = step.pose.rotation().col(2);
    d = goal.translation() - step.pose.translation();
    d -= d.dot(n) * n;
  }
  if (debug_) {
    std::cout << "Plan succeeded" << std::endl;
  }
  return local_plan;
}

Step LocalPlanner::planStep(
  const Step & start, const Eigen::Isometry3d & goal)
{
  Step step = start;
  step.swing_foot = "";
  Eigen::Vector3d p0_body = start.pose.translation();
  Eigen::Vector3d direction = (goal.translation() - p0_body).normalized();
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
  if (debug_) {
    std::cout << "Foot: " << swing << std::endl;
  }

  // Compute workspace
  auto W_swing = Polytope::createBox(3);
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
  Eigen::VectorXi valid = (c_(samples).array() < INFINITY).cast<int>();
  samples = samples(EigenUtils::maskToIndex(valid)).eval();
  Eigen::Matrix3Xd p = p_(Eigen::all, samples);

  // Filter reachable footholds
  Eigen::VectorXi reachable = W_select.containsAll(p);
  reachable = samples(EigenUtils::maskToIndex(reachable));
  p = p_(Eigen::all, reachable);
  // Filter footholds closer to the goal
  Eigen::Vector3d stance_sum = p_stance.rowwise().sum();
  Eigen::Vector3d start_centroid = (stance_sum + start.footholds.at(swing).translation()) / n;
  double goal_distance = (goal.translation() - start_centroid).norm();
  Eigen::VectorXd dist =
    ((p / n).colwise() + (stance_sum / n - goal.translation())).colwise().norm();
  Eigen::VectorXi closer = (dist.array() < std::max(
      goal_distance - min_step_length_ / n, min_step_length_ / n)).cast<int>();
  reachable = reachable(EigenUtils::maskToIndex(closer)).eval();
  p = p_(Eigen::all, reachable);
  // No reachable footholds found
  if (reachable.size() == 0) {
    if (debug_) {
      drawBox(W_swing, 0);
      for (const auto & foot : stance) {
        drawBox(start.pose * W_i.at(foot), 250);
      }
      drawBox(start.pose * W_i.at(swing), 250);
      drawBox(W_centered, -500);
      costmap_->getMatrixXfMap(1, 8, 4)(0, samples).setConstant(-500);
      costmap_->getMatrixXfMap(1, 8, 4)(0, reachable).setConstant(1000);
    }
    return step;
  }
  // Filter footholds with centered body if possible
  Eigen::VectorXi centered = W_centered.containsAll(p);
  if (centered.sum() > 0) {
    reachable = reachable(EigenUtils::maskToIndex(centered)).eval();
    p = p_(Eigen::all, reachable);
  } else {
    if (debug_) {
      std::cout << "Warning: no centered footholds" << std::endl;
    }
  }

  // Select foothold with lowest cost
  Eigen::VectorXd distance = (start.pose * W_i[swing]).distanceAll(
    p, -direction, workspace_angular_tol_);
  distance.array() -= min_margin;
  Eigen::Index index;
  (distance - c_(reachable) * incline_cost_).maxCoeff(&index);
  Eigen::Vector3d p_swing = p.col(index);

  // Update body pose
  Eigen::Vector3d p_body = p_stance.rowwise().sum() / n + p_swing / n;
  // Polytope W_body = map_to_body * p_swing - W_i.at(swing);  // For debugging
  // for (size_t i = 0; i < n - 1; ++i) {
  //   W_body.intersect(map_to_body * p_stance.col(i) - W_i.at(stance.at(i)));
  // }
  // W_body = start.pose * W_body;

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
    step.pose.prerotate(R);
  }
  Eigen::Vector3d n_swing = n_.col(reachable(index));
  step.footholds.at(swing).linear() = Eigen::Quaterniond::FromTwoVectors(
    Eigen::Vector3d{-1, 0, 0}, n_swing).matrix();

  // Compute ground clearance
  pcl::PointXYZ p_body_pcl{p_body.cast<float>().x(), p_body.cast<float>().y(),
    p_body.cast<float>().z()};
  indices.clear();
  distances.clear();
  kdtree_->radiusSearch(p_body_pcl, 0.1, indices, distances);
  Eigen::Matrix3Xd ground = p_(Eigen::all, indices);
  double clearance = (n_body.transpose() * ground).maxCoeff() - p_body.dot(n_body);
  p_body += clearance * n_body;

  step.pose.translation() = p_body;
  step.footholds.at(swing).translation() = p_swing;
  step.swing_foot = swing;
  return step;
}

void LocalPlanner::drawBox(const Polytope & box, double value)
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
  *costmap_ += cc;
  c_.conservativeResize(c_.cols() + pc.size());
  c_.tail(pc.size()).setConstant(value);
}

void LocalPlanner::declareParameters()
{
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
  declareParameter(
    "curvature_limit", 1.0, "Maximum foothold curvature in 1/meters", 0.0);
  declareParameter(
    "curvature_radius", 0.05, "Radius for foothold curvature estimation", 0.0);
  declareParameter(
    "incline_limit", M_PI,
    "Maximum foothold incline from the ground plane in radians", 0.0, M_PI);
  declareParameter(
    "incline_radius", 0.1, "Radius for foothold incline estimation", 0.0);
  declareParameter(
    "incline_cost", 0.0, "Penalty for incline relative to step distance", 0.0);
  declareParameter(
    "debug", false, "Display debugging information");
  declareParameter(
    "flip_normals_nn", 0,
    "Nearest neighbors for reorientating normal vectors (0 to disable)", 0);
}

void LocalPlanner::setParameter(
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
  } else if (param.get_name() == "curvature_limit") {
    curvature_limit_ = param.as_double();
  } else if (param.get_name() == "curvature_radius") {
    curvature_radius_ = param.as_double();
  } else if (param.get_name() == "incline_limit") {
    incline_limit_ = param.as_double();
  } else if (param.get_name() == "incline_radius") {
    incline_radius_ = param.as_double();
  } else if (param.get_name() == "incline_cost") {
    incline_cost_ = param.as_double();
  } else if (param.get_name() == "debug") {
    debug_ = param.as_bool();
  } else if (param.get_name() == "flip_normals_nn") {
    flip_normals_nn_ = param.as_int();
  }
}
