#include "climb_footstep_planner/planners/global_planner.hpp"

#include <unordered_map>
#include <tuple>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

typedef std::array<double, 3> Vertex;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
struct VertexHash
{
  std::size_t operator()(const Vertex & vertex) const
  {
    std::size_t h1 = std::hash<double>{}(vertex[0]);
    std::size_t h2 = std::hash<double>{}(vertex[1]);
    std::size_t h3 = std::hash<double>{}(vertex[2]);
    return h1 ^ (h2 << 1) ^ (h3 << 2);
  }
};
struct EdgeHash
{
  std::size_t operator()(const std::pair<Vertex, Vertex> & edge) const
  {
    std::size_t h1 = VertexHash{}(edge.first);
    std::size_t h2 = VertexHash{}(edge.second);
    return h1 ^ (h2 << 1);
  }
};

typedef std::unordered_map<Vertex, Step, VertexHash> VertexMap;
typedef std::unordered_map<std::pair<Vertex, Vertex>, Plan, EdgeHash> EdgeMap;

Vertex stateToVertex(const ob::State *state)
{
  auto values = state->as<ob::RealVectorStateSpace::StateType>()->values;
  return {values[0], values[1], values[2]};
}

Eigen::Vector3d stateToVector(const ob::State *state)
{
  auto values = state->as<ob::RealVectorStateSpace::StateType>()->values;
  return Eigen::Vector3d(values[0], values[1], values[2]);
}

void setState(ob::State *state, const Eigen::Vector3d & vector)
{
  auto values = state->as<ob::RealVectorStateSpace::StateType>()->values;
  values[0] = vector.x();
  values[1] = vector.y();
  values[2] = vector.z();
}

class PointCloudSampler : public ob::StateSampler
{
public:
  PointCloudSampler(const ob::StateSpace *si, PointCloud::Ptr cloud)
  : StateSampler(si), cloud_(cloud)
  {
  }

  void sampleUniform(ob::State *state) override
  {
    int sample = rng_.uniformInt(0, cloud_->size() - 1);
    double * values =
      static_cast<ob::RealVectorStateSpace::StateType *>(state)->values;
    auto point = cloud_->at(sample);
    values[0] = point.x;
    values[1] = point.y;
    values[2] = point.z;
  }

  void sampleUniformNear(ob::State *, const ob::State *, const double) override
  {
    throw ompl::Exception(
      "PointCloudStateSampler::sampleUniformNear", "not implemented");
  }

  void sampleGaussian(ob::State *, const ob::State *, const double) override
  {
    throw ompl::Exception(
      "PointCloudStateSampler::sampleGaussian", "not implemented");
  }

protected:
  PointCloud::Ptr cloud_;
};

class PointCloudStateSpace : public ob::RealVectorStateSpace
{
public:
  PointCloudStateSpace(PointCloud::Ptr cloud)
  : RealVectorStateSpace(3), cloud_(cloud), density_(getDensity(cloud)) {}

  ob::StateSamplerPtr allocDefaultStateSampler() const override
  {
    return std::make_shared<PointCloudSampler>(this, cloud_);
  }

  unsigned int getDimension() const override
  {
    return 2;
  }

  double getMeasure() const override
  {
    return cloud_->size() / density_;
  }

private:
  double getDensity(
    const PointCloud::Ptr cloud, double radius = 0.1, int samples = 100) const
  {
    int count = 0;
    pcl::search::KdTree<pcl::PointXYZ> kdtree(false);
    std::vector<int> indices;
    std::vector<float> distances;
    kdtree.setInputCloud(cloud);
    for (int i = 0; i < samples; ++i) {
      indices.clear();
      distances.clear();
      int sample = rand() % cloud->size();
      kdtree.radiusSearch(sample, radius, indices, distances);
      count += indices.size();
    }
    return count / (M_PI * radius * radius * samples);
  }

  PointCloud::Ptr cloud_;
  double density_;
};

class LocalPlannerMotionValidator : public ob::MotionValidator
{
public:
  LocalPlannerMotionValidator(
    const ob::SpaceInformationPtr & si, Planner * local_planner,
    VertexMap * vertices, EdgeMap * edges, double radius)
  : MotionValidator(si), local_planner_(local_planner),
    vertices_(vertices), edges_(edges), radius_(radius) {}

  bool checkMotion(
    const ob::State *s1, const ob::State *s2) const override
  {
    if ((stateToVector(s1) - stateToVector(s2)).norm() > radius_) {
      return false;
    }
    auto v1 = stateToVertex(s1);
    auto v2 = stateToVertex(s2);
    
    auto edge = std::make_pair(v1, v2);
    if (vertices_->find(v1) == vertices_->end()) {
      throw ompl::Exception(
        "LocalPlannerMotionValidator::checkMotion", "start state not found");
    }
    if (edges_->find(edge) == edges_->end()) {
      auto start = vertices_->at(v1);
      auto goal = Eigen::Isometry3d::Identity();
      goal.translation() = stateToVector(s2);
      edges_->emplace(edge, local_planner_->plan(start, goal));
    }
    if (edges_->at(edge).cost < INFINITY) {
      vertices_->emplace(v2, edges_->at(edge).back());
      return true;
    }
    return false;
  }

  bool checkMotion(
    const ob::State *, const ob::State *,
    std::pair<ob::State *, double> &) const override
  {
    throw ompl::Exception(
      "LocalPlannerMotionValidator::checkMotion", "not implemented");
  }

private:
  Planner * local_planner_;
  VertexMap * vertices_;
  EdgeMap * edges_;
  double radius_;
};

class InclineCostIntegralObjective : public ob::StateCostIntegralObjective
{
public:
  InclineCostIntegralObjective(
    const ob::SpaceInformationPtr & si, const PointCloud::Ptr cloud,
    const Eigen::Isometry3d & viewpoint, const Eigen::Vector3d & gravity,
    double radius, double incline_cost)
  : StateCostIntegralObjective(si),
    cloud_(cloud),
    viewpoint_(viewpoint.translation()),
    gravity_(gravity),
    radius_(radius),
    incline_cost_(incline_cost),
    kdtree_(std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>(false))
  {
    kdtree_->setInputCloud(cloud_);
  }

  ob::Cost stateCost(const ob::State *state) const override
  {
    auto values = state->as<ob::RealVectorStateSpace::StateType>()->values;
    auto point = pcl::PointXYZ(values[0], values[1], values[2]);
    pcl::Indices indices;
    std::vector<float> distance;
    kdtree_->radiusSearch(point, radius_, indices, distance);
    Eigen::Vector4f plane;
    float curvature;
    pcl::computePointNormal(*cloud_, indices, plane, curvature);
    Eigen::Vector3d normal = plane.head(3).cast<double>();
    if (normal.dot(viewpoint_ - point.getVector3fMap().cast<double>()) < 0) {
      normal *= -1;
    }
    return ob::Cost(incline_cost_ * (normal.dot(gravity_) + 1) + 1);
  }

private:
  PointCloud::Ptr cloud_;
  Eigen::Vector3d viewpoint_;
  Eigen::Vector3d gravity_;
  double radius_;
  double incline_cost_;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_;
};

ob::Cost costToGoHeuristic(
  const ob::State *state, const ob::Goal *goal)
{
  auto p1 = stateToVector(state);
  ob::State * goal_state = goal->getSpaceInformation()->allocState();
  goal->as<ob::GoalSampleableRegion>()->sampleGoal(goal_state);
  auto p2 = stateToVector(goal_state);
  return ob::Cost((p1 - p2).norm());
}

GlobalPlanner::GlobalPlanner(
  std::shared_ptr<KinematicsInterface> robot,
  std::unique_ptr<Planner> local_planner)
: Planner(robot),
  local_planner_(std::move(local_planner)) {}

bool GlobalPlanner::isInitialized(std::string & message)
{
  return Planner::isInitialized(message) &&
         local_planner_->isInitialized(message);
}

void GlobalPlanner::update(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud,
  const Eigen::Isometry3d & viewpoint,
  const Eigen::Vector3d & gravity)
{
  Planner::update(map_cloud, viewpoint, gravity);
  local_planner_->update(map_cloud, viewpoint, gravity);
}

Plan GlobalPlanner::plan(const Step & start, const Eigen::Isometry3d & goal)
{
  auto space = std::make_shared<PointCloudStateSpace>(map_);
  ob::RealVectorBounds bounds(3);
  Eigen::Vector3d lb = (map_->getMatrixXfMap(3, 4, 0).rowwise().minCoeff()).cast<double>();
  Eigen::Vector3d ub = (map_->getMatrixXfMap(3, 4, 0).rowwise().maxCoeff()).cast<double>();
  for (size_t i = 0; i < 3; ++i) {
    bounds.setLow(i, lb[i] - 0.5);
    bounds.setHigh(i, ub[i] + 0.5);
  }
  space->setBounds(bounds);
  og::SimpleSetup ss(space);
  ob::ScopedState<> start_state(space);
  setState(start_state.get(), start.pose.translation());
  ob::ScopedState<> goal_state(space);
  setState(goal_state.get(), goal.translation());
  ss.setStartAndGoalStates(start_state, goal_state);
  auto si = ss.getSpaceInformation();
  VertexMap vertices;
  EdgeMap edges;
  vertices.emplace(stateToVertex(start_state.get()), start);
  if (algorithm_ == "ait*") {
    si->setMotionValidator(std::make_shared<LocalPlannerMotionValidator>(
      si, local_planner_.get(), &vertices, &edges, search_radius_));
  } else {
    si->setMotionValidator(std::make_shared<LocalPlannerMotionValidator>(
      si, local_planner_.get(), &vertices, &edges, INFINITY));
  }
  auto opt = std::make_shared<InclineCostIntegralObjective>(
    si, map_, viewpoint_, gravity_, global_incline_radius_,
    global_incline_cost_);
  opt->setCostToGoHeuristic(&costToGoHeuristic);
  ss.setOptimizationObjective(opt);
  ob::PlannerPtr planner;
  if (algorithm_ == "ait*") {
    auto ait = std::make_shared<og::AITstar>(si);
    planner = ait;
  } else if (algorithm_ == "rrt*") {
    auto rrt = std::make_shared<og::RRTstar>(si);
    rrt->setRange(search_radius_);
    planner = rrt;
  }
  ss.setPlanner(planner);
  std::cout << "Planning..." << std::endl;
  Plan plan;
  auto status = ss.solve(runtime_);
  if (status == ob::PlannerStatus::EXACT_SOLUTION ||
    status == ob::PlannerStatus::APPROXIMATE_SOLUTION)
  {
    if (status == ob::PlannerStatus::APPROXIMATE_SOLUTION) {
      std::cout << "Found approximate solution" << std::endl;
    } else {
      std::cout << "Found solution!" << std::endl;
    }
    for (size_t i = 0; i < ss.getSolutionPath().getStateCount() - 1; ++i) {
      auto s1 = stateToVertex(ss.getSolutionPath().getState(i));
      auto s2 = stateToVertex(ss.getSolutionPath().getState(i + 1));
      auto edge = std::make_pair(s1, s2);
      if (edges.find(edge) != edges.end()) {
        if (plan.size() && edges.at(edge).size() &&
          plan.back().swing_foot == edges.at(edge).front().swing_foot)
        {
          plan.pop_back();
        }
        plan += edges.at(edge);
      }
    }
    std::cout << "Plan Cost = " << plan.cost << std::endl;
  } else {
    std::cout << "No solution found." << std::endl;
    plan.cost = INFINITY;
  }
  size_t i = 0;
  graph_.resize(6, vertices_only_ ? edges.size() : 0);
  for (const auto & [edge, plan] : edges) {
    if (vertices_only_) {
      graph_.col(i) <<
        edge.first[0], edge.first[1], edge.first[2],
        edge.second[0], edge.second[1], edge.second[2];
      ++i;
    } else {
      graph_.conservativeResize(6, i + plan.size());
      auto parent = vertices.at(edge.first);
      for (const auto & step : plan) {
        graph_.col(i) << parent.pose.translation(), step.pose.translation();
        parent = step;
        ++i;
      }
      graph_.conservativeResize(6, i);
    }
  }

  return plan;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr GlobalPlanner::getCostmap()
{
  return local_planner_->getCostmap();
}

void GlobalPlanner::declareParameters()
{
  for (const auto & param : local_planner_->getParameters()) {
    parameters_.push_back(param);
  }
  declareParameter("algorithm", "rrt*", "Base planning algorithm");
  declareParameter("runtime", 10.0, "Maximum planning time in seconds", 0.0);
  declareParameter("search_radius", 1.0,
    "Maximum distance to connect neighbors in search graph", 0.0);
  declareParameter("global_incline_radius", 0.2,
    "Radius for robot-scale incline estimation", 0.0);
  declareParameter("global_incline_cost", 1.0,
    "Penalty for robot-scale incline relative to Euclidean distance");
  declareParameter("vertices_only", false,
    "Hide intermediate steps between global planner vertices");
}

void GlobalPlanner::setParameter(
  const Parameter & param, SetParametersResult & result)
{
  if (param.get_name() == "debug") {
    debug_ = param.as_bool();
    return;
  } else if (param.get_name() == "algorithm") {
    algorithm_ = param.as_string();
  } else if (param.get_name() == "runtime") {
    runtime_ = param.as_double();
  } else if (param.get_name() == "search_radius") {
    search_radius_ = param.as_double();
  } else if (param.get_name() == "global_incline_radius") {
    global_incline_radius_ = param.as_double();
  } else if (param.get_name() == "global_incline_cost") {
    global_incline_cost_ = param.as_double();
  } else if (param.get_name() == "vertices_only") {
    vertices_only_ = param.as_bool();
  }
  local_planner_->setParameter(param, result);
}
