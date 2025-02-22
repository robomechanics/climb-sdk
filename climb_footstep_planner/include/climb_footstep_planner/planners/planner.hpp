#ifndef CLIMB_FOOTSTEP_PLANNER__PLANNERS__PLANNER_HPP_
#define CLIMB_FOOTSTEP_PLANNER__PLANNERS__PLANNER_HPP_

#include <Eigen/Geometry>
#include <pcl/common/common.h>
#include <initializer_list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <climb_kinematics/interfaces/kinematics_interface.hpp>
#include <climb_util/parameterized.hpp>

/**
 * @brief The full robot state at the completion of a step
 */
struct Step
{
  Eigen::Isometry3d pose;
  std::unordered_map<std::string, Eigen::Isometry3d> footholds;
  Eigen::VectorXd joint_positions;
  std::string swing_foot;
  double cost;
};

/**
 * @brief A sequence of steps
 */
struct Plan
{
  Plan()
  : cost(0) {}
  Plan(std::initializer_list<Step> steps)
  : cost(0)
  {
    for (const auto & step : steps) {
      push_back(step);
    }
  }
  void push_back(const Step & step)
  {
    steps.push_back(step);
    cost += step.cost;
  }
  Plan & operator+=(const Plan & plan)
  {
    steps.insert(steps.end(), plan.steps.begin(), plan.steps.end());
    cost += plan.cost;
    return *this;
  }
  Plan operator+(const Plan & plan)
  {
    Plan result = *this;
    return result += plan;
  }
  size_t size() const {return steps.size();}
  bool empty() const {return steps.empty();}
  auto begin() {return steps.begin();}
  auto end() {return steps.end();}
  auto begin() const {return steps.cbegin();}
  auto end() const {return steps.cend();}
  void pop_back() {steps.pop_back();}
  const Step & front() const {return steps.front();}
  const Step & back() const {return steps.back();}
  Step & front() {return steps.front();}
  Step & back() {return steps.back();}
  std::vector<Step> steps;
  double cost;
};

/**
 * @brief Base class for footstep planners
 */
class Planner : public Parameterized
{
public:
  explicit Planner(std::shared_ptr<KinematicsInterface> robot)
  : robot_(robot),
    costmap_(std::make_shared<pcl::PointCloud<pcl::PointXYZI>>()) {}

  virtual ~Planner() = default;

  /**
   * @brief Check if the planner is ready for use
   * @param[out] message Reason for failure
   * @return True if the planner is ready
   */
  virtual bool isInitialized(std::string & message)
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

  /**
   * @brief Update the terrain map with the latest data
   * @param map_cloud Point cloud representing the terrain map
   * @param viewpoint Pose of the camera for determining surface normals
   */
  virtual void update(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud,
    const Eigen::Isometry3d & viewpoint,
    const Eigen::Vector3d & gravity = Eigen::Vector3d(0, 0, -1))
  {
    map_ = map_cloud;
    viewpoint_ = viewpoint;
    gravity_ = gravity.normalized();
  }

  /**
   * @brief Find an optimal path through the current terrain map
   * @param start Initial robot state
   * @param goal Goal pose for body
   * @return A successful plan or an empty plan on failure
   */
  virtual Plan plan(
    const Step & start, const Eigen::Isometry3d & goal) = 0;

  /**
   * @brief Get the current cost map for visualization
   * @return Point cloud with cost values for intensity
   */
  virtual pcl::PointCloud<pcl::PointXYZI>::Ptr getCostmap() {return costmap_;}

  /**
   * @brief Get a graph of all explored edges for visualization
   * @return Matrix with one column per edge, each represented by two stacked
   * vertices
   */
  virtual Eigen::Matrix<double, 6, Eigen::Dynamic> getGraph() {return graph_;}

protected:
  std::shared_ptr<KinematicsInterface> robot_;
  Eigen::Isometry3d viewpoint_;
  Eigen::Matrix<double, 6, Eigen::Dynamic> graph_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr costmap_;
  Eigen::Vector3d gravity_;
};

#endif  // CLIMB_FOOTSTEP_PLANNER__PLANNERS__PLANNER_HPP_
