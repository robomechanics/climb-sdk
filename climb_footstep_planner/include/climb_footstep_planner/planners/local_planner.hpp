#ifndef CLIMB_FOOTSTEP_PLANNER__PLANNERS__LOCAL_PLANNER_HPP_
#define CLIMB_FOOTSTEP_PLANNER__PLANNERS__LOCAL_PLANNER_HPP_

#include <pcl/search/kdtree.h>
#include <string>
#include <memory>
#include <unordered_map>

#include <climb_util/geometry_utils.hpp>
#include "climb_footstep_planner/planners/planner.hpp"

class LocalPlanner : public Planner
{
public:
  explicit LocalPlanner(std::shared_ptr<KinematicsInterface> robot);

  void update(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud,
    const Eigen::Isometry3d & viewpoint,
    const Eigen::Vector3d & gravity = Eigen::Vector3d(0, 0, -1)) override;

  Plan plan(const Step & start, const Eigen::Isometry3d & goal) override;

  void declareParameters() override;

  void setParameter(
    const Parameter & param, SetParametersResult & result) override;

  using Parameterized::setParameter;

private:
  /**
   * @brief Find a single step toward the goal
   * @param start Initial robot state
   * @param goal Goal pose for body
   * @return Next step toward the goal
   */
  Step planStep(const Step & start, const Eigen::Isometry3d & goal);

  /**
   * @brief Precompute normals, curvatures, and cost function
   */
  void processCloud();

  /**
   * @brief Draw a box in the cost map for visualization purposes
   * @param box Box geometry
   * @param value Cost value assigned to the box
   */
  void drawBox(const geometry_utils::Polytope & box, double value = 0);

  Eigen::MatrixXd p_;   // Position
  Eigen::MatrixXd n_;   // Normal
  Eigen::MatrixXd t_;   // Tangential
  Eigen::MatrixXd k_;   // Curvature
  Eigen::VectorXd c_;   // Cost
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_;
  bool processed_cloud = false;
  Eigen::Vector3d workspace_min_limits_;
  Eigen::Vector3d workspace_max_limits_;
  double workspace_angular_tol_;
  double min_step_length_;
  double curvature_limit_;
  double curvature_radius_;
  double incline_limit_;
  double incline_radius_;
  double incline_cost_;
  bool debug_;
  std::unordered_map<std::string, double> timers_;
};

#endif  // CLIMB_FOOTSTEP_PLANNER__PLANNERS__LOCAL_PLANNER_HPP_
