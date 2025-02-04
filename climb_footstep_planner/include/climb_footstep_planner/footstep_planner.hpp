#ifndef CLIMB_FOOTSTEP_PLANNER__FOOTSTEP_PLANNER_HPP_
#define CLIMB_FOOTSTEP_PLANNER__FOOTSTEP_PLANNER_HPP_

#include <Eigen/Geometry>
#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>
#include <string>
#include <memory>
#include <unordered_map>
#include <vector>

#include <climb_kinematics/interfaces/kinematics_interface.hpp>
#include <climb_util/geometry_utils.hpp>
#include <climb_util/parameterized.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::Normal> NormalCloud;
typedef pcl::PointCloud<pcl::PrincipalCurvatures> CurvatureCloud;
typedef pcl::PointCloud<pcl::PointXYZI> CostCloud;

class FootstepPlanner : public Parameterized
{
public:
  struct Stance
  {
    Eigen::Isometry3d pose;
    Eigen::VectorXd joint_positions;
    std::unordered_map<std::string, Eigen::Isometry3d> footholds;
    std::string swing_foot;   // Most recently moved end effector
  };

  FootstepPlanner();

  bool initialize(const std::string & robot_description, std::string & message);

  bool isInitialized(std::string & message);

  void update(
    const PointCloud::Ptr & map_cloud, const Eigen::Isometry3d & viewpoint);

  void updateGravity(const Eigen::Quaterniond & orientation);

  void updateGravity(const Eigen::Vector3d & gravity) {gravity_ = gravity;}

  /**
   * @brief Precompute normals, curvatures, and cost function
   */
  void processCloud();

  /**
   * @brief Find a global path through the current terrain map
   * @param start Initial stance
   * @param goal Goal pose
   * @return Sequence of stances
   */
  std::vector<Stance> plan(
    const Stance & start, const Eigen::Isometry3d & goal);

  /**
   * @brief Find a path from a new starting stance to the previous goal
   * @param start Initial stance
   * @return Sequence of stances
   */
  std::vector<Stance> replan(const Stance & start);

  /**
   * @brief Find a local path from the given stance to an intermediate goal
   * @param start Initial stance
   * @param goal Goal pose
   * @return Sequence of stances
   */
  std::vector<Stance> localPlan(
    const Stance & start, const Eigen::Isometry3d & goal);

  /**
   * @brief Find a single step toward the goal
   * @param start Initial stance
   * @param goal Goal pose
   * @return Next stance
   */
  Stance step(const Stance & start, const Eigen::Isometry3d & goal);

  /**
   * @brief Get the current cost map for visualization
   * @return Cost map
   */
  CostCloud::Ptr getCostCloud() {return cost_;}

  void declareParameters() override;

  void setParameter(
    const Parameter & param, SetParametersResult & result) override;

  using Parameterized::setParameter;

private:
  void computeNormals(std::vector<int> indices, double radius);

  void drawBox(const geometry_utils::Polytope & box, double value = 0);

  std::shared_ptr<KinematicsInterface> robot_;
  Eigen::Vector3d gravity_;
  Eigen::Isometry3d viewpoint_;
  PointCloud::Ptr map_;
  NormalCloud::Ptr normals_;
  CurvatureCloud::Ptr curvatures_;
  CostCloud::Ptr cost_;
  std::vector<Stance> plan_;
  Eigen::Isometry3d goal_;
  Eigen::MatrixXd p_;   // Position
  Eigen::MatrixXd n_;   // Normal
  Eigen::MatrixXd t_;   // Tangential
  Eigen::MatrixXd k_;   // Curvature
  Eigen::VectorXd c_;   // Cost
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_;
  Eigen::Vector3d workspace_min_limits_;
  Eigen::Vector3d workspace_max_limits_;
  double workspace_angular_tol_;
  double min_step_length_;
  double curvature_limit_;
  double curvature_radius_;
  double incline_limit_;
  double incline_radius_;
  double incline_cost_;
};

#endif  // CLIMB_FOOTSTEP_PLANNER__FOOTSTEP_PLANNER_HPP_
