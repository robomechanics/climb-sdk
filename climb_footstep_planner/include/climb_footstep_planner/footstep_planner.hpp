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
  };

  FootstepPlanner();

  bool initialize(const std::string & robot_description, std::string & message);

  bool isInitialized(std::string & message);

  void update(
    const PointCloud::Ptr & map_cloud, const Eigen::Isometry3d & viewpoint);

  void updateGravity(const Eigen::Quaterniond & orientation);

  void updateGravity(const Eigen::Vector3d & gravity) {gravity_ = gravity;}

  std::vector<Stance> plan(Stance start, Eigen::Isometry3d goal);

  std::vector<Stance> replan(Stance start);

  CostCloud::Ptr getCostCloud() {return cost_;}

  void declareParameters() override;

  void setParameter(
    const Parameter & param, SetParametersResult & result) override;

  using Parameterized::setParameter;

private:
  void computeNormals(std::vector<int> indices, double radius);

  std::shared_ptr<KinematicsInterface> robot_;
  Eigen::Vector3d gravity_;
  Eigen::Isometry3d viewpoint_;
  PointCloud::Ptr map_;
  NormalCloud::Ptr normals_;
  CurvatureCloud::Ptr curvatures_;
  CostCloud::Ptr cost_;
  std::vector<Stance> plan_;
  Eigen::Isometry3d goal_;
  Eigen::MatrixXf p_;   // Position
  Eigen::MatrixXf n_;   // Normal
  Eigen::MatrixXf t_;   // Tangential
  Eigen::MatrixXf k_;   // Curvature
  Eigen::MatrixXf c_;   // Cost
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_;
};

#endif  // CLIMB_FOOTSTEP_PLANNER__FOOTSTEP_PLANNER_HPP_
