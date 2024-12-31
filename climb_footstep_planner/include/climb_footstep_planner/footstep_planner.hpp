#ifndef FOOTSTEP_PLANNER_HPP
#define FOOTSTEP_PLANNER_HPP

#include <climb_kinematics/kinematics_interfaces/kinematics_interface.hpp>
#include <pcl/common/common.h>
#include <Eigen/Geometry>
#include <unordered_map>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class FootstepPlanner
{
public:
  struct Stance
  {
    Eigen::Isometry3d pose;
    Eigen::VectorXd joint_positions;
    std::unordered_map<std::string, Eigen::Isometry3d> footholds;
  };

  FootstepPlanner();

  bool initialize(const std::string & robot_description);

  bool isInitialized()
  {
    return robot_->isInitialized() && map_cloud_.size() > 0;
  }

  void update(const PointCloud & map_cloud) {map_cloud_ = map_cloud;}

  std::vector<Stance> plan(Stance start, Eigen::Isometry3d goal);

  std::vector<Stance> replan(Stance start);

private:
  std::shared_ptr<KinematicsInterface> robot_;
  PointCloud map_cloud_;
  std::vector<Stance> plan_;
  Eigen::Isometry3d goal_;
};

#endif  // FOOTSTEP_PLANNER_HPP
