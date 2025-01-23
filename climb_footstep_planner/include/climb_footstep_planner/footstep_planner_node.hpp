#ifndef CLIMB_FOOTSTEP_PLANNER__FOOTSTEP_PLANNER_NODE_HPP_
#define CLIMB_FOOTSTEP_PLANNER__FOOTSTEP_PLANNER_NODE_HPP_

#include <Eigen/Geometry>
#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/path.hpp>

#include <climb_msgs/srv/set_string.hpp>
#include <climb_kinematics/kinematics_node.hpp>
#include "climb_footstep_planner/footstep_planner.hpp"

using climb_msgs::srv::SetString;
using std_msgs::msg::String;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::Imu;
using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Path;
using std_srvs::srv::Trigger;

class FootstepPlannerNode : public KinematicsNode
{
public:
  FootstepPlannerNode();
  void update();
  rcl_interfaces::msg::SetParametersResult parameterCallback(
    const std::vector<rclcpp::Parameter> & parameters) override;

private:
  void descriptionCallback(const String::SharedPtr msg) override;
  void pointCloudCallback(const PointCloud2::SharedPtr msg);
  void imuCallback(const Imu::SharedPtr msg);
  void planCallback(
    const Trigger::Request::SharedPtr request,
    Trigger::Response::SharedPtr response);
  void simulateCallback(
    const SetString::Request::SharedPtr request,
    SetString::Response::SharedPtr response);

  std::unique_ptr<FootstepPlanner> footstep_planner_;
  rclcpp::Subscription<PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr cost_pub_;
  rclcpp::Publisher<PoseStamped>::SharedPtr goal_pub_;
  std::vector<rclcpp::Publisher<Path>::SharedPtr> path_pubs_;
  rclcpp::Service<Trigger>::SharedPtr plan_service_;
  rclcpp::Service<SetString>::SharedPtr simulate_service_;
  int seed_;
  Eigen::Isometry3d goal_;
};

#endif  // CLIMB_FOOTSTEP_PLANNER__FOOTSTEP_PLANNER_NODE_HPP_
