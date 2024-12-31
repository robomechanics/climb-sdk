#ifndef FOOTSTEP_PLANNER_NODE_HPP
#define FOOTSTEP_PLANNER_NODE_HPP

#include "climb_footstep_planner/footstep_planner.hpp"
#include <climb_kinematics/kinematics_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_srvs/srv/trigger.hpp>

using std_msgs::msg::String;
using sensor_msgs::msg::PointCloud2;
using geometry_msgs::msg::PoseArray;
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
  void planCallback(
    const Trigger::Request::SharedPtr request,
    Trigger::Response::SharedPtr response);

  std::unique_ptr<FootstepPlanner> footstep_planner_;
  rclcpp::Subscription<PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Publisher<PoseArray>::SharedPtr footholds_pub_;
  std::vector<rclcpp::Publisher<Path>::SharedPtr> path_pubs_;
  rclcpp::Service<Trigger>::SharedPtr plan_service_;
  pcl::PointCloud<pcl::PointXYZ> point_cloud_;
};

#endif  // FOOTSTEP_PLANNER_NODE_HPP
