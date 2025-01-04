#include "climb_footstep_planner/footstep_planner_node.hpp"
#include <climb_util/ros_utils.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

FootstepPlannerNode::FootstepPlannerNode()
: KinematicsNode("FootstepPlannerNode")
{
  footstep_planner_ = std::make_unique<FootstepPlanner>();
  for (const auto & p : footstep_planner_->getParameters()) {
    if (this->has_parameter(p.name)) {
      this->set_parameter(this->get_parameter(p.name));
    } else {
      this->declare_parameter(p.name, p.default_value, p.descriptor);
    }
  }
  point_cloud_sub_ = create_subscription<PointCloud2>(
    "map_cloud", 1,
    std::bind(&FootstepPlannerNode::pointCloudCallback, this, _1));
  imu_sub_ = create_subscription<Imu>(
    "/imu/data", 1, std::bind(&FootstepPlannerNode::imuCallback, this, _1));
  cost_pub_ = create_publisher<PointCloud2>("cost_cloud", 1);
  footholds_pub_ = create_publisher<PoseArray>("planned_footholds", 1);
  path_pubs_.push_back(create_publisher<Path>("body_path", 1));
  plan_service_ = create_service<Trigger>(
    "plan", std::bind(&FootstepPlannerNode::planCallback, this, _1, _2));
  RCLCPP_INFO(this->get_logger(), "Footstep planner node initialized");
}

void FootstepPlannerNode::descriptionCallback(const String::SharedPtr msg)
{
  KinematicsNode::descriptionCallback(msg);
  std::string error_message;
  if (!footstep_planner_->initialize(msg->data, error_message)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to load robot description: %s", error_message.c_str());
  }
}

void FootstepPlannerNode::pointCloudCallback(
  const PointCloud2::SharedPtr msg)
{
  PointCloud::Ptr point_cloud = std::make_unique<PointCloud>();
  pcl::fromROSMsg(*msg, *point_cloud);
  footstep_planner_->update(std::move(point_cloud));
}

void FootstepPlannerNode::imuCallback(const Imu::SharedPtr msg)
{
  Eigen::Isometry3d map_to_imu = RosUtils::transformToEigen(
    lookupMapTransform("/" + msg->header.frame_id).transform);
  footstep_planner_->updateGravity(
    map_to_imu.rotation() * -RosUtils::vector3ToEigen(msg->linear_acceleration).normalized());
}

void FootstepPlannerNode::planCallback(
  const Trigger::Request::SharedPtr request [[maybe_unused]],
  Trigger::Response::SharedPtr response)
{
  std::string message;
  if (!footstep_planner_->isInitialized(message)) {
    response->success = false;
    response->message = message;
    return;
  }
  FootstepPlanner::Stance start;
  auto body_transform = lookupMapToBodyTransform();
  auto map_frame = body_transform.header.frame_id;
  start.pose = RosUtils::transformToEigen(body_transform.transform);
  start.joint_positions = robot_->getJointPosition();
  for (const auto & contact : robot_->getContactFrames()) {
    start.footholds[contact] = RosUtils::transformToEigen(
      lookupTransform(map_frame, contact).transform);
  }
  auto goal = start.pose;  // TODO: set goal pose
  auto plan = footstep_planner_->plan(start, goal);

  PointCloud2 cost_msg;
  pcl::toROSMsg(*footstep_planner_->getCostCloud(), cost_msg);
  cost_msg.header.frame_id = map_frame;
  cost_msg.header.stamp = now();
  cost_pub_->publish(cost_msg);

  PoseArray footholds_msg;
  footholds_msg.header.frame_id = map_frame;
  footholds_msg.header.stamp = now();
  std::unordered_map<std::string, Path> path_msgs;
  Path body_path_msg;
  body_path_msg.header.frame_id = map_frame;
  body_path_msg.header.stamp = now();
  for (const auto & stance : plan) {
    PoseStamped body_ps;
    body_ps.header.frame_id = map_frame;
    body_ps.pose = RosUtils::eigenToPose(stance.pose);
    body_path_msg.poses.push_back(body_ps);
    for (const auto & [contact, foothold] : stance.footholds) {
      if (path_msgs.find(contact) == path_msgs.end()) {
        path_msgs[contact] = Path();
        path_msgs[contact].header.frame_id = map_frame;
        path_msgs[contact].header.stamp = now();
      }
      PoseStamped ps;
      ps.header.frame_id = map_frame;
      ps.pose = RosUtils::eigenToPose(foothold);
      footholds_msg.poses.push_back(ps.pose);
      path_msgs[contact].poses.push_back(ps);
    }
  }
  auto contact_frames = robot_->getContactFrames();
  for (size_t i = 1; i < path_msgs.size() + 1; i++) {
    if (path_pubs_.size() == i) {
      path_pubs_.push_back(
        this->create_publisher<Path>(
          "foothold_path_" + std::to_string(i), 1));
    }
    path_pubs_.at(i)->publish(path_msgs[contact_frames[i - 1]]);
  }
  path_pubs_.at(0)->publish(body_path_msg);
  footholds_pub_->publish(footholds_msg);
  response->success = true;
}

rcl_interfaces::msg::SetParametersResult FootstepPlannerNode::parameterCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  auto result = KinematicsNode::parameterCallback(parameters);
  for (const auto & param : parameters) {
    footstep_planner_->setParameter(param, result);
  }
  return result;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FootstepPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
