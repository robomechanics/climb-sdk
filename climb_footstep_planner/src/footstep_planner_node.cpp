#include "climb_footstep_planner/footstep_planner_node.hpp"

#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

#include <climb_msgs/msg/footstep.hpp>
#include <climb_util/ros_utils.hpp>
#include "climb_footstep_planner/terrain_generator.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using climb_msgs::msg::Footstep;

FootstepPlannerNode::FootstepPlannerNode()
: KinematicsNode("FootstepPlannerNode")
{
  footstep_planner_ = std::make_unique<FootstepPlanner>();
  for (const auto & p : footstep_planner_->getParameters()) {
    if (has_parameter(p.name)) {
      set_parameter(get_parameter(p.name));
    } else {
      declare_parameter(p.name, p.default_value, p.descriptor);
    }
  }
  declare_parameter("seed", 0);
  goal_ = Eigen::Isometry3d::Identity();
  point_cloud_sub_ = create_subscription<PointCloud2>(
    "map_cloud", 1,
    std::bind(&FootstepPlannerNode::pointCloudCallback, this, _1));
  imu_sub_ = create_subscription<Imu>(
    "/imu/data", 1, std::bind(&FootstepPlannerNode::imuCallback, this, _1));
  goal_sub_ = create_subscription<PoseStamped>(
    "goal", 1, [this](const PoseStamped::SharedPtr msg) {
      goal_ = RosUtils::poseToEigen(msg->pose);
    });
  cost_pub_ = create_publisher<PointCloud2>("cost_cloud", 1);
  goal_pub_ = create_publisher<PoseStamped>("goal", 1);
  plan_pub_ = create_publisher<FootstepPlan>("footstep_plan", 1);
  path_pubs_.push_back(create_publisher<Path>("body_path", 1));
  plan_service_ = create_service<Trigger>(
    "plan", std::bind(&FootstepPlannerNode::planCallback, this, _1, _2));
  simulate_service_ = create_service<SetString>(
    "simulate",
    std::bind(&FootstepPlannerNode::simulateCallback, this, _1, _2));
  RCLCPP_INFO(get_logger(), "Footstep planner node initialized");
}

void FootstepPlannerNode::descriptionCallback(const String::SharedPtr msg)
{
  KinematicsNode::descriptionCallback(msg);
  std::string error_message;
  if (!footstep_planner_->initialize(msg->data, error_message)) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to load robot description: %s", error_message.c_str());
  }
}

void FootstepPlannerNode::pointCloudCallback(
  const PointCloud2::SharedPtr msg)
{
  auto point_cloud = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(*msg, *point_cloud);
  Eigen::Isometry3d viewpoint = RosUtils::transformToEigen(
    lookupTransform("/map", "camera_link").transform);
  footstep_planner_->update(std::move(point_cloud), viewpoint);
}

void FootstepPlannerNode::imuCallback(const Imu::SharedPtr msg)
{
  Eigen::Isometry3d map_to_imu = RosUtils::transformToEigen(
    lookupTransform("/map", "/" + msg->header.frame_id).transform);
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
  auto body_transform = lookupTransform("/map", robot_->getBodyFrame());
  auto map_frame = body_transform.header.frame_id;
  start.pose = RosUtils::transformToEigen(body_transform.transform);
  start.joint_positions = robot_->getJointPosition();
  for (const auto & contact : robot_->getContactFrames()) {
    start.footholds[contact] = RosUtils::transformToEigen(
      lookupTransform(map_frame, contact).transform);
  }
  map_frame = getPrefixedFrameId(map_frame);
  auto plan = footstep_planner_->plan(start, goal_);

  PointCloud2 cost_msg;
  pcl::toROSMsg(*footstep_planner_->getCostCloud(), cost_msg);
  cost_msg.header.frame_id = map_frame;
  cost_msg.header.stamp = now();
  cost_pub_->publish(cost_msg);

  FootstepPlan plan_msg;
  plan_msg.header.frame_id = map_frame;

  std::unordered_map<std::string, Path> path_msgs;
  Path body_path_msg;
  body_path_msg.header.frame_id = map_frame;
  body_path_msg.header.stamp = now();
  Eigen::AngleAxisd flip(M_PI, Eigen::Vector3d::UnitZ());
  Eigen::Isometry3d flipped;
  for (const auto & stance : plan) {
    if (!stance.swing_foot.empty()) {
      Footstep footstep;
      footstep.header.frame_id = map_frame;
      footstep.header.stamp = now();
      footstep.body = RosUtils::eigenToPose(stance.pose);
      footstep.frames.push_back(stance.swing_foot);
      footstep.footholds.push_back(
        RosUtils::eigenToPose(stance.footholds.at(stance.swing_foot)));
      footstep.overrides.name.push_back("spine_joint");
      footstep.overrides.position.push_back(0.0);
      plan_msg.steps.push_back(footstep);
    }
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
      flipped = foothold;
      flipped.rotate(flip);
      ps.pose = RosUtils::eigenToPose(flipped);
      path_msgs[contact].poses.push_back(ps);
    }
  }
  plan_msg.header.stamp = now();
  plan_pub_->publish(plan_msg);
  auto contact_frames = robot_->getContactFrames();
  for (size_t i = 1; i < path_msgs.size() + 1; i++) {
    if (path_pubs_.size() == i) {
      path_pubs_.push_back(
        create_publisher<Path>("foothold_path_" + std::to_string(i), 1));
    }
    path_pubs_.at(i)->publish(path_msgs[contact_frames[i - 1]]);
  }
  path_pubs_.at(0)->publish(body_path_msg);
  response->success = true;
}

void FootstepPlannerNode::simulateCallback(
  const SetString::Request::SharedPtr request,
  SetString::Response::SharedPtr response)
{
  double res = 0.01;
  PointCloud::Ptr point_cloud = std::make_unique<PointCloud>();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d viewpoint = Eigen::Isometry3d::Identity();
  Eigen::AngleAxisd Ry = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY());
  goal_ = Eigen::Isometry3d::Identity();
  if (seed_) {
    std::srand(seed_);
  } else {
    std::srand(std::time(nullptr));
    int seed = std::rand() % 1000 + 1;
    std::srand(seed);
    RCLCPP_INFO(get_logger(), "Terrain generation seed: %d", seed);
  }
  if (request->data == "floor") {
    *point_cloud += terrain::planeXY({0.5, 0, 0}, 2, 2, res);
    viewpoint.translate(Eigen::Vector3d{0, 0, 1000});
    goal_.translate(Eigen::Vector3d{1, 0, 0});
  } else if (request->data == "sideways") {
    *point_cloud += terrain::planeXY({0, 0.5, 0}, 2, 2, res);
    viewpoint.translate(Eigen::Vector3d{0, 0, 1000});
    goal_.translate(Eigen::Vector3d{0, 1, 0});
  } else if (request->data == "diagonal") {
    *point_cloud += terrain::planeXY({0.5, 0.5, 0}, 2, 2, res);
    viewpoint.translate(Eigen::Vector3d{0, 0, 1000});
    goal_.translate(Eigen::Vector3d{1, 1, 0});
  } else if (request->data == "corner") {
    *point_cloud += terrain::planeXY({0, 0, 0}, 1, 1, res);
    *point_cloud += terrain::planeYZ({0.5, 0, 0.5}, 1, 1, res);
    viewpoint.translate(Eigen::Vector3d{-1000, 0, 1000});
    goal_.translate(Eigen::Vector3d{0.5, 0, 0.5});
    goal_.rotate(Ry.inverse());
  } else if (request->data == "wall") {
    *point_cloud += terrain::planeYZ({0, 0, 0.5}, 2, 2, res);
    transform.rotate(Ry.inverse());
    viewpoint.translate(Eigen::Vector3d{-1000, 0, 0});
    goal_.translate(Eigen::Vector3d{0, 0, 1});
    goal_.rotate(Ry.inverse());
  } else if (request->data == "corner2") {
    *point_cloud += terrain::planeYZ({0, 0, 0}, 1, 1, res);
    *point_cloud += terrain::planeXY({0.5, 0, 0.5}, 1, 1, res);
    transform.rotate(Ry.inverse());
    viewpoint.translate(Eigen::Vector3d{-1000, 0, 1000});
    goal_.translate(Eigen::Vector3d{0.5, 0, 0.5});
  } else if (request->data == "gap") {
    *point_cloud += terrain::planeXY({0, 0, 0}, 1, 1, res);
    *point_cloud += terrain::planeXY({1.1, 0, 0}, 1, 1, res);
    viewpoint.translate(Eigen::Vector3d{0, 0, 1000});
    goal_.translate(Eigen::Vector3d{1.1, 0, 0});
  } else if (request->data == "uneven") {
    *point_cloud += terrain::unevenXY({0.5, 0, 0}, 2, 2, 0.3, res);
    viewpoint.translate(Eigen::Vector3d{0, 0, 1000});
    goal_.translate(Eigen::Vector3d{1, 0, 0});
  } else if (request->data == "rocky") {
    *point_cloud += terrain::unevenXY({0.5, 0, 0}, 2, 2, 0.6, res);
    viewpoint.translate(Eigen::Vector3d{0, 0, 1000});
    goal_.translate(Eigen::Vector3d{1, 0, 0});
  } else if (request->data == "slag") {
    *point_cloud += terrain::unevenYZ({0, 0, 0.5}, 2, 2, 0.15, res, 0.5);
    transform.rotate(Ry.inverse());
    viewpoint.translate(Eigen::Vector3d{-1000, 0, 0});
    goal_.translate(Eigen::Vector3d{0, 0, 1});
    goal_.rotate(Ry.inverse());
  } else if (request->data == "tufa") {
    *point_cloud += terrain::unevenYZ({0, 0, 0.5}, 2, 2, 0.1, res, 0);
    transform.rotate(Ry.inverse());
    viewpoint.translate(Eigen::Vector3d{-1000, 0, 0});
    goal_.translate(Eigen::Vector3d{0, 0, 1});
    goal_.rotate(Ry.inverse());
  } else if (request->data == "cliff") {
    Eigen::Isometry3d cliff = Eigen::Isometry3d::Identity();
    cliff.rotate(Ry.inverse());
    cliff.translate(Eigen::Vector3d{0.5, 0, 0});
    *point_cloud += terrain::uneven(cliff, 2, 2, 0.3, res);
    transform.rotate(Ry.inverse());
    viewpoint.translate(Eigen::Vector3d{-1000, 0, 0});
    goal_.translate(Eigen::Vector3d{0, 0, 1});
    goal_.rotate(Ry.inverse());
  } else {
    response->success = false;
    response->message = "Environment not found";
    return;
  }

  // Project goal onto surface
  auto kdtree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>(false);
  kdtree->setInputCloud(point_cloud);
  Eigen::Vector3f goal = goal_.translation().cast<float>();
  std::vector<int> indices = {0};
  std::vector<float> distances = {0};
  kdtree->nearestKSearch({goal(0), goal(1), goal(2)}, 1, indices, distances);
  goal_.translation() = point_cloud->points[indices[0]].getVector3fMap().cast<double>();

  double offset = 0;
  int count = 0;
  for (const auto & contact : robot_->getContactFrames()) {
    if (robot_->getContactType(contact) != ContactType::TAIL) {
      offset += robot_->getTransform(contact).translation()(2);
      ++count;
    }
  }
  offset /= count;
  transform = transform * Eigen::Translation3d(0, 0, -offset);

  PoseStamped goal_msg;
  goal_msg.header.frame_id = "/map";
  goal_msg.pose = RosUtils::eigenToPose(goal_);
  goal_msg.header.stamp = now();
  goal_pub_->publish(goal_msg);

  PointCloud2 cloud_msg;
  pcl::toROSMsg(*point_cloud, cloud_msg);
  cloud_msg.header.frame_id = getPrefixedFrameId("/map");
  cloud_msg.header.stamp = now();
  cost_pub_->publish(cloud_msg);
  footstep_planner_->update(std::move(point_cloud), viewpoint);

  Eigen::Isometry3d odomToBody =
    RosUtils::transformToEigen(
    lookupTransform("/odom", robot_->getBodyFrame()).transform);
  Eigen::Isometry3d mapToOdom = transform * odomToBody.inverse();
  TransformStamped transform_msg;
  transform_msg.header.frame_id = "/map";
  transform_msg.child_frame_id = "/odom";
  transform_msg.transform = RosUtils::eigenToTransform(mapToOdom);
  transform_msg.header.stamp = now();
  sendTransform(transform_msg);
  response->success = true;
}

rcl_interfaces::msg::SetParametersResult FootstepPlannerNode::parameterCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  auto result = KinematicsNode::parameterCallback(parameters);
  for (const auto & param : parameters) {
    if (param.get_name() == "seed") {
      seed_ = param.as_int();
    }
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
