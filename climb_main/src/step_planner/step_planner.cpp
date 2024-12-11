#include "climb_main/step_planner/step_planner.hpp"
#include "climb_main/util/eigen_utils.hpp"
#include "climb_main/util/ros_utils.hpp"

using geometry_msgs::msg::Twist;
using geometry_msgs::msg::Wrench;

StepPlanner::StepPlanner(std::shared_ptr<KinematicsInterface> robot)
: robot_(robot), transform_(Eigen::Isometry3d::Identity())
{
}

void StepPlanner::step(
  const std::string & contact, const Eigen::Isometry3d & foothold)
{
  footsteps_[contact] =
  {State::DISENGAGE, transform_ * robot_->getTransform(contact),
    foothold, t_, false};
}

void StepPlanner::cancel(const std::string & contact)
{
  if (contact.empty()) {
    footsteps_.clear();
  } else if (footsteps_.find(contact) != footsteps_.end()) {
    footsteps_[contact].state = State::STOP;
  }
}

void StepPlanner::advance(const std::string & contact)
{
  if (footsteps_.find(contact) == footsteps_.end()) {return;}
  switch (footsteps_[contact].state) {
    case State::DISENGAGE:
      footsteps_[contact].state = State::LIFT;
      break;
    case State::LIFT:
      footsteps_[contact].state = State::SWING;
      break;
    case State::SWING:
      footsteps_[contact].state = State::PLACE;
      break;
    case State::PLACE:
      footsteps_[contact].state = State::ENGAGE;
      break;
    case State::ENGAGE:
      footsteps_[contact].state = State::STANCE;
      break;
    default:
      break;
  }
}

void StepPlanner::moveFoothold(
  const std::string & contact, const Eigen::Vector<double, 6> & twist)
{
  if (footsteps_.find(contact) != footsteps_.end()) {
    EigenUtils::applyTwistInPlace(footsteps_.at(contact).foothold, twist);
  }
}

std::unordered_map<std::string, StepPlanner::State> StepPlanner::update(
  const ContactForce & forces, const TransformStamped & transform)
{
  std::unordered_map<std::string, State> new_states;
  t_ = forces.header.stamp;
  if (transform.child_frame_id == robot_->getBodyFrame()) {
    transform_ = RosUtils::transformToEigen(transform.transform);
  } else {
    throw std::invalid_argument(
            "Invalid transform: expected child frame " + robot_->getBodyFrame() +
            " but received " + transform.child_frame_id);
  }
  State initial_state;
  int index;
  Eigen::Matrix<double, 6, Eigen::Dynamic> basis;
  Eigen::Isometry3d origin;
  Eigen::Isometry3d foothold;
  Eigen::Isometry3d ee_pose;
  Eigen::Vector<double, 6> force;
  Eigen::VectorXd displacement;
  std::vector<std::string> remove;
  for (auto & [contact, footstep] : footsteps_) {
    if (footstep.paused) {continue;}
    index = std::distance(
      forces.frame.begin(),
      std::find(forces.frame.begin(), forces.frame.end(), contact));
    force = RosUtils::wrenchToEigen(forces.wrench[index]);
    basis = robot_->getWrenchBasis(contact);  // TODO rotate to body frame
    origin = transform_.inverse() * footstep.origin;
    foothold = transform_.inverse() * footstep.foothold;
    ee_pose = robot_->getTransform(contact);
    initial_state = footstep.state;
    switch (footstep.state) {
      case State::STANCE:     // Wait for disengage command
        displacement =
          basis.transpose() * EigenUtils::getTwist(origin, ee_pose);
        if (displacement.norm() > retry_distance_) {
          footstep.state = State::SLIP;
        }
        break;
      case State::DISENGAGE:  // Disengage foot until contact is lost
        if (force.norm() < disengage_tolerance_) {
          footstep.state = State::LIFT;
        }
        break;
      case State::LIFT:       // Raise foot above step height (in body frame)
        displacement = EigenUtils::getTwist(origin, ee_pose);
        if (displacement(2) > step_height_) {
          footstep.state = State::SWING;
        } else if (force.norm() > snag_threshold_) {
          footstep.state = State::SNAG;
        }
        break;
      case State::SWING:      // Move foot over next foothold (in body frame)
        displacement = EigenUtils::getTwist(ee_pose, foothold);
        displacement(2) = 0;
        displacement = basis.transpose() * displacement;
        if (displacement.norm() < foothold_tolerance_) {
          footstep.state = State::PLACE;
        } else if (force.norm() > snag_threshold_) {
          footstep.state = State::SNAG;
        }
        break;
      case State::PLACE:      // Lower foot until contact is detected
        if (force[0] > contact_threshold_) {
          footstep.state = State::ENGAGE;
          footstep.foothold = transform_ * ee_pose;
        }
        break;
      case State::ENGAGE:     // Engage foot until force is reached
        displacement = basis.transpose() * EigenUtils::getTwist(ee_pose, foothold);
        if ((force - engage_force_).norm() < engage_tolerance_) {
          footstep.state = State::STANCE;
          footstep.foothold = transform_ * ee_pose;
        } else if (displacement.norm() > retry_distance_) {
          footstep.state = State::RETRY;
        }
        break;
      case State::STOP:       // Freeze end effector in place
        break;
      default:                // Return to disengage state on failure
        footstep.state = State::DISENGAGE;
        break;
    }
    if (footstep.state != initial_state) {
      footstep.t0 = t_;
      new_states[contact] = footstep.state;
      if (footstep.state == State::STANCE) {
        remove.push_back(contact);
      }
    }
  }
  for (const auto & contact : remove) {
    footsteps_.erase(contact);
  }
  return new_states;
}

EndEffectorCommand StepPlanner::getCommand() const
{
  EndEffectorCommand cmd;
  Eigen::Vector<double, 6> vel;
  double theta;
  for (const auto & [contact, footstep] : footsteps_) {
    if (footstep.paused) {
      continue;
    }
    cmd.frame.push_back(contact);
    switch (footstep.state) {
      case State::STANCE:
        break;
      case State::DISENGAGE:
        cmd.mode.push_back(EndEffectorCommand::MODE_TRANSITION);
        cmd.twist.push_back(Twist());
        vel = disengage_force_;
        theta = (t_ - footstep.t0).seconds() * 2 * M_PI * disengage_frequency_;
        vel(0) *= -std::cos(2 * theta);
        vel(1) *= std::sin(theta);
        vel(2) *= std::abs(std::sin(theta));
        cmd.wrench.push_back(RosUtils::eigenToWrench(vel));
        break;
      case State::LIFT:
        cmd.mode.push_back(EndEffectorCommand::MODE_FREE);
        vel = {-swing_velocity_, 0, 0, 0, 0, 0};
        cmd.twist.push_back(RosUtils::eigenToTwist(vel));
        cmd.wrench.push_back(Wrench());
        break;
      case State::SWING:
        cmd.mode.push_back(EndEffectorCommand::MODE_FREE);
        vel = EigenUtils::getTwist(
          Eigen::Isometry3d::Identity(),
          robot_->getTransform(contact).inverse() *
          EigenUtils::getTransform({0, 0, step_height_, 0, 0, 0}) *
          transform_.inverse() * footstep.foothold,
          swing_velocity_);
        cmd.twist.push_back(RosUtils::eigenToTwist(vel));
        cmd.wrench.push_back(Wrench());
        break;
      case State::PLACE:
        cmd.mode.push_back(EndEffectorCommand::MODE_FREE);
        vel = {swing_velocity_, 0, 0, 0, 0, 0};
        cmd.twist.push_back(RosUtils::eigenToTwist(vel));
        cmd.wrench.push_back(Wrench());
        break;
      case State::ENGAGE:
        cmd.mode.push_back(EndEffectorCommand::MODE_TRANSITION);
        cmd.twist.push_back(Twist());
        cmd.wrench.push_back(RosUtils::eigenToWrench(engage_force_));
        break;
      case State::SNAG:
        break;
      case State::RETRY:
        break;
      case State::SLIP:
        break;
      case State::STOP:
        cmd.mode.push_back(EndEffectorCommand::MODE_FREE);
        cmd.twist.push_back(Twist());
        cmd.wrench.push_back(Wrench());
        break;
    }
  }
  return cmd;
}

Eigen::Isometry3d StepPlanner::getNominalFoothold(const std::string & contact) const
{
  return transform_ * EigenUtils::applyTwist(
    robot_->getTransform(contact),
    Eigen::Vector<double, 6>{step_length_, 0, 0, 0, 0, 0});
}

StepPlanner::State StepPlanner::getState(const std::string & contact) const
{
  if (footsteps_.find(contact) == footsteps_.end()) {
    return State::STANCE;
  }
  return footsteps_.at(contact).state;
}

void StepPlanner::declareParameters()
{
  declareParameter(
    "step_height", 0.05,
    "Height of the swing end effector above the surface");
  declareParameter(
    "step_length", 0.5,
    "Nominal distance between footholds in m");
  declareParameter(
    "swing_velocity", 0.5,
    "Velocity of the end effector during swing in m/s");
  declareParameter(
    "engage_force", std::vector<double>(),
    "Engagement force in contact frame wrench basis in N");
  declareParameter(
    "engage_tolerance", 3.0,
    "Tolerance for engagement force error in N", 0.0);
  declareParameter(
    "disengage_force", std::vector<double>(),
    "Disengagement force in contact frame wrench basis in N");
  declareParameter(
    "disengage_frequency", 0.0,
    "Frequency of lateral oscillation during disengagement in Hz", 0.0);
  declareParameter(
    "disengage_tolerance", 3.0,
    "Tolerance for engagement force error in N", 0.0);
  declareParameter(
    "contact_threshold", 1.0,
    "Threshold for contact detection in N", 0.0);
  declareParameter(
    "snag_threshold", 1.0,
    "Threshold for snag detection in N", 0.0);
  declareParameter(
    "foothold_tolerance", 0.1,
    "Tolerance for foothold position error in m", 0.0);
  declareParameter(
    "retry_distance", 0.0,
    "Maximum displacement before retrying foothold in m", 0.0);
  declareParameter(
    "angular_scaling", 1.0,
    "Scaling factor for angular components in rad/m and N/Nm", 0.0);
}

void StepPlanner::setParameter(
  const Parameter & param, SetParametersResult & result)
{
  if (param.get_name() == "step_height") {
    step_height_ = param.as_double();
  } else if (param.get_name() == "step_length") {
    step_length_ = param.as_double();
  } else if (param.get_name() == "swing_velocity") {
    swing_velocity_ = param.as_double();
  } else if (param.get_name() == "engage_force") {
    std::vector<double> force = param.as_double_array();
    if (force.size() == 0) {
      engage_force_ = Eigen::Vector<double, 6>::Zero();
    } else if (force.size() == 6) {
      engage_force_ = RosUtils::vectorToEigen(force);
    } else {
      result.successful = false;
      result.reason = "Invalid wrench size (expected 0 or 6)";
    }
  } else if (param.get_name() == "engage_tolerance") {
    engage_tolerance_ = param.as_double();
  } else if (param.get_name() == "disengage_force") {
    std::vector<double> force = param.as_double_array();
    if (force.size() == 0) {
      disengage_force_ = Eigen::Vector<double, 6>::Zero();
    } else if (force.size() == 6) {
      disengage_force_ = RosUtils::vectorToEigen(force);
    } else {
      result.successful = false;
      result.reason = "Invalid wrench size (expected 0 or 6)";
    }
  } else if (param.get_name() == "disengage_frequency") {
    disengage_frequency_ = param.as_double();
  } else if (param.get_name() == "disengage_tolerance") {
    disengage_tolerance_ = param.as_double();
  } else if (param.get_name() == "contact_threshold") {
    contact_threshold_ = param.as_double();
  } else if (param.get_name() == "snag_threshold") {
    snag_threshold_ = param.as_double();
  } else if (param.get_name() == "foothold_tolerance") {
    foothold_tolerance_ = param.as_double();
  } else if (param.get_name() == "retry_distance") {
    retry_distance_ = param.as_double();
  } else if (param.get_name() == "angular_scaling") {
    angular_scaling_ = param.as_double();
  } else {
    result.successful = false;
    result.reason = "Parameter not found";
  }
}
