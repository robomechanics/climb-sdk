#ifndef CLIMB_GAIT_PLANNER__GAIT_PLANNER_HPP_
#define CLIMB_GAIT_PLANNER__GAIT_PLANNER_HPP_

#include <Eigen/Geometry>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <climb_msgs/msg/contact_force.hpp>
#include <climb_msgs/msg/controller_command.hpp>
#include <climb_msgs/action/footstep_command.hpp>
#include <climb_kinematics/interfaces/kinematics_interface.hpp>
#include <climb_util/parameterized.hpp>

using geometry_msgs::msg::TransformStamped;
using climb_msgs::msg::ContactForce;
using climb_msgs::msg::ControllerCommand;
using climb_msgs::action::FootstepCommand;

/**
 * @brief State-machine for executing a planned step
 */
class GaitPlanner : public Parameterized
{
public:
  enum class State : uint8_t
  {
    // Nominal sequence
    STANCE = FootstepCommand::Feedback::STATE_STANCE,
    DISENGAGE = FootstepCommand::Feedback::STATE_DISENGAGE,
    LIFT = FootstepCommand::Feedback::STATE_LIFT,
    SWING = FootstepCommand::Feedback::STATE_SWING,
    PLACE = FootstepCommand::Feedback::STATE_PLACE,
    ENGAGE = FootstepCommand::Feedback::STATE_ENGAGE,
    // Error states
    SNAG = FootstepCommand::Feedback::STATE_SNAG,
    RETRY = FootstepCommand::Feedback::STATE_RETRY,
    SLIP = FootstepCommand::Feedback::STATE_SLIP,
    STOP = FootstepCommand::Feedback::STATE_STOP
  };

  struct Footstep
  {
    State state;
    Eigen::Isometry3d origin;
    Eigen::Isometry3d foothold;
    rclcpp::Time t0;
    bool paused;
  };

  struct StateChange
  {
    rclcpp::Time t;
    std::string contact;
    State state;
  };

  /**
   * @brief Constructor for GaitPlanner
   * @param[in] robot Kinematics interface for the robot
   * @return Constructed object of type GaitPlanner
   */
  explicit GaitPlanner(std::shared_ptr<KinematicsInterface> robot);

  /**
   * @brief Reset the state machine
   */
  void reset();

  /**
   * @brief Begin a step to the specified foothold
   * @param[in] contact Name of the contact frame taking the step
   * @param[in] foothold Desired foothold in the map frame
   */
  void step(const std::string & contact, const Eigen::Isometry3d & foothold);

  /**
   * @brief Begin a step to the nominal foothold
   * @param[in] contact Name of the contact frame taking the step
   */
  void step(const std::string & contact)
  {step(contact, getNominalFoothold(contact));}

  /**
   * @brief Cancel the current step
   * @param[in] contact Name of the contact frame taking the step
   */
  void cancel(const std::string & contact);

  /**
   * @brief Pause the current step
   * @param[in] contact Name of the contact frame taking the step
   */
  void pause(const std::string & contact)
  {
    if (!footsteps_.at(contact).paused) {
      footsteps_.at(contact).paused = true;
      change_queue_.push({t_, contact, State::STOP});
    }
  }

  /**
   * @brief Resume the current step
   * @param[in] contact Name of the contact frame taking the step
   */
  void resume(const std::string & contact)
  {
    if (footsteps_.at(contact).paused) {
      footsteps_.at(contact).paused = false;
      change_queue_.push({t_, contact, footsteps_.at(contact).state});
    }
  }

  /**
   * @brief Advance the current step to the next state
   * @param[in] contact Name of the contact frame taking the step
   */
  void advance(const std::string & contact);

  /**
   * @brief Retry the current step
   * @param[in] contact Name of the contact frame taking the step
   */
  void retry(const std::string & contact)
  {
    footsteps_.at(contact).state = State::RETRY;
    change_queue_.push({t_, contact, footsteps_.at(contact).state});
  }

  /**
   * @brief Move the foothold by the specified twist
   * @param[in] contact Name of the contact frame
   * @param[in] twist Twist in the map frame
   */
  void moveFoothold(
    const std::string & contact, const Eigen::Vector<double, 6> & twist);

  /**
   * @brief Update the state machine based on the latest contact forces
   * @param[in] forces Message containing contact forces
   * @param[in] transform Message containing transform from map to body frame
   * @return Map from contact frames that changed state to their new states
   */
  void update(const ContactForce & forces, const TransformStamped & transform);

  /**
   * @brief Get all state changes since the last query and clear the queue
   * @return Queue of state changes
   */
  std::queue<StateChange> getStateChanges();

  /**
   * @brief Update the transform from the map to the robot base frame
   * @param[in] transform Transform from map to robot base frame
   */
  void updateTransform(const Eigen::Isometry3d & transform)
  {transform_ = transform;}

  /**
   * @brief Get the current end effector command
   * @return End effector command ROS message
   */
  ControllerCommand getCommand();

  /**
   * @brief Get the nominal foothold for the specified contact
   * @param[in] contact Name of the contact frame
   * @return Nominal foothold in the map frame
   */
  Eigen::Isometry3d getNominalFoothold(const std::string & contact) const;

  /**
   * @brief Get the current state of the state machine
   * @param[in] contact Name of the contact frame
   * @return Current state
   */
  State getState(const std::string & contact) const;

  /**
   * @brief Get the current foothold for the specified contact
   * @param[in] contact Name of the contact frame
   * @return Current foothold in the map frame
   */
  Eigen::Isometry3d getFoothold(const std::string & contact) const
  {return footsteps_.at(contact).foothold;}

  void declareParameters() override;

  void setParameter(
    const Parameter & param, SetParametersResult & result) override;

  using Parameterized::setParameter;

private:
  // Robot kinematics interface
  std::shared_ptr<KinematicsInterface> robot_;
  // Transform from map to robot base frame
  Eigen::Isometry3d transform_;
  // Footstep states
  std::unordered_map<std::string, Footstep> footsteps_;
  // Time of last joint state update
  rclcpp::Time t_;
  // Time of last timeout
  rclcpp::Time t0_;
  // Queue of state changes
  std::queue<StateChange> change_queue_;
  // Parameters
  bool auto_advance_;
  double step_height_;
  double step_length_;
  double swing_velocity_;
  Eigen::VectorXd engage_force_;
  double engage_tolerance_;
  Eigen::VectorXd disengage_force_;
  double disengage_frequency_;
  double disengage_tolerance_;
  double contact_threshold_;
  double snag_threshold_;
  double foothold_tolerance_;
  double retry_distance_;
  double angular_scaling_;
  double lift_timeout_;
  double foothold_offset_;
};

#endif  // CLIMB_GAIT_PLANNER__GAIT_PLANNER_HPP_
