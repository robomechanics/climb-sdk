#ifndef FORCE_CONTROLLER_HPP
#define FORCE_CONTROLLER_HPP

#include "climb_msgs/msg/end_effector_command.hpp"

#include "climb_main/util/parameterized.hpp"
#include "climb_main/kinematics/kinematics_interface.hpp"
#include "climb_main/optimization/qp_interface.hpp"

using climb_msgs::msg::EndEffectorCommand;

class ForceController : public Parameterized
{
public:
  /**
   * @brief Constructor for ForceController
   * @param[in] robot Kinematics interface for the robot
   */
  ForceController(std::shared_ptr<KinematicsInterface> robot);

  /**
   * @brief Update the contact forces based on the given force command
   * @param[in] force Current contact force estimates
   * @return Vector of commanded joint positions
   */
  Eigen::VectorXd update(const Eigen::VectorXd & force);

  /**
   * @brief Set the desired behavior of each end-effector
   * @param[in] command Commanded end-effector behavior
   */
  void setEndEffectorCommand(const EndEffectorCommand & command);

  /**
   * @brief Set the obstacle constraints for the controller
   * @param[in] obstacles List of obstacles to avoid
   */
  void setObstacleConstraints(
    const std::vector<std::string> & frames,
    const std::vector<Eigen::Vector3d> & displacements);

  void declareParameters() override;

  void setParameter(
    const Parameter & param, SetParametersResult & result) override;

  using Parameterized::setParameter;

private:
  // Commanded mode and setpoint for an end-effector
  struct EndEffectorGoal
  {
    // Contact mode
    int mode;
    // Setpoint twist or wrench in rad/s, m/s, Nm, or N
    Eigen::Vector<double, 6> setpoint;
  };

  struct ObstacleConstraint
  {
    // Name of the link at risk of collision
    std::string frame;
    // Displacement vector from link to obstacle in the body frame
    Eigen::Vector3d displacement;
  };

  // Robot kinematics interface
  std::shared_ptr<KinematicsInterface> robot_;
  // QP solver
  std::unique_ptr<QpInterface> solver_;
  // Commanded goal for each end-effector
  std::unordered_map<std::string, EndEffectorGoal> end_effector_goals_;
  // Obstacle constraints
  std::vector<ObstacleConstraint> obstacles_;
  // Expected compliance at the point of contact in N/m
  double stiffness_;
  // Maximum joint displacement
  double joint_step_;
  // Minimum height of body frame origin above ground plane
  double clearance_;
  // Relative cost of joint displacement in 1/N^2
  double normalization_;
  // Relative cost of end-effector error in 1/N
  double tracking_;
};

#endif  // FORCE_CONTROLLER_HPP
