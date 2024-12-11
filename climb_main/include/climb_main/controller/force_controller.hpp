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
   * @brief Linear constraint of the form: A x <= b
   */
  struct Constraint
  {
    Eigen::MatrixXd A;  // Constraint matrix
    Eigen::VectorXd b;  // Constraint vector
  };

  /**
   * @brief Constructor for ForceController
   * @param[in] robot Kinematics interface for the robot
   */
  ForceController(std::shared_ptr<KinematicsInterface> robot);

  /**
   * @brief Reset the controller setpoints
   */
  void reset();

  /**
   * @brief Update the contact forces based on the given force command
   * @param[in] force Current contact force estimates (robot on world)
   * @return True if an optimal solution was found
   */
  bool update(const Eigen::VectorXd & force);

  /**
   * @brief Set the desired behavior of each end-effector
   * @param[in] command Commanded end-effector behavior
   */
  void setEndEffectorCommand(const EndEffectorCommand & command);

  /**
   * @brief Set the obstacle constraint to maintain clearance between the body
   * frame and the estimated ground plane.
   * @param[in] normal Normal vector of the ground plane in the body frame
   * @param[in] distance Distance from the ground plane to the body frame
   */
  void setGroundConstraint(const Eigen::Vector3d & normal, double distance);

  /**
   * @brief Set the obstacle constraints for the controller
   * @param[in] frames Names of the link frames at risk of collision
   * @param[in] normals Unit vectors from links to obstacles in the body frame
   * @param[in] distances Signed distances from links to obstacles
   */
  void setObstacleConstraints(
    const std::vector<std::string> & frames,
    const std::vector<Eigen::Vector3d> & normals,
    const std::vector<double> & distances);

  /**
   * @brief Set the nominal joint configuration for the controller
   * @param[in] configuration Nominal joint configuration
   */
  void setNominalConfiguration(const Eigen::VectorXd & configuration)
  {configuration_ = configuration;}

  /**
   * @brief Get the adhesion constraint for a given contact force
   * @details Coordinate system: X is normal (pressing against surface), Y is
   * lateral (perpendicular to preferred gripper loading direction), and Z is
   * axial (along preferred gripper loading direction)
   * @param[in] frame Name of the end-effector frame
   * @return Adhesion constraint for the end-effector's contact force
   * @throws std::invalid_argument if the contact type is not supported
   */
  Constraint getAdhesionConstraint(const std::string & frame) const;

  /**
   * @brief Get the optimal joint positions
   * @return Vector of joint positions (rad or m)
   */
  Eigen::VectorXd getJointPosition() const {return position_cmd_;}

  /**
   * @brief Get the optimal joint displacements
   * @return Vector of joint displacements (rad or m)
   */
  Eigen::VectorXd getJointDisplacement() const
  {
    return displacement_cmd_.head(position_cmd_.size());
  }

  /**
   * @brief Get the optimal body displacements
   * @return Vector of body displacements (m or rad)
   */
  Eigen::VectorXd getBodyDisplacement() const
  {
    return displacement_cmd_.tail(
      displacement_cmd_.size() - position_cmd_.size());
  }

  /**
   * @brief Get the optimal joint efforts
   * @return Vector of joint efforts (Nm or N)
   */
  Eigen::VectorXd getJointEffort() const {return effort_cmd_;}

  /**
   * @brief Get the optimal contact forces
   * @return Vector of robot-on-world contact forces (N)
   */
  Eigen::VectorXd getContactForce() const {return force_cmd_;}

  /**
   * @brief Get the stability margin of the optimal solution
   * @return Stability margin of the optimal solution
   */
  double getMargin() const {return margin_;}

  /**
   * @brief Get the maximum tracking error of the optimal solution
   * @return Maximum tracking error of the optimal solution
   */
  double getError() const {return error_;}

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
    // Unit vector from link to obstacle in the body frame
    Eigen::Vector3d normal;
    // Distance from link to obstacle in the body frame
    double distance;
  };

  // Robot kinematics interface
  std::shared_ptr<KinematicsInterface> robot_;
  // QP solver
  std::unique_ptr<QpInterface> solver_;
  // Commanded goal for each end-effector
  std::unordered_map<std::string, EndEffectorGoal> end_effector_goals_;
  // Obstacle constraints
  std::vector<ObstacleConstraint> obstacles_;
  // Nominal joint configuration
  Eigen::VectorXd configuration_;
  // Commanded joint and body displacements
  Eigen::VectorXd displacement_cmd_;
  // Commanded joint positions
  Eigen::VectorXd position_cmd_;
  // Commanded joint efforts
  Eigen::VectorXd effort_cmd_;
  // Commanded contact forces of end effectors on world
  Eigen::VectorXd force_cmd_;
  // Stability margin of the optimal solution
  double margin_;
  // Maximum tracking error of the optimal solution
  double error_;
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
  // Minimum link mass for center of mass optimization in kg
  double mass_threshold_;
  // Maximum out-of-plane loading angle of microspine gripper in rad
  double microspine_pitch_angle_;
  // Maximum in-plane loading angle of microspine gripper in rad
  double microspine_yaw_angle_;
  // End effector coefficient of friction
  double friction_coefficient_;
  // Minimum tangential force on microspine gripper in N
  double microspine_min_force_;
  // Maximum tangential force on microspine gripper in N
  double microspine_max_force_;
  // Maximum joint effort in Nm
  double max_effort_;
};

#endif  // FORCE_CONTROLLER_HPP
