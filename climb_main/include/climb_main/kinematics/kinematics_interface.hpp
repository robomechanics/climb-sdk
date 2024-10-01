#ifndef KINEMATICS_INTERFACE_H
#define KINEMATICS_INTERFACE_H

#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "climb_main/util/parameterized.hpp"

using sensor_msgs::msg::JointState;
using geometry_msgs::msg::TransformStamped;

/**
 * @brief Abstract interface for a kinematics library
 */
class KinematicsInterface : public Parameterized
{
public:
  enum ContactType { microspine, magnet };

  /**
   * @brief Constructor for KinematicsInterface
   * @return Constructed object of type KinematicsInterface
   */
  KinematicsInterface();

  /**
   * @brief Default destructor for KinematicsInterface
   */
  virtual ~KinematicsInterface() = default;

  /**
   * @brief Load robot description from a URDF string
   * @param[in] description robot description in URDF format
   * @return True if the robot description was successfully loaded
   */
  virtual bool loadRobotDescription(std::string description) = 0;

  /**
   * @brief Compute Jacobian mapping from joint velocities to contact velocities
   * @return Jacobian matrix of size num_constraints x num_joints
   *
   * Contact velocity is the velocity of the contact frame relative to the base
   * frame along constrained directions, measured in the contact frame.
   */
  virtual Eigen::MatrixXd getJacobian() = 0;

  /**
   * @brief Compute grasp map mapping from contact forces to body wrench
   * @return Grasp map matrix of size 6 x num_constraints
   *
   * Contact force is the force of the world on the robot at the contact along
   * constrained directions, measured in the contact frame.
   */
  virtual Eigen::MatrixXd getGraspMap() = 0;

  /**
   * @brief Convert std::vector to Eigen::VectorXd ordered by joint
   * @param[in] values std::vector of values for each joint
   * @param[in] joints std::vector of corresponding joint names [optional]
   * @return The corresponding Eigen vector ordered by joint
   */
  Eigen::VectorXd vectorToEigen(
    const std::vector<double> & values,
    const std::vector<std::string> & joints = std::vector<std::string>()
  );

  /**
   * @brief Convert Eigen::VectorXd to std::vector
   * @param[in] vector Eigen::VectorXd of values
   * @return The corresponding std::vector<double> with same ordering
   */
  std::vector<double> eigenToVector(const Eigen::VectorXd & vector);

  /**
   * @brief Update joint states with latest data from robot
   * @param[in] states Joint states
   */
  void updateJointState(const JointState & states);

  /**
   * @brief Update contact frames with latest data from robot
   * @param[in] transforms Transforms from end-effector frames to contact frames
   */
  void updateContactFrames(const std::vector<TransformStamped> & transforms);

  // Robot configuration
  bool initialized = false;                 // Has robot description been loaded
  std::string urdf_path;                    // Path to robot description file
  std::vector<std::string> joints;          // Joint names
  std::string body_frame;                   // Body frame name
  std::vector<std::string> contact_frames;  // Contact frame names
  std::vector<ContactType> contact_types;   // Contact frame types
  int num_joints;                           // Number of joints
  int num_contacts;                         // Number of contacts
  int num_constraints;                      // Number of contact constraints

  // Joint state
  Eigen::VectorXd joint_pos;                // Joint positions (rad or m)
  Eigen::VectorXd joint_vel;                // Joint velocities (rad/s or m/s)
  Eigen::VectorXd joint_eff;                // Joint efforts (Nm or N)

  // Joint limits
  Eigen::VectorXd joint_pos_min;  // Joint position lower bounds (rad or m)
  Eigen::VectorXd joint_pos_max;  // Joint position upper bounds (rad or m)
  Eigen::VectorXd joint_vel_min;  // Joint velocity lower bounds (rad/s or m/s)
  Eigen::VectorXd joint_vel_max;  // Joint velocity upper bounds (rad/s or m/s)
  Eigen::VectorXd joint_eff_min;  // Joint effort lower bounds (Nm or N)
  Eigen::VectorXd joint_eff_max;  // Joint effort upper bounds (Nm or N)
};

#endif  // KINEMATICS_INTERFACE_H
