#ifndef KINEMATICS_INTERFACE_HPP
#define KINEMATICS_INTERFACE_HPP

#include <unordered_map>
#include <vector>
#include <string>
#include <Eigen/Geometry>
#include <urdf/model.h>
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
  /**
   * @brief Contact type between end-effector and the environment
   */
  enum ContactType { DEFAULT, MICROSPINE, TAIL, MAGNET_WHEEL, FRICTION };

  /**
   * @brief Wrist type for end-effector
   */
  enum WristType { FIXED, FREE, SPRING, GRAVITY };

  /**
   * @brief Mapping of contact types to wrench basis specifications
   * Axes are ordered as [Fx, Fy, Fz, Tx, Ty, Tz]
   * A value of 1 indicates the axis is included in the basis
   */
  const std::map<ContactType, std::vector<int>> WRENCH_BASES = {
    {MICROSPINE, {1, 1, 1, 0, 0, 0}},
    {TAIL, {1, 0, 0, 0, 0, 0}},
    {MAGNET_WHEEL, {1, 1, 1, 0, 0, 0}},
    {DEFAULT, {1, 1, 1, 1, 1, 1}},
    {FRICTION, {1, 1, 1, 0, 0, 0}}
  };

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
   * @param[out] error_message Error message if initialization fails
   * @return True if the robot description was successfully loaded
   */
  bool loadRobotDescription(
    const std::string & description, std::string & error_message);

  /**
   * @brief Get transform from parent frame to child frame
   * @param[in] parent Name of parent frame
   * @param[in] child Name of child frame
   * @return Pair of position and rotation matrix of child frame in parent frame
   */
  virtual std::pair<Eigen::Vector3d, Eigen::Matrix3d> getTransform(
    const std::string & parent, const std::string & child) = 0;

  /**
   * @brief Get transform from body frame to child frame
   * @param[in] child Name of child frame
   * @return Pair of position and rotation matrix of child frame in parent frame
   */
  virtual std::pair<Eigen::Vector3d, Eigen::Matrix3d> getTransform(
    const std::string & child) {return getTransform(body_frame_, child);}

  /**
   * @brief Compute Jacobian mapping joint velocities to end effector
   * velocity with respect to the body in the contact wrench basis
   * @param[in] contact_frame Name of contact frame
   * @return Jacobian matrix of size cols(wrench basis) x num_joints
   */
  virtual Eigen::MatrixXd getHandJacobian(
    const std::string & contact_frame) = 0;

  /**
   * @brief Compute Jacobian mapping joint velocities to end effector
   * velocities with respect to the body in the contact wrench basis
   * @return Jacobian matrix of size num_constraints x num_joints
   */
  virtual Eigen::MatrixXd getHandJacobian();

  /**
   * @brief Compute Jacobian mapping joint velocities to child frame
   * velocity with respect to the parent frame in the parent frame at the
   * origin of the child frame
   * @param[in] parent Name of parent frame
   * @param[in] child Name of child frame
   * @param[in] linear True for linear axes, false for full twist
   * @return Jacobian matrix of size 3 x num_joints or 6 x num_joints
   */
  virtual Eigen::MatrixXd getJacobian(
    const std::string & parent, const std::string & child,
    bool linear = false) = 0;

  /**
   * @brief Compute Jacobian mapping joint velocities to child frame
   * velocity with respect to the body frame in the body frame at the
   * origin of the child frame
   * @param[in] child Name of child frame
   * @param[in] linear True for linear axes, false for full twist
   * @return Jacobian matrix of size 3 x num_joints or 6 x num_joints
   **/
  Eigen::MatrixXd getJacobian(const std::string & child, bool linear = false)
  {
    return getJacobian(body_frame_, child, linear);
  }

  /**
   * @brief Compute Jacobian mapping joint velocities to end effector
   * velocities with respect to the body in the body frame at the origins of the
   * contact frames
   * @param[in] linear True for linear axes only, false for full twist
   * @return Mixed Jacobian matrix of size (3 * num_contacts) x num_joints or
   * (6 * num_contacts) x num_joints
   */
  virtual Eigen::MatrixXd getJacobian(bool linear = false);

  /**
   * @brief Compute skew symmetric matrix for a 3D vector
   * @param[in] vector 3D vector
   * @return Skew symmetric matrix of size 3 x 3
   */
  Eigen::Matrix3d getSkew(const Eigen::Vector3d & vector) const;

  /**
   * @brief Compute adjoint mapping twist in child frame to twist in parent
   * frame
   * @param[in] position Position of child frame in parent frame
   * @param[in] rotation Rotation of child frame in parent frame
   * @return Adjoint matrix of size 6 x 6
   */
  virtual Eigen::Matrix<double, 6, 6> getAdjoint(
    const Eigen::Vector3d & position,
    const Eigen::Matrix3d & rotation);

  /**
   * @brief Compute adjoint mapping twist in child frame to twist in parent
   * frame
   * @param[in] parent Name of parent frame
   * @param[in] child Name of child frame
   * @return Adjoint matrix of size 6 x 6
   */
  virtual Eigen::Matrix<double, 6, 6> getAdjoint(
    const std::string & parent, const std::string & child)
  {
    auto transform = getTransform(parent, child);
    return getAdjoint(transform.first, transform.second);
  }

  /**
   * @brief Compute adjoint mapping twist in child frame to twist in body frame
   * @param[in] child Name of child frame
   * @return Adjoint matrix of size 6 x 6
   */
  Eigen::Matrix<double, 6, 6> getAdjoint(const std::string & child)
  {
    return getAdjoint(body_frame_, child);
  }

  /**
   * @brief Compute grasp map mapping contact forces on the end effectors in
   * the contact wrench basis to wrench on the body
   * @return Grasp map matrix of size 6 x num_constraints
   */
  virtual Eigen::MatrixXd getGraspMap() = 0;

  /**
   * @brief Compute mass matrix in joint space of the robot
   * @return Mass matrix of size num_joints x num_joints
   */
  virtual Eigen::MatrixXd getMassMatrix() = 0;

  /**
   * @brief Compute Coriolis vector in joint space of the robot
   * @return Coriolis vector of size num_joints
   */
  virtual Eigen::VectorXd getCoriolisVector() = 0;

  /**
   * @brief Compute gravity vector in joint space of the robot
   * @param[in] gravity Gravity vector in the base frame
   * @return Gravity vector of size num_joints
   */
  virtual Eigen::VectorXd getGravitationalVector(
    const Eigen::Vector3d & gravity) = 0;

  /**
   * @brief Compute derivative of gravitational vector in joint space of the
   * robot with respect to gravity in the base frame
   * @return Gravitational matrix of size num_joints x 3
   */
  virtual Eigen::MatrixXd getGravitationalMatrix() = 0;

  /**
   * @brief Compute center of mass of the robot in the body frame
   * @return Center of mass of the robot in the body frame
   */
  virtual Eigen::Vector3d getCenterOfMass() = 0;

  /**
   * @brief Update joint states with latest data from robot
   * @param[in] states Joint states
   */
  virtual void updateJointState(const JointState & states);

  /**
   * @brief Update contact frames with latest data from robot
   * @param[in] transforms Transforms from end-effector frames to contact frames
   */
  virtual void updateContactFrames(
    const std::vector<TransformStamped> & transforms) = 0;

  void declareParameters() override;

  void setParameter(
    const Parameter & param, SetParametersResult & result) override;

  using Parameterized::setParameter;

  /**
   * @return True if the robot description has been loaded
   */
  bool isInitialized() const {return initialized_;}

  /**
   * @return Number of joints
   */
  int getNumJoints() const {return num_joints_;}

  /**
   * @return Number of contact frames
   */
  int getNumContacts() const {return num_contacts_;}

  /**
   * @return Number of contact constraints
   */
  int getNumConstraints() const {return num_constraints_;}

  /**
   * @return Vector of joint names
   */
  std::vector<std::string> getJointNames() const {return joint_names_;}

  /**
   * @return Name of body frame
   */
  std::string getBodyFrame() const {return body_frame_;}

  /**
   * @return Vector of contact frame names
   */
  std::vector<std::string> getContactFrames() const
  {
    return contact_frames_;
  }

  /**
   * @return Vector of end effector frame names
   */
  std::vector<std::string> getEndEffectorFrames() const
  {
    return end_effector_frames_;
  }

  /**
   * @return Total mass of the robot (kg)
   */
  double getMass() const {return mass_;}

  /**
   * @return Mapping of link names to masses (kg)
   */
  std::unordered_map<std::string, double> getLinkMasses() const
  {
    return link_masses_;
  }

  /**
   * @return Wrench basis of the given contact frame
   */
  Eigen::MatrixXd getWrenchBasis(const std::string & contact) const
  {
    auto it = wrench_bases_.find(contact);
    if (it == wrench_bases_.end()) {
      return Eigen::MatrixXd();
    }
    return it->second;
  }

  /**
   * @return Vector of contact types
   */
  std::vector<ContactType> getContactTypes() const
  {
    return contact_types_;
  }

  /**
   * @return Vector of wrist types
   */
  std::vector<WristType> getWristTypes() const
  {
    return wrist_types_;
  }

  /**
   * @return Contact type of the given contact frame
   */
  ContactType getContactType(const std::string & contact) const
  {
    auto it = std::find(
      contact_frames_.begin(), contact_frames_.end(), contact);
    if (it == contact_frames_.end()) {
      return ContactType::DEFAULT;
    }
    return contact_types_[it - contact_frames_.begin()];
  }

  /**
   * @return Wrist type of the given contact frame
   */
  WristType getWristType(const std::string & contact) const
  {
    auto it = std::find(
      contact_frames_.begin(), contact_frames_.end(), contact);
    if (it == contact_frames_.end()) {
      return WristType::FIXED;
    }
    return wrist_types_[it - contact_frames_.begin()];
  }

  /**
   * @return Vector of joint positions (rad or m)
   */
  Eigen::VectorXd getJointPosition() const {return joint_pos_;}

  /**
   * @return Vector of joint velocities (rad/s or m/s)
   */
  Eigen::VectorXd getJointVelocity() const {return joint_vel_;}

  /**
   * @return Vector of joint efforts (Nm or N)
   */
  Eigen::VectorXd getJointEffort() const {return joint_eff_;}

  /**
   * @return Vector of joint position lower bounds (rad or m)
   */
  Eigen::VectorXd getJointPositionMin() const {return joint_pos_min_;}

  /**
   * @return Vector of joint position upper bounds (rad or m)
   */
  Eigen::VectorXd getJointPositionMax() const {return joint_pos_max_;}

  /**
   * @return Vector of joint velocity lower bounds (rad/s or m/s)
   */
  Eigen::VectorXd getJointVelocityMin() const {return joint_vel_min_;}

  /**
   * @return Vector of joint velocity upper bounds (rad/s or m/s)
   */
  Eigen::VectorXd getJointVelocityMax() const {return joint_vel_max_;}

  /**
   * @return Vector of joint effort lower bounds (Nm or N)
   */
  Eigen::VectorXd getJointEffortMin() const {return joint_eff_min_;}

  /**
   * @return Vector of joint effort upper bounds (Nm or N)
   */
  Eigen::VectorXd getJointEffortMax() const {return joint_eff_max_;}

protected:
  /**
   * @brief Perform any necessary initialization steps after loading the robot
   * description and setting parameter values
   * @param[out] error_message Error message if initialization fails
   * @return True if the robot was successfully initialized
   */
  virtual bool initialize(std::string & error_message);

  bool initialized_ = false;      // Has robot description been loaded
  size_t num_joints_;             // Number of joints
  size_t num_contacts_;           // Number of contacts
  size_t num_constraints_;        // Number of contact constraints

  // URDF model
  urdf::Model urdf_model_;
  // Body frame name
  std::string body_frame_;
  // Contact frame names
  std::vector<std::string> contact_frames_;
  // End effector contact types
  std::vector<ContactType> contact_types_;
  // End effector wrist types
  std::vector<WristType> wrist_types_;
  // End effector frame names
  std::vector<std::string> end_effector_frames_;
  // Link masses
  std::unordered_map<std::string, double> link_masses_;
  // Wrench basis for each contact frame
  std::unordered_map<std::string, Eigen::Matrix<double, 6, Eigen::Dynamic>>
  wrench_bases_;
  // Joint names
  std::vector<std::string> joint_names_;

  // Joint state
  Eigen::VectorXd joint_pos_;       // Joint position (rad or m)
  Eigen::VectorXd joint_vel_;       // Joint velocity (rad/s or m/s)
  Eigen::VectorXd joint_eff_;       // Joint effort (Nm or N)

  // Joint limits
  Eigen::VectorXd joint_pos_min_;   // Joint position lower bound (rad or m)
  Eigen::VectorXd joint_pos_max_;   // Joint position upper bound (rad or m)
  Eigen::VectorXd joint_vel_min_;   // Joint velocity lower bound (rad/s or m/s)
  Eigen::VectorXd joint_vel_max_;   // Joint velocity upper bound (rad/s or m/s)
  Eigen::VectorXd joint_eff_min_;   // Joint effort lower bound (Nm or N)
  Eigen::VectorXd joint_eff_max_;   // Joint effort upper bound (Nm or N)

  // Inertial properties
  double mass_;                     // Total mass (kg)
};

#endif  // KINEMATICS_INTERFACE_HPP
