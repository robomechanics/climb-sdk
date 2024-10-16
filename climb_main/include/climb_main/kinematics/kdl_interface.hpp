#ifndef KDL_INTERFACE_HPP
#define KDL_INTERFACE_HPP

#include "climb_main/kinematics/kinematics_interface.hpp"
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>

/**
 * @brief Interface for the Orocos Kinematics and Dynamics Library (KDL)
 */
class KdlInterface : public KinematicsInterface
{
public:
  bool initialize(std::string & error_message) override;

  Eigen::MatrixXd getHandJacobian(std::string contact_frame) override;
  Eigen::MatrixXd getMixedJacobian(
    std::string contact_frame,
    bool linear) override;
  Eigen::MatrixXd getGraspMap() override;

  Eigen::MatrixXd getMassMatrix() override;
  Eigen::VectorXd getCoriolisVector() override;
  Eigen::VectorXd getGravitationalVector(
    const Eigen::Vector3d & gravity) override;

  Eigen::MatrixXd getGravitationalMatrix() override;
  Eigen::Vector3d getCenterOfMass() override;

  void updateJointState(const JointState & states) override;
  void updateContactFrames(
    const std::vector<TransformStamped> & transforms) override;

private:
  // Struct to represent information about a joint
  struct Joint
  {
    int index;                              // Joint index in full joint array
    std::vector<std::string> chain_names;   // Name of each parent chain
    std::vector<int> chain_indices;         // Joint index in each chain
  };

  // Struct to represent information about a kinematic chain
  struct Chain
  {
    std::vector<std::string> joint_names;   // Names of joints in the chain
    KDL::JntArray joint_pos;                // Positions of joints in the chain
    KDL::JntArray joint_vel;                // Velocities of joints in the chain
    KDL::Frame transform;                   // Transform across the chain
    KDL::Chain chain;                       // KDL chain
  };

  /**
   * @brief Update the transform from the end effector to the contact frame
   * @param contact Name of the contact frame
   * @param transform Transform from end effector to contact frame
   */
  void updateContactFrame(std::string contact, KDL::Frame transform);

  // KDL tree representing robot
  KDL::Tree tree_;
  // Kinematic chains from body to each contact
  std::unordered_map<std::string, Chain> chains_;
  // Joint information by joint name
  std::unordered_map<std::string, Joint> joints_;
};

#endif  // KDL_INTERFACE_HPP
