#ifndef RBDL_INTERFACE_H
#define RBDL_INTERFACE_H

// #include <rbdl/rbdl.h>
#include "climb_main/kinematics/kinematics_interface.hpp"
#include <kdl/tree.hpp>

/**
 * @brief Interface for the Rigid Body Dynamics Library (RBDL)
 */
class KdlInterface : public KinematicsInterface
{
public:
  bool loadRobotDescription(std::string urdf_file) override;
  Eigen::MatrixXd getJacobian() override;
  Eigen::MatrixXd getGraspMap() override;

  void declareParameters(const rclcpp::Node::SharedPtr node) override;
  void setParameter(
    const rclcpp::Parameter & param,
    rcl_interfaces::msg::SetParametersResult & result) override;

private:
  /**
   * @brief Cache KDL chains for each end effector
   */
  void update_chains();

  // KDL tree representing robot
  KDL::Tree tree_;
  // Flag indicating whether the KDL tree has been initialized
  bool initialized_;
  // End effector frame names
  std::vector<std::string> ee_frames_;
  // Contact frame names
  std::vector<std::string> contact_frames_;
  // KDL chains from body to each end effector frame
  std::map<std::string, KDL::Chain> ee_chains_;
  // KDL chains from body to each contact frame
  std::map<std::string, KDL::Chain> contact_chains_;
};

#endif  // RBDL_INTERFACE_H
