#include "climb_main/kinematics/kinematics_interface.hpp"

KinematicsInterface::KinematicsInterface()
{
}

Eigen::VectorXd KinematicsInterface::vectorToEigen(
  const std::vector<double> & values,
  const std::vector<std::string> & joints)
{
  // Implement std::vector to Eigen conversion
  return Eigen::VectorXd();
}

std::vector<double> KinematicsInterface::eigenToVector(const Eigen::VectorXd & vector)
{
  // Implement Eigen to std::vector conversion
  return std::vector<double>();
}

void KinematicsInterface::updateJointState(const JointState & state)
{
  // Implement joint state update
}

void KinematicsInterface::updateContactFrames(const std::vector<TransformStamped> & transforms)
{
  // Implement contact frame update
}
