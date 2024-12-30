#ifndef KDL_INTERFACE_HPP
#define KDL_INTERFACE_HPP

#include "climb_kinematics/kinematics_interfaces/kinematics_interface.hpp"
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>

/**
 * @brief Hash function for std::pair
 */
struct pair_hash
{
  template<class T1, class T2>
  std::size_t operator()(const std::pair<T1, T2> & pair) const
  {
    std::size_t h1 = std::hash<T1>{}(pair.first);
    std::size_t h2 = std::hash<T2>{}(pair.second);
    return h1 ^ (h2 << 1);
  }
};

/**
 * @brief Interface for the Orocos Kinematics and Dynamics Library (KDL)
 */
class KdlInterface : public KinematicsInterface
{
public:
  bool initialize(std::string & error_message) override;

  Eigen::Isometry3d getTransform(
    const std::string & parent, const std::string & child) override;
  Eigen::MatrixXd getHandJacobian(const std::string & contact_frame) override;
  Eigen::MatrixXd getHandJacobian() override;
  Eigen::MatrixXd getJacobian(
    const std::string & parent, const std::string & child,
    bool linear) override;
  Eigen::MatrixXd getJacobian(bool linear) override;
  Eigen::Matrix<double, 6, Eigen::Dynamic> getGraspMap() override;

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
  /**
   * @brief Kinematic chain with joint information
   */
  struct Chain
  {
    KDL::Chain kdl_chain;
    std::vector<std::string> joint_names;
    std::vector<int> joint_indices;
    size_t joints;
    size_t segments;
    std::unordered_map<std::string, int> contact_indices;
  };

  /**
   * @brief Get a Joint array of values for a specific chain
   * @param var Vector of joint values for the entire robot
   * @param chain Kinematic chain
   */
  KDL::JntArray getJointArray(Eigen::VectorXd var, Chain chain) const;

  /**
   * @brief Get and cache the requested chain or return the cached chain
   * @param parent Name of parent frame
   * @param child Name of child frame
   * @return Chain from parent to child frame
   * @throws std::invalid_argument if chain cannot be found
   */
  Chain getChain(const std::string & parent, const std::string & child);

  /**
   * @brief Get the transform from parent to child frame
   * @param parent Name of parent frame
   * @param child Name of child frame
   * @return KDL frame of child frame in parent frame
   */
  KDL::Frame getTransformKdl(const std::string & parent, const std::string & child);

  std::tuple<std::string, bool> findSuffix(
    const std::string & name, const std::string & suffix);

  // KDL tree representing robot
  KDL::Tree tree_;
  // Joint indices by joint name
  std::unordered_map<std::string, int> joint_indices_;
  // Contact frame transforms
  std::unordered_map<std::string, KDL::Frame> contact_transforms_;
  // Cached kinematic chains by parent and child frame names
  std::unordered_map<std::pair<std::string, std::string>, Chain, pair_hash>
  chains_;
  // Cached large matrix computations by name
  std::unordered_map<std::string, Eigen::MatrixXd> matrices_;
  // Cached transforms by parent and child frame names
  std::unordered_map<std::pair<std::string, std::string>,
    KDL::Frame, pair_hash> transforms_;
  // Cached Jacobians by parent and child frame names
  std::unordered_map<std::pair<std::string, std::string>,
    Eigen::MatrixXd, pair_hash> jacobians_;
};

#endif  // KDL_INTERFACE_HPP
