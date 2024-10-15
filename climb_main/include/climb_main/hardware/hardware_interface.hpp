#ifndef HARDWARE_INTERFACE_HPP
#define HARDWARE_INTERFACE_HPP

#include <string>
#include <vector>
#include <unordered_map>
#include <sensor_msgs/msg/joint_state.hpp>
#include <climb_msgs/msg/actuator_state.hpp>
#include "climb_msgs/msg/joint_command.hpp"
#include "climb_main/util/parameterized.hpp"

using sensor_msgs::msg::JointState;
using climb_msgs::msg::ActuatorState;
using climb_msgs::msg::JointCommand;

/**
 * @brief Abstract interface for communicating with a physical robot
 */
class HardwareInterface : public Parameterized
{
public:
  /**
   * @brief Default destructor for HardwareInterface
   */
  virtual ~HardwareInterface() = default;

  /**
   * @brief Connect to the robot
   * @return True if the connection was successful
   */
  virtual bool connect() = 0;

  /**
   * @brief Disconnect from the robot
   */
  virtual void disconnect() = 0;

  /**
   * @brief Check if the interface is connected to the robot
   * @return True if the interface is connected
   */
  virtual bool isConnected() = 0;

  /**
   * @brief Add multiple actuators of the same model to the interface
   * @param ids Actuator IDs
   * @param joints Joint names corresponding to each actuator
   * @param model Actuator model
   * @param ratio Gear ratio of the actuators (high number means more torque)
   *
   * Multiple actuators sharing a joint name will act in parallel
   */
  virtual void addActuators(
    std::vector<int> ids,
    std::vector<std::string> joints, std::string model, double ratio = 1);

  /**
   * @brief Remove actuators by ID (removed actuators will be disabled)
   * @param ids Actuator IDs to remove
   */
  virtual void removeActuators(std::vector<int> ids);

  /**
   * @brief Remove all actuators (removed actuators will be disabled)
   */
  inline void removeActuators() {removeActuators(ids_);}

  /**
   * @brief Remove actuators corresponding to specific joints
   */
  void removeJoints(std::vector<std::string> joints);

  /**
   * @brief Get actuator model by actuator ID
   * @param id Actuator ID
   * @return Actuator model ("" if ID not found)
   */
  inline std::string getActuator(int id) const
  {
    try {
      return models_by_id_.at(id);
    } catch (const std::out_of_range & e) {
      return "";
    }
  }

  /**
   * @brief Get actuator models indexed by actuator ID
   * @return Map of actuator IDs to actuator models
   */
  inline std::unordered_map<int, std::string> getActuators() const
  {
    return models_by_id_;
  }

  /**
   * @brief Get gear ratio by actuator ID
   * @param id Actuator ID
   * @return Gear ratio (0 if ID not found)
   */
  inline double getRatio(int id) const
  {
    try {
      return ratios_by_id_.at(id);
    } catch (const std::out_of_range & e) {
      return 0;
    }
  }

  /**
   * @brief Get gear ratios indexed by actuator ID
   * @return Map of actuator IDs to gear ratios
   */
  inline std::unordered_map<int, double> getRatios() const
  {
    return ratios_by_id_;
  }

  /**
   * @brief Change gear ratio of specific actuators
   * @param ids Actuator IDs
   * @param ratio New gear ratio
   */
  void setRatios(std::vector<int> ids, double ratio);

  /**
   * @brief Get joint name by actuator ID
   * @param id Actuator ID
   * @return Joint name ("" if ID not found)
   */
  inline std::string getJoint(int id) const
  {
    try {
      return joints_by_id_.at(id);
    } catch (const std::out_of_range & e) {
      return "";
    }
  }

  /**
   * @brief Get joint names indexed by actuator ID
   * @return Map of actuator IDs to joint names
   */
  inline std::unordered_map<int, std::string> getJoints() const
  {
    return joints_by_id_;
  }

  /**
   * @brief Get actuator ID by joint name
   * @param joint Joint name
   * @return Vector of corresponding actuator IDs (empty list if not found)
   */
  inline std::vector<int> getId(std::string joint) const
  {
    try {
      return ids_by_joint_.at(joint);
    } catch (const std::out_of_range & e) {
      return {};
    }
  }

  /**
   * @brief Get all actuator IDs
   * @return Vector of actuator IDs
   */
  inline std::vector<int> getIds() const {return ids_;}

  /**
   * @brief Enable actuators by ID
   * @param ids Actuator IDs to enable
   * @return True on success
   */
  virtual bool enable(std::vector<int> ids) = 0;

  /**
   * @brief Enable all actuators
   * @return True on success
   */
  inline bool enable() {return enable(ids_);}

  /**
   * @brief Disable actuators by ID
   * @param ids Actuator IDs to disable
   * @return True on success
   */
  virtual bool disable(std::vector<int> ids) = 0;

  /**
   * @brief Disable all actuators
   * @return True on success
   */
  inline bool disable() {return disable(ids_);}

  /**
   * @brief Read current actuator states
   * @return Actuator state message (empty list on failure)
   */
  virtual ActuatorState readActuatorState() = 0;

  /**
   * @brief Read current joint states from actuators
   * @return Joint state message (empty list on failure)
   */
  virtual JointState readJointState() = 0;

  /**
   * @brief Write desired joint state and operating modes to actuators
   * @param command Joint command message
   * @return True on success
   */
  virtual bool writeJointCommand(JointCommand command) = 0;

  /**
   * @brief Ensure a joint command is valid
   * @param command Joint command to validate
   * @return True if command is valid
   */
  bool validateJointCommand(const JointCommand & command);

protected:
  // Actuator IDs
  std::vector<int> ids_;
  // Actuator IDs for each joint name
  std::unordered_map<std::string, std::vector<int>> ids_by_joint_;
  // Actuator models for each ID
  std::unordered_map<int, std::string> models_by_id_;
  // Joint names for each actuator ID
  std::unordered_map<int, std::string> joints_by_id_;
  // Gear ratios for each actuator ID
  std::unordered_map<int, double> ratios_by_id_;
};

#endif  // HARDWARE_INTERFACE_HPP
