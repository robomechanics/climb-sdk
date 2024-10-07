#ifndef PARAMETERIZED_HPP
#define PARAMETERIZED_HPP

#include <rclcpp/rclcpp.hpp>

/**
 * @brief Interface to support ROS parameter handling
 */
class Parameterized
{
public:
  virtual ~Parameterized() = default;

  /**
   * @brief Declare all parameters
   * @param[in] node ROS node to declare the parameters
   */
  virtual void declareParameters(const rclcpp::Node::SharedPtr node) = 0;

  /**
   * @brief Update the value of a parameter
   * @param[in] param The modified parameter
   * @param[out] result Result of the update (only modified if parameter is
   * present but value is rejected)
   */
  virtual void setParameter(
    const rclcpp::Parameter & param,
    rcl_interfaces::msg::SetParametersResult & result) = 0;

protected:
  /**
   * @brief Shorthand to declare a single parameter
   * @param[in] node ROS node to declare the parameter
   * @param[in] name Name of the parameter
   * @param[in] default_value Default value of the parameter
   * @param[in] description Description of the parameter
   * @tparam T Type of the parameter
   */
  template<typename T>
  void declareParameter(
    const rclcpp::Node::SharedPtr node, const std::string & name,
    const T & default_value, const std::string & description)
  {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = description;
    node->declare_parameter(name, default_value, param_desc);
  }

  /**
   * @brief Shorthand to declare a parameter with floating point bounds
   * @param[in] node ROS node to declare the parameter
   * @param[in] name Name of the parameter
   * @param[in] default_value Default value of the parameter
   * @param[in] description Description of the parameter
   * @param[in] min Lower limit of the floating point parameter
   * @param[in] max Upper limit of the floating point parameter
   * @tparam T Type of the parameter
   *
   * Note that bounds are not currently enforced on double array parameters
   */
  template<typename T>
  void declareParameter(
    const rclcpp::Node::SharedPtr node, const std::string & name,
    const T & default_value, const std::string & description,
    const double & min, const double & max)
  {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = description;
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = min;
    range.to_value = max;
    param_desc.floating_point_range.push_back(range);
    node->declare_parameter(name, default_value, param_desc);
  }

  /**
   * @brief Shorthand to declare a parameter with integer bounds
   * @param[in] node ROS node to declare the parameter
   * @param[in] name Name of the parameter
   * @param[in] default_value Default value of the parameter
   * @param[in] description Description of the parameter
   * @param[in] min Lower limit of the integer parameter
   * @param[in] max Upper limit of the integer parameter
   * @tparam T Type of the parameter
   *
   * Note that bounds are not currently enforced on integer array parameters
   */
  template<typename T>
  void declareParameter(
    const rclcpp::Node::SharedPtr node, const std::string & name,
    const T & default_value, const std::string & description,
    const int & min, const int & max)
  {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = description;
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = min;
    range.to_value = max;
    range.step = 1;
    param_desc.integer_range.push_back(range);
    node->declare_parameter(name, default_value, param_desc);
  }
};

#endif  // PARAMETERIZED_HPP
