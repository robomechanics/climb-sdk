#ifndef PARAMETERIZED_HPP
#define PARAMETERIZED_HPP

#include <rclcpp/rclcpp.hpp>

using rcl_interfaces::msg::SetParametersResult;
using rcl_interfaces::msg::ParameterDescriptor;
using rclcpp::Parameter;
using rclcpp::ParameterValue;

struct ParameterDeclaration
{
  std::string name;
  ParameterValue default_value;
  ParameterDescriptor descriptor;
};

/**
 * @brief Interface to support ROS parameter handling
 */
class Parameterized
{
public:
  virtual ~Parameterized() = default;

  /**
   * @brief Get a list of the node's requested parameters
   * @return List of parameters
   */
  const std::vector<ParameterDeclaration> getParameters()
  {
    if (parameters_.empty()) {
      declareParameters();
    }
    return parameters_;
  }

  /**
   * @brief Update the value of a parameter
   * @param[in] param The modified parameter
   * @param[out] result Result of the update (only modified if parameter is
   * present but value is rejected)
   */
  virtual void setParameter(
    const Parameter & param, SetParametersResult & result) = 0;

  /**
   * @brief Update the value of a parameter and discard the result
   * @param[in] name The name of the parameter
   * @param[in] value The new value of the parameter
   * @tparam T Type of the parameter or ParameterValue
   */
  template<typename T>
  void setParameter(const std::string & name, const T & value)
  {
    SetParametersResult result;
    setParameter(Parameter(name, value), result);
  }

  /**
   * @brief Set all parameters to their default values
   */
  void defaultParameters();

protected:
  /**
   * @brief Declare all parameters using the declareParameter method
   */
  virtual void declareParameters() = 0;

  /**
   * @brief Shorthand to declare a parameter
   * @param[in] name Name of the parameter
   * @param[in] default_value Default value of the parameter
   * @param[in] description Description of the parameter
   * @tparam T Type of the parameter or ParameterValue
   */
  template<typename T>
  void declareParameter(
    const std::string & name,
    const T & default_value,
    const std::string & description)
  {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = description;
    parameters_.emplace_back(
      ParameterDeclaration{name, ParameterValue(default_value), param_desc});
  }

  /**
   * @brief Shorthand to declare a parameter with floating point bounds
   * @param[in] node ROS node to declare the parameter
   * @param[in] name Name of the parameter
   * @param[in] default_value Default value of the parameter
   * @param[in] description Description of the parameter
   * @param[in] min Lower limit of the floating point parameter
   * @param[in] max Upper limit of the floating point parameter
   * @tparam T Type of the parameter or ParameterValue
   *
   * Note that bounds are not currently enforced on double array parameters
   */
  template<typename T>
  void declareParameter(
    const std::string & name,
    const T & default_value,
    const std::string & description,
    const double & min, const double & max = INFINITY)
  {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = description;
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = min;
    range.to_value = max;
    param_desc.floating_point_range.push_back(range);
    parameters_.emplace_back(
      ParameterDeclaration{name, ParameterValue(default_value), param_desc});
  }

  /**
   * @brief Shorthand to declare a parameter with integer bounds
   * @param[in] node ROS node to declare the parameter
   * @param[in] name Name of the parameter
   * @param[in] default_value Default value of the parameter
   * @param[in] description Description of the parameter
   * @param[in] min Lower limit of the integer parameter
   * @param[in] max Upper limit of the integer parameter
   * @tparam T Type of the parameter or ParameterValue
   *
   * Note that bounds are not currently enforced on integer array parameters
   */
  template<typename T>
  void declareParameter(
    const std::string & name,
    const T & default_value,
    const std::string & description,
    const int & min, const int & max = std::numeric_limits<int>::max())
  {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = description;
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = min;
    range.to_value = max;
    range.step = 1;
    param_desc.integer_range.push_back(range);
    parameters_.emplace_back(
      ParameterDeclaration{name, ParameterValue(default_value), param_desc});
  }

  std::vector<ParameterDeclaration> parameters_;  // List of declared parameters
};

#endif  // PARAMETERIZED_HPP
