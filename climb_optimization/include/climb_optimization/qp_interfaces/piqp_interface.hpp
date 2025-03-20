#ifndef CLIMB_OPTIMIZATION__QP_INTERFACES__PIQP_INTERFACE_HPP_
#define CLIMB_OPTIMIZATION__QP_INTERFACES__PIQP_INTERFACE_HPP_

#include <Eigen/Core>
#include <piqp/piqp.hpp>

#include "climb_optimization/qp_interfaces/qp_interface.hpp"

/**
 * @brief Interface for the PIQP (Proximal Interior Point Quadratic
 * Programming) solver
 */
class PiqpInterface : public QpInterface
{
public:
  PiqpInterface();

  bool solve(
    const Eigen::MatrixXd & H,
    const Eigen::VectorXd & f,
    const Eigen::MatrixXd & A,
    const Eigen::VectorXd & b,
    const Eigen::MatrixXd & Aeq,
    const Eigen::VectorXd & beq,
    const Eigen::VectorXd & lb,
    const Eigen::VectorXd & ub) override;

  bool update(
    const Eigen::MatrixXd & H,
    const Eigen::VectorXd & f,
    const Eigen::MatrixXd & A,
    const Eigen::VectorXd & b,
    const Eigen::MatrixXd & Aeq,
    const Eigen::VectorXd & beq,
    const Eigen::VectorXd & lb,
    const Eigen::VectorXd & ub) override;

  void declareParameters() override;

  void setParameter(
    const Parameter & param, SetParametersResult & result) override;

  using QpInterface::setParameter;

private:
  piqp::DenseSolver<double> solver_;
};

#endif  // CLIMB_OPTIMIZATION__QP_INTERFACES__PIQP_INTERFACE_HPP_
