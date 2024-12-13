#ifndef PIQP_INTERFACE_HPP
#define PIQP_INTERFACE_HPP

#include "climb_main/optimization/qp_interface.hpp"
#include <piqp/piqp.hpp>

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
    const Eigen::VectorXd & ub);

  bool update(
    const Eigen::MatrixXd & H,
    const Eigen::VectorXd & f,
    const Eigen::MatrixXd & A,
    const Eigen::VectorXd & b,
    const Eigen::MatrixXd & Aeq,
    const Eigen::VectorXd & beq,
    const Eigen::VectorXd & lb,
    const Eigen::VectorXd & ub);

  void declareParameters() override;

  void setParameter(
    const Parameter & param, SetParametersResult & result) override;

  using QpInterface::setParameter;

private:
  piqp::DenseSolver<double> solver_;
};

#endif  // PIQP_INTERFACE_HPP
