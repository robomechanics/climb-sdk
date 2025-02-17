#include "climb_optimization/qp_interfaces/qp_interface.hpp"

QpInterface::QpInterface() {}

void QpInterface::declareParameters()
{
  declareParameter("verbose", false, "Enable solver output");
  declareParameter(
    "max_iter", -1,
    "Maximum number of solver iterations (-1 for solver default)");
  declareParameter(
    "eps_abs", -1.0,
    "Absolute convergence tolerance (-1.0 for solver default)");
  declareParameter(
    "eps_rel", -1.0,
    "Relative convergence tolerance (-1.0 for solver default)");
}
