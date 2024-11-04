#include "climb_main/optimization/qp_interface.hpp"

QpInterface::QpInterface()
{
}

void QpInterface::declareParameters()
{
  declareParameter("verbose", false, "Enable solver output");
}
