#include "climb_main/optimization/piqp_interface.hpp"

PiqpInterface::PiqpInterface()
{
}

bool PiqpInterface::solve(
  const Eigen::MatrixXd & H,
  const Eigen::VectorXd & f,
  const Eigen::MatrixXd & A,
  const Eigen::VectorXd & b,
  const Eigen::MatrixXd & Aeq,
  const Eigen::VectorXd & beq,
  const Eigen::VectorXd & lb,
  const Eigen::VectorXd & ub)
{
  Eigen::MatrixXd H_full = H;
  if (!H_full.size()) {
    H_full = Eigen::MatrixXd::Zero(f.size(), f.size());
  }
  Eigen::VectorXd f_full = f;
  if (!f_full.size()) {
    f_full = Eigen::VectorXd::Zero(H_full.rows());
  }
  solver_.setup(
    H_full, f_full,
    Aeq.size() ? piqp::optional<decltype(Aeq)>(Aeq) : piqp::nullopt,
    beq.size() ? piqp::optional<decltype(beq)>(beq) : piqp::nullopt,
    A.size() ? piqp::optional<decltype(A)>(A) : piqp::nullopt,
    b.size() ? piqp::optional<decltype(b)>(b) : piqp::nullopt,
    lb.size() ? piqp::optional<decltype(lb)>(lb) : piqp::nullopt,
    ub.size() ? piqp::optional<decltype(ub)>(ub) : piqp::nullopt);
  piqp::Status status = solver_.solve();
  solution_ = solver_.result().x;
  cost_ = solver_.result().info.primal_obj;
  return status == piqp::Status::PIQP_SOLVED;
}

bool PiqpInterface::update(
  const Eigen::MatrixXd & H,
  const Eigen::VectorXd & f,
  const Eigen::MatrixXd & A,
  const Eigen::VectorXd & b,
  const Eigen::MatrixXd & Aeq,
  const Eigen::VectorXd & beq,
  const Eigen::VectorXd & lb,
  const Eigen::VectorXd & ub)
{
  Eigen::MatrixXd H_full = H;
  if (!H_full.size()) {
    H_full = Eigen::MatrixXd::Zero(f.size(), f.size());
  }
  Eigen::VectorXd f_full = f;
  if (!f_full.size()) {
    f_full = Eigen::VectorXd::Zero(H_full.rows());
  }
  solver_.update(
    H_full, f_full,
    Aeq.size() ? piqp::optional<decltype(Aeq)>(Aeq) : piqp::nullopt,
    beq.size() ? piqp::optional<decltype(beq)>(beq) : piqp::nullopt,
    A.size() ? piqp::optional<decltype(A)>(A) : piqp::nullopt,
    b.size() ? piqp::optional<decltype(b)>(b) : piqp::nullopt,
    lb.size() ? piqp::optional<decltype(lb)>(lb) : piqp::nullopt,
    ub.size() ? piqp::optional<decltype(ub)>(ub) : piqp::nullopt);
  piqp::Status status = solver_.solve();
  solution_ = solver_.result().x;
  cost_ = solver_.result().info.primal_obj;
  return status == piqp::Status::PIQP_SOLVED;
}

void PiqpInterface::declareParameters()
{
  declareParameter("check_duality_gap", true,
    "Check terminal criterion on duality gap");
  declareParameter("eps_duality_gap_abs", -1.0,
    "Absolute tolerance for duality gap (-1 for default)");
  declareParameter("eps_duality_gap_rel", -1.0,
    "Relative duality gap tolerance (-1 for default)");
}

void PiqpInterface::setParameter(
  const Parameter & param, [[maybe_unused]] SetParametersResult & result)
{
  if (param.get_name() == "verbose") {
    solver_.settings().verbose = param.as_bool();
    solver_.settings().compute_timings = param.as_bool();
  } else if (param.get_name() == "max_iter") {
    if (param.as_int() >= 0) solver_.settings().max_iter = param.as_int();
  } else if (param.get_name() == "eps_abs") {
    if (param.as_double() >= 0) solver_.settings().eps_abs = param.as_double();
  } else if (param.get_name() == "eps_rel") {
    if (param.as_double() >= 0) solver_.settings().eps_rel = param.as_double();
  }
}
