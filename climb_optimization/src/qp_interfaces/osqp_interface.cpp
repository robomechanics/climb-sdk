#include "climb_optimization/qp_interfaces/osqp_interface.hpp"

#include <Eigen/Sparse>

OsqpInterface::OsqpInterface()
{
  settings_ = std::make_unique<OSQPSettings>();
  osqp_set_default_settings(settings_.get());
}

bool OsqpInterface::solve(
  const Eigen::MatrixXd & H,
  const Eigen::VectorXd & f,
  const Eigen::MatrixXd & A,
  const Eigen::VectorXd & b,
  const Eigen::MatrixXd & Aeq,
  const Eigen::VectorXd & beq,
  const Eigen::VectorXd & lb,
  const Eigen::VectorXd & ub)
{
  int n = std::max(H.cols(), f.size());
  int m = b.size();
  int meq = beq.size();
  int mbound = lb.size();

  // Merge constraints
  Eigen::MatrixXd A_full(m + meq + mbound, n);
  Eigen::VectorXd lb_full(m + meq + mbound);
  Eigen::VectorXd ub_full(m + meq + mbound);
  int row = 0;
  if (m) {
    A_full.block(row, 0, m, n) = A;
    lb_full.segment(row, m).setConstant(-INFINITY);
    ub_full.segment(row, m) = b;
    row += m;
  }
  if (meq) {
    A_full.block(row, 0, meq, n) = Aeq;
    lb_full.segment(row, meq) = beq;
    ub_full.segment(row, meq) = beq;
    row += meq;
  }
  if (mbound) {
    A_full.block(row, 0, mbound, n) = Eigen::MatrixXd::Identity(n, n);
    lb_full.segment(row, mbound) = lb;
    ub_full.segment(row, mbound) = ub;
  }

  // Populate data
  if (H.size()) {
    assert(H.rows() == n && H.cols() == n);
    data_.P = matrixToOSQP(H.triangularView<Eigen::Upper>());
  } else {
    data_.P = matrixToOSQP(Eigen::MatrixXd::Zero(n, n));
  }
  if (f.size()) {
    assert(f.size() == n);
    data_.q = vectorToOSQP(f);
  } else {
    data_.q = vectorToOSQP(Eigen::VectorXd::Zero(n));
  }
  data_.A = matrixToOSQP(A_full);
  data_.l = vectorToOSQP(lb_full);
  data_.u = vectorToOSQP(ub_full);

  // Setup workspace
  if (workspace_.setup(data_.get(), settings_.get()) != 0) {
    return initialized_ = false;
  }
  initialized_ = true;

  // Solve problem
  osqp_solve(workspace_.get());
  solution_ = osqpToVector(
    workspace_.get()->solution->x, workspace_.get()->data->n);
  cost_ = workspace_.get()->info->obj_val;
  return workspace_.get()->info->status_val == OSQP_SOLVED;
}

bool OsqpInterface::update(
  const Eigen::MatrixXd & H,
  const Eigen::VectorXd & f,
  const Eigen::MatrixXd & A,
  const Eigen::VectorXd & b,
  const Eigen::MatrixXd & Aeq,
  const Eigen::VectorXd & beq,
  const Eigen::VectorXd & lb,
  const Eigen::VectorXd & ub)
{
  int n = std::max(H.cols(), f.size());
  int m = b.size();
  int meq = beq.size();
  int mbound = lb.size();
  if (!initialized_ ||
    m + meq + mbound != workspace_.get()->data->m ||
    n != workspace_.get()->data->n)
  {
    throw std::runtime_error(
      "Problem structure has changed: use solve() instead of update()");
  }

  // Merge constraints
  Eigen::MatrixXd A_full(m + meq + mbound, n);
  Eigen::VectorXd lb_full(m + meq + mbound);
  Eigen::VectorXd ub_full(m + meq + mbound);
  int row = 0;
  if (m) {
    A_full.block(row, 0, m, n) = A;
    lb_full.segment(row, m).setConstant(-INFINITY);
    ub_full.segment(row, m) = b;
    row += m;
  }
  if (meq) {
    A_full.block(row, 0, meq, n) = Aeq;
    lb_full.segment(row, meq) = beq;
    ub_full.segment(row, meq) = beq;
    row += meq;
  }
  if (mbound) {
    A_full.block(row, 0, mbound, n) = Eigen::MatrixXd::Identity(n, n);
    lb_full.segment(row, mbound) = lb;
    ub_full.segment(row, mbound) = ub;
  }

  // Convert to OSQP format
  auto P_new = matrixToOSQP(H.triangularView<Eigen::Upper>());
  auto q_new = vectorToOSQP(f);
  auto A_new = matrixToOSQP(A_full);
  auto l_new = vectorToOSQP(lb_full);
  auto u_new = vectorToOSQP(ub_full);

  // Update workspace
  if (H.size() && A.size()) {
    osqp_update_P_A(
      workspace_.get(),
      P_new.get()->x, OSQP_NULL, P_new.get()->nzmax,
      A_new.get()->x, OSQP_NULL, A_new.get()->nzmax);
    osqp_update_bounds(workspace_.get(), l_new.data(), u_new.data());
  } else if (H.size()) {
    osqp_update_P(
      workspace_.get(), P_new.get()->x, OSQP_NULL, P_new.get()->nzmax);
  } else if (A.size()) {
    osqp_update_A(
      workspace_.get(), A_new.get()->x, OSQP_NULL, A_new.get()->nzmax);
    osqp_update_bounds(workspace_.get(), l_new.data(), u_new.data());
  }
  if (f.size()) {
    osqp_update_lin_cost(workspace_.get(), q_new.data());
  }

  // Solve problem
  osqp_solve(workspace_.get());
  solution_ = osqpToVector(
    workspace_.get()->solution->x, workspace_.get()->data->n);
  cost_ = workspace_.get()->info->obj_val;
  if (workspace_.get()->info->status_val != OSQP_SOLVED) {
    std::cerr << "OSQP error: " << workspace_.get()->info->status << std::endl;
  }
  return workspace_.get()->info->status_val == OSQP_SOLVED;
}

OsqpInterface::CscWrapper OsqpInterface::matrixToOSQP(
  const Eigen::MatrixXd & mat)
{
  Eigen::SparseMatrix<double> s = mat.sparseView();
  std::vector<c_int> i(s.innerIndexPtr(), s.innerIndexPtr() + s.nonZeros());
  std::vector<c_int> p(s.outerIndexPtr(), s.outerIndexPtr() + s.cols() + 1);
  std::vector<c_float> x;
  x = std::vector<c_float>(s.valuePtr(), s.valuePtr() + s.nonZeros());
  return CscWrapper(
    s.rows(), s.cols(), s.nonZeros(), x, i, p);
}

std::vector<c_float> OsqpInterface::vectorToOSQP(const Eigen::VectorXd & vec)
{
  return std::vector<c_float>(vec.data(), vec.data() + vec.size());
}

Eigen::VectorXd OsqpInterface::osqpToVector(const c_float * vec, c_int size)
{
  return Eigen::Map<const Eigen::VectorXd>(vec, size);
}

void OsqpInterface::declareParameters()
{
  QpInterface::declareParameters();
  declareParameter(
    "eps_prim_inf", -1.0,
    "Primal infeasibility tolerance (-1.0 for solver default)");
  declareParameter(
    "eps_dual_inf", -1.0,
    "Dual infeasibility tolerance (-1.0 for solver default)");
  declareParameter(
    "alpha", -1.0,
    "Relaxation parameter (-1.0 for solver default)");
  declareParameter(
    "rho", -1.0,
    "ADMM penalty parameter initial value (-1.0 for solver default)");
}

void OsqpInterface::setParameter(
  const Parameter & param, [[maybe_unused]] SetParametersResult & result)
{
  if (param.get_name() == "verbose") {
    settings_->verbose = param.as_bool();
  } else if (param.get_name() == "max_iter") {
    if (param.as_int() >= 0) {settings_->max_iter = param.as_int();}
  } else if (param.get_name() == "eps_abs") {
    if (param.as_double() >= 0) {settings_->eps_abs = param.as_double();}
  } else if (param.get_name() == "eps_rel") {
    if (param.as_double() >= 0) {settings_->eps_rel = param.as_double();}
  } else if (param.get_name() == "eps_prim_inf") {
    if (param.as_double() >= 0) {settings_->eps_prim_inf = param.as_double();}
  } else if (param.get_name() == "eps_dual_inf") {
    if (param.as_double() >= 0) {settings_->eps_dual_inf = param.as_double();}
  } else if (param.get_name() == "alpha") {
    if (param.as_double() >= 0) {settings_->alpha = param.as_double();}
  } else if (param.get_name() == "rho") {
    if (param.as_double() >= 0) {settings_->rho = param.as_double();}
  }
}
