#include "climb_main/optimization/qp_problem.hpp"

QpProblem::QpProblem(std::map<std::string, int> var_sizes)
{
  N = 0;
  for (const auto & var : var_sizes) {
    vars_[var.first] = {var.second, N};
    N += var.second;
  }
  M = 0;
  H = Eigen::MatrixXd::Zero(N, N);
  f = Eigen::VectorXd::Zero(N);
  A = Eigen::MatrixXd::Zero(M, N);
  lb = Eigen::VectorXd::Zero(M);
  ub = Eigen::VectorXd::Zero(M);
}

void QpProblem::addQuadraticCost(
  const std::string & var1,
  const std::string & var2,
  const Eigen::MatrixXd & H_in,
  const Eigen::VectorXd & x0,
  const Eigen::VectorXd & y0)
{
  assert(vars_.find(var1) != vars_.end());
  assert(vars_.find(var2) != vars_.end());
  auto [n, i] = vars_[var1];
  auto [m, j] = vars_[var2];
  Eigen::VectorXd x = x0.size() ? x0 : zeros(n);
  Eigen::VectorXd y = y0.size() ? y0 : zeros(m);
  assert(H_in.rows() == n && H_in.cols() == m);
  assert(x.rows() == n && y.rows() == m);

  H.block(i, j, n, m) += H_in / 2;
  H.block(j, i, m, n) += H_in.transpose() / 2;
  f.segment(i, n) -= H_in * y;
  f.segment(j, m) -= H_in.transpose() * x;
}

void QpProblem::addLinearCost(
  const std::string & var, const Eigen::VectorXd & f_in)
{
  assert(vars_.find(var) != vars_.end());
  auto [n, i] = vars_[var];
  assert(f_in.rows() == n);
  f.segment(i, n) += f_in;
}

void QpProblem::addLinearConstraint(
  const std::vector<std::string> & vars,
  const std::vector<Eigen::MatrixXd> & A_in,
  const Eigen::VectorXd & lb_in,
  const Eigen::VectorXd & ub_in)
{
  assert(A_in.size() && A_in.size() == vars.size());
  auto m = A_in[0].rows();
  Eigen::VectorXd lower = lb_in.size() ? lb_in : -infinity(m);
  Eigen::VectorXd upper = ub_in.size() ? ub_in : infinity(m);
  assert(lower.rows() == m);
  assert(upper.rows() == m);

  A.conservativeResize(M + m, N);
  A.block(M, 0, m, N).setZero();
  lb.conservativeResize(M + m);
  ub.conservativeResize(M + m);
  for (size_t k = 0; k < A_in.size(); k++) {
    assert(vars_.find(vars[k]) != vars_.end());
    auto [n, i] = vars_[vars[k]];
    assert(A_in[k].cols() == n && A_in[k].rows() == m);
    A.block(M, i, m, n) = A_in[k];
  }
  lb.segment(M, m) = lower;
  ub.segment(M, m) = upper;
  M += m;
}
