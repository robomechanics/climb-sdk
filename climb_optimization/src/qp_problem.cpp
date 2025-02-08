#include "climb_optimization/qp_problem.hpp"

QpProblem::QpProblem(std::vector<std::string> vars, std::vector<int> sizes)
{
  for (size_t i = 0; i < vars.size(); i++) {
    vars_[vars[i]] = {sizes[i], N};
    N += sizes[i];
  }
  H = Eigen::MatrixXd::Zero(N, N);
  f = Eigen::VectorXd::Zero(N);
  A = Eigen::MatrixXd::Zero(M, N);
  b = Eigen::VectorXd::Zero(M);
  Aeq = Eigen::MatrixXd::Zero(Meq, N);
  beq = Eigen::VectorXd::Zero(Meq);
  lb = Eigen::VectorXd::Constant(N, -INFINITY);
  ub = Eigen::VectorXd::Constant(N, INFINITY);
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

void QpProblem::addInequalityConstraint(
  const std::vector<std::string> & vars,
  const std::vector<Eigen::MatrixXd> & A_in,
  const Eigen::VectorXd & b_in)
{
  assert(A_in.size() && A_in.size() == vars.size());
  auto m = A_in[0].rows();
  assert(b_in.rows() == m);

  A.conservativeResize(M + m, N);
  A.block(M, 0, m, N).setZero();
  for (size_t k = 0; k < A_in.size(); k++) {
    assert(vars_.find(vars[k]) != vars_.end());
    auto [n, i] = vars_[vars[k]];
    assert(A_in[k].cols() == n && A_in[k].rows() == m);
    A.block(M, i, m, n) += A_in[k];
  }
  b.conservativeResize(M + m);
  b.segment(M, m) = b_in;
  M += m;
}

void QpProblem::addEqualityConstraint(
  const std::vector<std::string> & vars,
  const std::vector<Eigen::MatrixXd> & Aeq_in,
  const Eigen::VectorXd & beq_in)
{
  assert(Aeq_in.size() && Aeq_in.size() == vars.size());
  auto m = Aeq_in[0].rows();
  assert(beq_in.rows() == m);

  Aeq.conservativeResize(Meq + m, N);
  Aeq.block(Meq, 0, m, N).setZero();
  for (size_t k = 0; k < Aeq_in.size(); k++) {
    assert(vars_.find(vars[k]) != vars_.end());
    auto [n, i] = vars_[vars[k]];
    assert(Aeq_in[k].cols() == n && Aeq_in[k].rows() == m);
    Aeq.block(Meq, i, m, n) += Aeq_in[k];
  }
  beq.conservativeResize(Meq + m);
  beq.segment(Meq, m) = beq_in;
  Meq += m;
}

void QpProblem::addBounds(
  const std::string & var,
  const Eigen::VectorXd & lb_in,
  const Eigen::VectorXd & ub_in)
{
  assert(vars_.find(var) != vars_.end());
  auto [n, i] = vars_[var];
  assert(lb_in.rows() == n && ub_in.rows() == n);
  lb.segment(i, n) = lb.segment(i, n).cwiseMax(lb_in);
  ub.segment(i, n) = ub.segment(i, n).cwiseMin(ub_in);
}
