#include "climb_main/controller/osqp_interface.hpp"

OsqpInterface::OsqpInterface()
{
  settings_ = std::make_unique<OSQPSettings>();
  osqp_set_default_settings(settings_.get());
}

bool OsqpInterface::solve(
  const Eigen::MatrixXd & H,
  const Eigen::VectorXd & f,
  const Eigen::MatrixXd & A,
  const Eigen::VectorXd & lb,
  const Eigen::VectorXd & ub,
  const Eigen::SparseMatrix<double> & H_sparsity,
  const Eigen::SparseMatrix<double> & A_sparsity)
{
  // Populate data
  H_sparsity_ = H_sparsity.triangularView<Eigen::Upper>();
  H_sparsity_.makeCompressed();
  A_sparsity_ = A_sparsity;
  data_.P = eigenToOSQP(H.triangularView<Eigen::Upper>(), H_sparsity_);
  data_.q = eigenToOSQP(f);
  data_.A = eigenToOSQP(A, A_sparsity);
  data_.l = eigenToOSQP(lb);
  data_.u = eigenToOSQP(ub);

  // Setup workspace
  if (workspace_.setup(data_.get(), settings_.get()) != 0) {
    return initialized_ = false;
  }

  // Solve problem
  osqp_solve(workspace_.get());
  solution_ = osqpToEigen(
    workspace_.get()->solution->x, workspace_.get()->data->n);
  cost_ = workspace_.get()->info->obj_val;
  return initialized_ = true;
}

bool OsqpInterface::update(
  const Eigen::MatrixXd & H,
  const Eigen::VectorXd & f,
  const Eigen::MatrixXd & A,
  const Eigen::VectorXd & lb,
  const Eigen::VectorXd & ub)
{
  // Reset solver if sparsity structure may have changed
  if (!initialized_ ||
    (H.size() && !H_sparsity_.size()) ||
    (A.size() && !A_sparsity_.size()))
  {
    return solve(H, f, A, lb, ub);
  }

  // Update data
  if (H.size()) {
    data_.P = eigenToOSQP(H.triangularView<Eigen::Upper>(), H_sparsity_);
  }
  if (A.size()) {
    data_.A = eigenToOSQP(A, A_sparsity_);
  }
  if (f.size()) {
    data_.q = eigenToOSQP(f);
  }
  if (lb.size()) {
    data_.l = eigenToOSQP(lb);
  }
  if (ub.size()) {
    data_.u = eigenToOSQP(ub);
  }

  // Update workspace
  if (H.size() && A.size()) {
    osqp_update_P_A(
      workspace_.get(),
      data_.P.get()->x, OSQP_NULL, data_.P.get()->nzmax,
      data_.A.get()->x, OSQP_NULL, data_.A.get()->nzmax);
  } else if (H.size()) {
    osqp_update_P(
      workspace_.get(), data_.P.get()->x, OSQP_NULL, data_.P.get()->nzmax);
  } else if (A.size()) {
    osqp_update_A(
      workspace_.get(), data_.A.get()->x, OSQP_NULL, data_.A.get()->nzmax);
  }
  if (f.size()) {
    osqp_update_lin_cost(workspace_.get(), data_.q.data());
  }
  if (lb.size() && ub.size()) {
    osqp_update_bounds(workspace_.get(), data_.l.data(), data_.u.data());
  } else if (lb.size()) {
    osqp_update_lower_bound(workspace_.get(), data_.l.data());
  } else if (ub.size()) {
    osqp_update_upper_bound(workspace_.get(), data_.u.data());
  }

  // Solve problem
  osqp_solve(workspace_.get());
  solution_ = Eigen::Map<Eigen::VectorXd>(
    workspace_.get()->solution->x, data_.q.size());
  cost_ = workspace_.get()->info->obj_val;
  return true;
}

OsqpInterface::CscWrapper OsqpInterface::eigenToOSQP(
  const Eigen::MatrixXd & mat, const Eigen::SparseMatrix<double> & sparsity)
{
  Eigen::SparseMatrix<double> s = sparsity.size() ? sparsity : mat.sparseView();
  std::vector<c_int> i(s.innerIndexPtr(), s.innerIndexPtr() + s.nonZeros());
  std::vector<c_int> p(s.outerIndexPtr(), s.outerIndexPtr() + s.cols() + 1);
  std::vector<c_float> x;
  if (sparsity.size() == 0) {
    // Use the sparsity pattern of the original matrix
    x = std::vector<c_float>(s.valuePtr(), s.valuePtr() + s.nonZeros());
  } else {
    // Use the specified sparsity pattern
    x.reserve(s.nonZeros());
    for (int k = 0; k < s.outerSize(); ++k) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(s, k); it; ++it) {
        x.push_back(mat(it.row(), it.col()));
      }
    }
  }
  return CscWrapper(
    s.rows(), s.cols(), s.nonZeros(), x, i, p);
}

std::vector<c_float> OsqpInterface::eigenToOSQP(const Eigen::VectorXd & vec)
{
  return std::vector<c_float>(vec.data(), vec.data() + vec.size());
}

Eigen::VectorXd OsqpInterface::osqpToEigen(const c_float * vec, c_int size)
{
  return Eigen::Map<const Eigen::VectorXd>(vec, size);
}
