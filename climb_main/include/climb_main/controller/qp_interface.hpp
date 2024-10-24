#ifndef QP_INTERFACE_HPP
#define QP_INTERFACE_HPP

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "climb_main/util/parameterized.hpp"

/**
 * @brief Abstract interface for a quadratic program solver
 *
 * Minimize 0.5 * x'Hx + f'x subject to lb <= Ax <= ub
 */
class QpInterface : public Parameterized
{
public:
  QpInterface();

  virtual ~QpInterface() = default;

  /**
   * @brief Solve a quadratic program
   * @param[in] H Quadratic cost matrix
   * @param[in] f Linear cost vector
   * @param[in] A Linear constraint matrix
   * @param[in] lb Linear constraint lower bound vector
   * @param[in] ub Linear constraint upper bound vector
   * @return True if a solution was found
   */
  virtual bool solve(
    const Eigen::MatrixXd & H,
    const Eigen::VectorXd & f,
    const Eigen::MatrixXd & A,
    const Eigen::VectorXd & lb,
    const Eigen::VectorXd & ub) = 0;

  /**
   * @brief Solve a quadratic program with a fixed sparsity structure
   * @param[in] H Quadratic cost matrix
   * @param[in] f Linear cost vector
   * @param[in] A Linear constraint matrix
   * @param[in] lb Linear constraint lower bound vector
   * @param[in] ub Linear constraint upper bound vector
   * @param[in] H_sparsity Sparsity structure of H
   * @param[in] A_sparsity Sparsity structure of A
   * @return True if a solution was found
   */
  virtual bool solve(
    const Eigen::MatrixXd & H,
    const Eigen::VectorXd & f,
    const Eigen::MatrixXd & A,
    const Eigen::VectorXd & lb,
    const Eigen::VectorXd & ub,
    [[maybe_unused]] const Eigen::SparseMatrix<double> & H_sparsity,
    [[maybe_unused]] const Eigen::SparseMatrix<double> & A_sparsity)
  {
    return solve(H, f, A, lb, ub);
  }

  /**
   * @brief Solve the quadratic program with updated parameters
   * @param[in] H Quadratic cost matrix (empty if unchanged)
   * @param[in] f Linear cost vector (empty if unchanged)
   * @param[in] A Linear constraint matrix (empty if unchanged)
   * @param[in] lb Linear constraint lower bound vector (empty if unchanged)
   * @param[in] ub Linear constraint upper bound vector (empty if unchanged)
   * @return True if a solution was found
   */
  virtual bool update(
    const Eigen::MatrixXd & H,
    const Eigen::VectorXd & f,
    const Eigen::MatrixXd & A,
    const Eigen::VectorXd & lb,
    const Eigen::VectorXd & ub)
  {
    return solve(H, f, A, lb, ub);
  }

  /**
   * @brief Get the solution of the last solve
   * @return Solution of the last solve or empty vector if no solution
   */
  Eigen::VectorXd getSolution() const {return solution_;}

  /**
   * @brief Get the cost of the last solution
   * @return Cost of the last solution or infinity if no solution
   */
  double getCost() const {return cost_;}

  /**
   * @brief Check if the solve function has been called
   * @return True if the solve function has been called
   */
  bool getInitialized() const {return initialized_;}

protected:
  Eigen::VectorXd solution_;    // Solution of the last solve
  double cost_;                 // Cost of the last solution
  bool initialized_;            // True if the solver has been initialized
};

#endif  // QP_INTERFACE_HPP
