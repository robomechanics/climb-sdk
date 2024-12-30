#ifndef QP_INTERFACE_HPP
#define QP_INTERFACE_HPP

#include "climb_optimization/qp_problem.hpp"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <climb_util/parameterized.hpp>

/**
 * @brief Abstract interface for a quadratic program solver
 *
 * Minimize 0.5 * x'Hx + f'x subject to Ax <= b, Aeq x = beq, lb <= x <= ub
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
   * @param[in] A Linear inequality constraint matrix
   * @param[in] b Linear inequality constraint vector
   * @param[in] Aeq Linear equality constraint matrix
   * @param[in] beq Linear equality constraint vector
   * @param[in] lb Lower bound vector
   * @param[in] ub Upper bound vector
   * @return True if a solution was found
   */
  virtual bool solve(
    const Eigen::MatrixXd & H,
    const Eigen::VectorXd & f,
    const Eigen::MatrixXd & A,
    const Eigen::VectorXd & b,
    const Eigen::MatrixXd & Aeq,
    const Eigen::VectorXd & beq,
    const Eigen::VectorXd & lb,
    const Eigen::VectorXd & ub) = 0;

  /**
   * @brief Solve a quadratic program
   * @param[in] problem Quadratic program problem definition
   * @return True if a solution was found
   */
  bool solve(QpProblem & problem)
  {
    return solve(
      problem.H, problem.f, problem.A, problem.b, problem.Aeq, problem.beq,
      problem.lb, problem.ub);
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
    const Eigen::MatrixXd & b,
    const Eigen::MatrixXd & Aeq,
    const Eigen::VectorXd & beq,
    const Eigen::VectorXd & lb,
    const Eigen::VectorXd & ub)
  {
    return solve(H, f, A, b, Aeq, beq, lb, ub);
  }

  /**
   * @brief Solve the quadratic program with updated parameters
   * @param[in] problem Quadratic program problem definition
   * @return True if a solution was found
   */
  bool update(QpProblem & problem)
  {
    return update(
      problem.H, problem.f, problem.A, problem.b, problem.Aeq, problem.beq,
      problem.lb, problem.ub);
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
   * @brief Check if the solve function has previously been called
   * @return True if the solve function has previously been called
   */
  bool getInitialized() const {return initialized_;}

  virtual void declareParameters() override;

protected:
  Eigen::VectorXd solution_;    // Solution of the last solve
  double cost_;                 // Cost of the last solution
  bool initialized_;            // True if the solver has been initialized
};

#endif  // QP_INTERFACE_HPP
