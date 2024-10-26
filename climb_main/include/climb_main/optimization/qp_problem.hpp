#ifndef QP_PROBLEM_HPP
#define QP_PROBLEM_HPP

#include <Eigen/Dense>
#include <map>

class QpProblem
{
public:
  QpProblem(std::map<std::string, int> var_sizes);

  /**
   * @brief Add a quadratic cost term of the form: 1/2 (x-x0)' H (y-y0)
   * @param var1 The name of the first variable
   * @param var2 The name of the second variable
   * @param H_in The Hessian matrix of the quadratic cost term
   * @param x0 The nominal value of the first variable (or empty for zero)
   * @param y0 The nominal value of the second variable (or empty for zero)
   */
  void addQuadraticCost(
    const std::string & var1,
    const std::string & var2,
    const Eigen::MatrixXd & H_in,
    const Eigen::VectorXd & x0,
    const Eigen::VectorXd & y0);

  /**
   * @brief Add a quadratic cost term of the form: 1/2 (x - x0)' H (x - x0)
   * @param var The name of the variable
   * @param H_in The Hessian matrix of the quadratic cost term
   * @param x0 The nominal value of the variable (or empty for zero)
   */
  void addQuadraticCost(
    const std::string & var,
    const Eigen::MatrixXd & H_in,
    const Eigen::VectorXd & x0)
  {
    addQuadraticCost(var, var, H_in, x0, x0);
  }

  /**
   * @brief Add a quadratic cost term of the form: cost/2 (x - x0)' (x - x0)
   * @param var The name of the variable
   * @param cost The cost of the quadratic term
   * @param x0 The nominal value of the variable (or empty for zero)
   */
  void addQuadraticCost(
    const std::string & var, float cost, const Eigen::VectorXd & x0)
  {
    assert(vars_.find(var) != vars_.end());
    addQuadraticCost(var, identity(vars_[var].size) * cost, x0);
  }

  /**
   * @brief Add a linear cost term of the form: f' x
   * @param var The name of the variable
   * @param f_in The vector of coefficients of the linear cost term
   */
  void addLinearCost(const std::string & var, const Eigen::VectorXd & f_in);

  /**
   * @brief Add a linear cost term of the form: cost sum(x)
   * @param var The name of the variable
   * @param cost The scalar value of the linear cost term
   */
  void addLinearCost(const std::string & var, float cost)
  {
    assert(vars_.find(var) != vars_.end());
    addLinearCost(var, Eigen::VectorXd::Constant(vars_[var].size, cost));
  }

  /**
   * @brief Add a linear constraint of the form: lb <= A x <= ub
   * @param vars The names of the variables
   * @param A_in The constraint matrix for each variable
   * @param lb_in The lower bound of the constraint
   * @param ub_in The upper bound of the constraint
   */
  void addLinearConstraint(
    const std::vector<std::string> & vars,
    const std::vector<Eigen::MatrixXd> & A_in,
    const Eigen::VectorXd & lb_in,
    const Eigen::VectorXd & ub_in);

  /**
   * @brief Add a linear constraint of the form: lb <= x <= ub
   * @param var The name of the variable
   * @param lb_in The lower bound of the constraint
   * @param ub_in The upper bound of the constraint
   */
  void addBounds(
    const std::string & var,
    const Eigen::VectorXd & lb_in,
    const Eigen::VectorXd & ub_in)
  {
    assert(vars_.find(var) != vars_.end());
    addLinearConstraint({var}, {identity(vars_[var].size)}, lb_in, ub_in);
  }

  /**
   * @brief Add a linear constraint of the form: lb <= x <= ub
   * @param var The name of the variable
   * @param lb_in The lower bound of the constraint
   * @param ub_in The upper bound of the constraint
   */
  void addBounds(
    const std::string & var,
    const double lb_in,
    const double ub_in)
  {
    assert(vars_.find(var) != vars_.end());
    addBounds(
      {var}, lb_in * ones(vars_[var].size), ub_in * ones(vars_[var].size));
  }

  Eigen::MatrixXd H;
  Eigen::VectorXd f;
  Eigen::MatrixXd A;
  Eigen::VectorXd lb;
  Eigen::VectorXd ub;
  int N;
  int M;

private:
  struct var
  {
    int size;
    int index;
  };

  Eigen::MatrixXd identity(int n)
  {
    return Eigen::MatrixXd::Identity(n, n);
  }

  Eigen::VectorXd zeros(int n)
  {
    return Eigen::VectorXd::Zero(n);
  }

  Eigen::VectorXd ones(int n)
  {
    return Eigen::VectorXd::Ones(n);
  }

  Eigen::VectorXd infinity(int n)
  {
    return Eigen::VectorXd::Constant(n, INFINITY);
  }

  std::map<std::string, var> vars_;
};

#endif  // QP_PROBLEM_HPP
