#ifndef CLIMB_OPTIMIZATION__QP_PROBLEM_HPP_
#define CLIMB_OPTIMIZATION__QP_PROBLEM_HPP_

#include <Eigen/Core>
#include <map>
#include <string>
#include <vector>

/**
 * @brief Quadratic program problem definition
 */
class QpProblem
{
public:
  /**
   * @brief Construct a new QP problem
   * @param vars The names of the optimization variables
   * @param sizes The sizes of the optimization variables
   */
  QpProblem(std::vector<std::string> vars, std::vector<int> sizes);

  /**
   * @brief Define an alias for a contiguous set of optimization variables
   * @param name The name of the alias
   * @param index The starting index of the alias
   * @param size The number of elements in the alias
   */
  void addAlias(const std::string & name, int index, int size)
  {
    vars_[name] = {size, index};
  }

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
   * @brief Add a linear inequality constraint of the form: Ax <= b
   * @param vars The names of the variables
   * @param A_in The constraint matrix for each variable
   * @param b_in The upper bound of the constraint
   */
  void addInequalityConstraint(
    const std::vector<std::string> & vars,
    const std::vector<Eigen::MatrixXd> & A_in,
    const Eigen::VectorXd & b_in);

  /**
   * @brief Add a linear inequality constraint of the form: Ax <= b
   * @param vars The names of the variables
   * @param A_in The constraint matrix for each variable
   * @param b_in The upper bound of the constraint
   */
  void addInequalityConstraint(
    const std::vector<std::string> & vars,
    const std::vector<Eigen::MatrixXd> & A_in,
    double b_in)
  {
    addInequalityConstraint(
      vars, A_in, Eigen::VectorXd::Constant(A_in.at(0).rows(), b_in));
  }

  /**
   * @brief Add a linear inequality constraint of the form: lb <= Ax <= ub
   * @param vars The names of the variables
   * @param A_in The constraint matrix for each variable
   * @param lb_in The lower bound of the constraint
   * @param ub_in The upper bound of the constraint
   */
  void addInequalityConstraint(
    const std::vector<std::string> & vars,
    const std::vector<Eigen::MatrixXd> & A_in,
    const Eigen::VectorXd & lb_in,
    const Eigen::VectorXd & ub_in)
  {
    if (lb_in.size()) {
      std::vector<Eigen::MatrixXd> A_neg;
      for (const auto & A_i : A_in) {
        A_neg.push_back(-A_i);
      }
      addInequalityConstraint(vars, A_neg, -lb_in);
    }
    if (ub_in.size()) {
      addInequalityConstraint(vars, A_in, ub_in);
    }
  }

  /**
   * @brief Add a linear inequality constraint of the form: lb <= Ax <= ub
   * @param vars The names of the variables
   * @param A_in The constraint matrix for each variable
   * @param lb_in The lower bound of the constraint
   * @param ub_in The upper bound of the constraint
   */
  void addInequalityConstraint(
    const std::vector<std::string> & vars,
    const std::vector<Eigen::MatrixXd> & A_in,
    double lb_in,
    double ub_in)
  {
    addInequalityConstraint(
      vars, A_in, Eigen::VectorXd::Constant(A_in.at(0).rows(), lb_in),
      Eigen::VectorXd::Constant(A_in.at(0).rows(), ub_in));
  }

  /**
   * @brief Add a linear equality constraint of the form: Aeq x = beq
   * @param vars The names of the variables
   * @param Aeq_in The constraint matrix for each variable
   * @param beq_in The value of the constraint
   */
  void addEqualityConstraint(
    const std::vector<std::string> & vars,
    const std::vector<Eigen::MatrixXd> & Aeq_in,
    const Eigen::VectorXd & beq_in);

  /**
   * @brief Add a linear equality constraint of the form: Aeq x = beq
   * @param vars The names of the variables
   * @param Aeq_in The constraint matrix for each variable
   * @param beq_in The value of the constraint
   */
  void addEqualityConstraint(
    const std::vector<std::string> & vars,
    const std::vector<Eigen::MatrixXd> & Aeq_in,
    double beq_in)
  {
    addEqualityConstraint(
      vars, Aeq_in, Eigen::VectorXd::Constant(Aeq_in.at(0).rows(), beq_in));
  }

  /**
   * @brief Add a linear constraint of the form: lb <= x <= ub
   * @param var The name of the variable
   * @param lb_in The lower bound of the constraint (or empty for -infinity)
   * @param ub_in The upper bound of the constraint (or empty for infinity)
   */
  void addBounds(
    const std::string & var,
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
    const double lb_in,
    const double ub_in)
  {
    addBounds(
      var, lb_in * ones(vars_.at(var).size), ub_in * ones(vars_.at(var).size));
  }

  Eigen::MatrixXd H;
  Eigen::VectorXd f;
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
  Eigen::MatrixXd Aeq;
  Eigen::VectorXd beq;
  Eigen::VectorXd lb;
  Eigen::VectorXd ub;
  int N;
  int M;
  int Meq;

private:
  struct Var
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

  std::map<std::string, Var> vars_;
};

#endif  // CLIMB_OPTIMIZATION__QP_PROBLEM_HPP_
