#include <gtest/gtest.h>
#include "climb_main/optimization/osqp_interface.hpp"
#include "climb_main/optimization/qp_problem.hpp"
#include "util/test_utils.hpp"

const double TOL = 1e-3;

TEST(OptimizerTest, OsqpInterface)
{
  // Setup problem:
  // Minimize 1/2 (x^2 + 2y^2) + 4x + y
  // Subject to 0 <= x <= inf, 0 < y <= inf, 3 <= x + y <= inf
  std::unique_ptr<QpInterface> solver = std::make_unique<OsqpInterface>();
  Eigen::MatrixXd H(2, 2);
  H << 1, 0,
    0, 2;
  Eigen::Vector2d f{4, 1};
  Eigen::MatrixXd A(3, 2);
  A << 1, 0,
    0, 1,
    1, 1;
  Eigen::Vector3d lb{0, 0, 3};
  Eigen::Vector3d ub{INFINITY, INFINITY, INFINITY};

  // Solve without sparsity structure
  ASSERT_TRUE(solver->solve(H, f, A, lb, ub)) << "Dense solve failed";
  Eigen::VectorXd sol = solver->getSolution();
  double cost = solver->getCost();
  Eigen::Vector2d sol_expected(1, 2);
  EXPECT_NEAR_EIGEN(sol, sol_expected, TOL) << "Incorrect dense solution";
  double cost_expected = 10.5;
  EXPECT_NEAR(cost, cost_expected, TOL) << "Incorrect dense solution cost";

  // Solve with sparsity structure
  Eigen::SparseMatrix<double> H_sparsity = H.sparseView();
  Eigen::SparseMatrix<double> A_sparsity = A.sparseView();
  ASSERT_TRUE(solver->solve(H, f, A, lb, ub, H_sparsity, A_sparsity)) <<
    "Sparse solve failed";
  sol = solver->getSolution();
  EXPECT_NEAR_EIGEN(sol, sol_expected, TOL) << "Incorrect sparse solution";
  cost = solver->getCost();
  EXPECT_NEAR(cost, cost_expected, TOL) << "Incorrect sparse solution cost";

  // Update problem
  ASSERT_TRUE(solver->update(H, f, A, lb, ub)) << "Update failed";
  sol = solver->getSolution();
  EXPECT_NEAR_EIGEN(sol, sol_expected, TOL) << "Incorrect updated solution";
  cost = solver->getCost();
  EXPECT_NEAR(cost, cost_expected, TOL) << "Incorrect updated solution cost";
}

TEST(OptimizerTest, QpProblem)
{
  QpProblem problem({{"x", "y", "z"}, {2, 3, 1}});
  problem.addQuadraticCost("y", 5, {});
  problem.addLinearCost("y", Eigen::Vector3d{1, 2, 3});
  problem.addLinearCost("y", Eigen::Vector3d{1, 2, 3});
  problem.addQuadraticCost(
    "x", "z", Eigen::MatrixXd::Constant(2, 1, 4), Eigen::Vector2d{1, 2}, {});

  Eigen::MatrixXd H_expected(6, 6);
  H_expected <<
    0, 0, 0, 0, 0, 2,
    0, 0, 0, 0, 0, 2,
    0, 0, 5, 0, 0, 0,
    0, 0, 0, 5, 0, 0,
    0, 0, 0, 0, 5, 0,
    2, 2, 0, 0, 0, 0;
  EXPECT_NEAR_EIGEN(problem.H, H_expected, TOL);
  Eigen::Vector<double, 6> f_expected{0, 0, 2, 4, 6, -12};
  EXPECT_NEAR_EIGEN(problem.f, f_expected, TOL);

  Eigen::MatrixXd Ay(2, 3);
  Ay << 1, 2, 3, 4, 5, 6;
  problem.addLinearConstraint(
    {"y", "x"}, {Ay, Eigen::MatrixXd::Identity(2, 2)},
    Eigen::Vector2d{1, 2}, {});
  problem.addBounds({"z"}, {}, Eigen::Vector<double, 1>{4});

  Eigen::MatrixXd A_expected(3, 6);
  A_expected <<
    1, 0, 1, 2, 3, 0,
    0, 1, 4, 5, 6, 0,
    0, 0, 0, 0, 0, 1;
  EXPECT_NEAR_EIGEN(problem.A, A_expected, TOL);
  Eigen::Vector3d lb_expected{1, 2, -INFINITY};
  EXPECT_EQ_EIGEN(problem.lb, lb_expected);
  Eigen::Vector3d ub_expected{INFINITY, INFINITY, 4};
  EXPECT_EQ_EIGEN(problem.ub, ub_expected);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
