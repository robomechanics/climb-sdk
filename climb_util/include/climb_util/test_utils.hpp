#ifndef CLIMB_UTIL__TEST_UTILS_HPP_
#define CLIMB_UTIL__TEST_UTILS_HPP_

const double PI = 3.14159265;

#define EXPECT_NEAR_EIGEN(A, B, tol) \
  EXPECT_TRUE((A).isApprox(B, tol)) << \
    #A << " =" << std::endl << A << std::endl << \
    #B << " =" << std::endl << B << std::endl

#define EXPECT_EQ_EIGEN(A, B) \
  EXPECT_TRUE(A == B) << \
    #A << " =" << std::endl << A << std::endl << \
    #B << " =" << std::endl << B << std::endl

#endif  // CLIMB_UTIL__TEST_UTILS_HPP_
