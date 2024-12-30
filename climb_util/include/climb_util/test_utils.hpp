#ifndef TEST_UTILS_HPP
#define TEST_UTILS_HPP

const double PI = 3.14159265;

#define EXPECT_NEAR_EIGEN(A, B, tol) \
  EXPECT_TRUE((A).isApprox(B, tol)) << \
    #A << " =" << std::endl << A << std::endl << \
    #B << " =" << std::endl << B << std::endl

#define EXPECT_EQ_EIGEN(A, B) \
  EXPECT_TRUE(A == B) << \
    #A << " =" << std::endl << A << std::endl << \
    #B << " =" << std::endl << B << std::endl

#endif  // TEST_UTILS_HPP
