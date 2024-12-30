#ifndef OSQP_INTERFACE_HPP
#define OSQP_INTERFACE_HPP

#include "climb_optimization/qp_interfaces/qp_interface.hpp"
#include <osqp.h>

/**
 * @brief Interface for the OSQP (Operator Splitting Quadratic Program) solver
 */
class OsqpInterface : public QpInterface
{
public:
  OsqpInterface();

  bool solve(
    const Eigen::MatrixXd & H,
    const Eigen::VectorXd & f,
    const Eigen::MatrixXd & A,
    const Eigen::VectorXd & b,
    const Eigen::MatrixXd & Aeq,
    const Eigen::VectorXd & beq,
    const Eigen::VectorXd & lb,
    const Eigen::VectorXd & ub);

  bool update(
    const Eigen::MatrixXd & H,
    const Eigen::VectorXd & f,
    const Eigen::MatrixXd & A,
    const Eigen::VectorXd & b,
    const Eigen::MatrixXd & Aeq,
    const Eigen::VectorXd & beq,
    const Eigen::VectorXd & lb,
    const Eigen::VectorXd & ub);

  void declareParameters() override;

  void setParameter(
    const Parameter & param, SetParametersResult & result) override;

  using QpInterface::setParameter;

private:
  /**
   * @brief Wrapper for csc matrix to handle deallocation
   */
  struct CscWrapper
  {
public:
    /**
     * @brief Default constructor
     */
    CscWrapper()
    : data_(nullptr) {}

    /**
     * @brief Create a csc matrix (see csc_matrix for parameter details)
     */
    CscWrapper(
      c_int m, c_int n, c_int nzmax,
      std::vector<c_float> x, std::vector<c_int> i, std::vector<c_int> p)
    : x_(x), i_(i), p_(p)
    {
      data_ = std::unique_ptr<csc>(
        csc_matrix(m, n, nzmax, x_.data(), i_.data(), p_.data()));
    }

    /**
     * @brief Get a raw pointer to the csc matrix
     */
    csc * get() const {return data_.get();}

private:
    std::unique_ptr<csc> data_;
    std::vector<c_float> x_;
    std::vector<c_int> i_;
    std::vector<c_int> p_;
  };

  /**
   * @brief Wrapper for OSQP data to handle deallocation
   */
  struct OSQPDataWrapper
  {
public:
    CscWrapper P;
    std::vector<c_float> q;
    CscWrapper A;
    std::vector<c_float> l;
    std::vector<c_float> u;

    /**
     * Return a raw pointer to the OSQP data
     */
    const OSQPData * get()
    {
      data.P = P.get();
      data.q = q.data();
      data.A = A.get();
      data.l = l.data();
      data.u = u.data();
      data.m = A.get()->m;
      data.n = A.get()->n;
      return &data;
    }

private:
    OSQPData data;
  };

  /**
   * @brief Wrapper for OSQP settings to handle deallocation
   */
  struct OSQPWorkspaceWrapper
  {
public:
    OSQPWorkspaceWrapper()
    : data_(nullptr) {}

    ~OSQPWorkspaceWrapper() {cleanup();}

    /**
     * @brief Setup the workspace with the given data and settings
     * @param data_input OSQP data
     * @param settings OSQP settings
     * @return Error code
     */
    c_int setup(const OSQPData * data_input, const OSQPSettings * settings)
    {
      cleanup();
      return osqp_setup(&data_, data_input, settings);
    }

    /**
     * @brief Get a raw pointer to the OSQP workspace
     */
    OSQPWorkspace * get() const {return data_;}

private:
    /**
     * @brief Deallocate memory
     */
    void cleanup()
    {
      if (data_) {
        osqp_cleanup(data_);
        data_ = nullptr;
      }
    }

    OSQPWorkspace * data_;
  };

  /**
   * @brief Convert Eigen matrix to csc matrix
   * @param mat Eigen matrix
   * @return csc matrix
   */
  CscWrapper matrixToOSQP(const Eigen::MatrixXd & mat);

  /**
   * @brief Convert Eigen vector to c_float array
   * @param vec Eigen vector
   * @return c_float array
   */
  std::vector<c_float> vectorToOSQP(const Eigen::VectorXd & vec);

  /**
   * @brief Convert c_float array to Eigen vector
   * @param vec c_float array
   * @param size Size of the array
   * @return Eigen vector
   */
  Eigen::VectorXd osqpToVector(const c_float * vec, c_int size);

  std::unique_ptr<OSQPSettings> settings_;      // OSQP settings
  OSQPDataWrapper data_;                        // OSQP data
  OSQPWorkspaceWrapper workspace_;              // OSQP workspace
};

#endif  // OSQP_INTERFACE_HPP
