#ifndef CLIMB_UTIL__GEOMETRY_UTILS_HPP_
#define CLIMB_UTIL__GEOMETRY_UTILS_HPP_

#include <Eigen/Geometry>

/**
 * @brief Utility classes for geometric representations
 */
namespace geometry_utils
{

/**
 * @brief A 3D polytope defined by a set of linear inequalities Ax <= b
 */
struct Polytope
{
  /**
   * @brief Construct an unbounded polytope
   */
  Polytope();

  /**
   * @brief Construct a polytope from a set of linear inequalities Ax <= b
   */
  Polytope(
    const Eigen::Matrix<double, Eigen::Dynamic, 3> & A,
    const Eigen::VectorXd & b);

  /**
   * @brief Construct an axis-aligned box polytope from lower and upper bounds
   */
  static Polytope createBox(
    const Eigen::Vector3d lb,
    const Eigen::Vector3d ub);

  Polytope(const Polytope & other) = default;
  Polytope(Polytope && other) = default;
  Polytope & operator=(const Polytope & other) = default;
  Polytope & operator=(Polytope && other) = default;

  /**
   * @brief Add a new half-plane constraint to the polytope
   */
  void addFacet(const Eigen::Vector3d & Ai, double bi);

  /**
   * @brief Compute the intersection with another polytope
   * (redundant constraints are not removed)
   */
  Polytope intersection(const Polytope & other) const;

  /**
   * @brief Compute the intersection with another polytope in place
   * (redundant constraints are not removed)
   */
  void intersect(const Polytope & other);

  /**
   * @brief Scale the polytope along each axis
   */
  void scale(const Eigen::Vector3d & scale);

  /**
   * @brief Compute the Minkowski sum with another polytope in place
   */
  Polytope & operator+=(const Polytope & other);
  Polytope & operator+=(const Eigen::Ref<const Eigen::Vector3d> & translation);
  Polytope & operator-=(const Eigen::Ref<const Eigen::Vector3d> & translation);
  Polytope & operator*=(double scale);
  Polytope & operator/=(double scale);
  Polytope operator-() const;

  /// @brief Constraint matrix
  Eigen::Matrix<double, Eigen::Dynamic, 3> A;
  /// @brief Constraint vector
  Eigen::VectorXd b;
  /// @brief Flag for axis-aligned box (easier computation)
  bool box = false;
};

Polytope operator+(const Polytope & p, const Polytope & other);
Polytope operator+(
  const Polytope & p, const Eigen::Ref<const Eigen::Vector3d> & translation);
Polytope operator+(
  const Eigen::Ref<const Eigen::Vector3d> & translation, const Polytope & p);
Polytope operator-(
  const Polytope & p, const Eigen::Ref<const Eigen::Vector3d> & translation);
Polytope operator-(
  const Eigen::Ref<const Eigen::Vector3d> & translation, const Polytope & p);
Polytope operator*(
  const Eigen::Ref<const Eigen::Matrix3d> & rotation, const Polytope & p);
Polytope operator*(const Eigen::Isometry3d & transform, const Polytope & p);
Polytope operator*(const Polytope & p, double scale);
Polytope operator*(double scale, const Polytope & p);
Polytope operator/(const Polytope & p, double scale);
}  // namespace geometry_utils

#endif  // CLIMB_UTIL__GEOMETRY_UTILS_HPP_
