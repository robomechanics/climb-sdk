#include "climb_util/geometry_utils.hpp"

namespace geometry_utils
{
Polytope::Polytope()
{
  A = Eigen::MatrixXd(0, 3);
  b = Eigen::VectorXd(0);
}

Polytope::Polytope(
  const Eigen::Matrix<double, Eigen::Dynamic, 3> & A,
  const Eigen::VectorXd & b)
: A(A), b(b)
{
  if (A.rows() != b.size()) {
    throw std::invalid_argument("Polytope: A and b must have same row count");
  }
}

Polytope Polytope::createBox(
  const Eigen::Vector3d lb,
  const Eigen::Vector3d ub)
{
  Eigen::MatrixXd A(6, 3);
  A << -Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity();
  Eigen::VectorXd b(6);
  b << -lb, ub;
  Polytope p(A, b);
  p.box = true;
  return p;
}

void Polytope::addFacet(const Eigen::Vector3d & Ai, double bi)
{
  A.conservativeResize(A.rows() + 1, Eigen::NoChange);
  A.row(A.rows() - 1) = Ai;
  b.conservativeResize(b.size() + 1);
  b[b.size() - 1] = bi;
  box = false;
}

void Polytope::intersect(const Polytope & other)
{
  if (box && other.box) {
    b = b.cwiseMin(other.b);
  } else {
    A.conservativeResize(A.rows() + other.A.rows(), Eigen::NoChange);
    A.bottomRows(other.A.rows()) = other.A;
    b.conservativeResize(b.size() + other.b.size());
    b.tail(other.b.size()) = other.b;
    box = false;
  }
}

Polytope Polytope::intersection(const Polytope & other) const
{
  Polytope result(*this);
  result.intersect(other);
  return result;
}

void Polytope::scale(const Eigen::Vector3d & scale)
{
  if (box) {
    b.head(3).array() *= scale.array();
    b.tail(3).array() *= scale.array();
  } else {
    Eigen::Vector3d s = scale;
    for (int i = 0; i < s.size(); ++i) {
      if (!s[i]) {
        s[i] = 1.0;
        addFacet(Eigen::Vector3d::Unit(i), 0);
        addFacet(-Eigen::Vector3d::Unit(i), 0);
      }
    }
    A = A * s.cwiseInverse().asDiagonal();
  }
}

Polytope & Polytope::operator+=(const Polytope & other)
{
  if (box && other.box) {
    b += other.b;
    return *this;
  } else {
    // TODO: Implement Minkowski sum for general polytopes
    throw std::runtime_error("Polytope addition not yet implemented");
  }
}

Polytope & Polytope::operator+=(
  const Eigen::Ref<const Eigen::Vector3d> & translation)
{
  b += A * translation;
  return *this;
}

Polytope & Polytope::operator*=(double scale)
{
  b *= scale;
  return *this;
}

Polytope & Polytope::operator-=(
  const Eigen::Ref<const Eigen::Vector3d> & translation)
{
  return *this += -translation;
}

Polytope & Polytope::operator/=(double scale)
{
  return *this *= 1.0 / scale;
}

Polytope Polytope::operator-() const
{
  return Polytope(*this) *= -1.0;
}

Polytope operator+(const Polytope & p, const Polytope & other)
{
  return Polytope(p) += other;
}

Polytope operator+(
  const Polytope & p, const Eigen::Ref<const Eigen::Vector3d> & translation)
{
  return Polytope(p) += translation;
}

Polytope operator+(
  const Eigen::Ref<const Eigen::Vector3d> & translation, const Polytope & p)
{
  return p + translation;
}

Polytope operator-(
  const Polytope & p, const Eigen::Ref<const Eigen::Vector3d> & translation)
{
  return Polytope(p) -= translation;
}

Polytope operator-(
  const Eigen::Ref<const Eigen::Vector3d> & translation, const Polytope & p)
{
  return -p + translation;
}

Polytope operator*(
  const Eigen::Ref<const Eigen::Matrix3d> & rotation, const Polytope & p)
{
  return Polytope(p.A * rotation.transpose(), p.b);
}

Polytope operator*(const Eigen::Isometry3d & transform, const Polytope & p)
{
  return transform.linear() * p + transform.translation();
}

Polytope operator*(const Polytope & p, double scale)
{
  return Polytope(p) *= scale;
}

Polytope operator*(double scale, const Polytope & p)
{
  return p * scale;
}

Polytope operator/(const Polytope & p, double scale)
{
  return Polytope(p) /= scale;
}
}  // namespace geometry_utils
