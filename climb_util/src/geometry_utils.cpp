#include "climb_util/geometry_utils.hpp"

namespace geometry_utils
{
Polytope::Polytope()
{
  A = Eigen::MatrixXd(0, 3);
  b = Eigen::VectorXd(0);
}

Polytope::Polytope(
  const Eigen::MatrixX3d & A,
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
  b(b.size() - 1) = bi;
  box = false;
}

bool Polytope::contains(const Eigen::Vector3d & point) const
{
  return (b - A * point).minCoeff() >= 0;
}

Eigen::Vector<bool, Eigen::Dynamic> Polytope::containsAll(
  const Eigen::Matrix3Xd & points) const
{
  return ((-A * points).colwise() + b).colwise().minCoeff().array() >= 0;
}

double Polytope::distance(const Eigen::Vector3d & point) const
{
  Eigen::ArrayXd norm = A.rowwise().norm();
  return ((b - A * point).array() / norm).minCoeff();
}

Eigen::VectorXd Polytope::distanceAll(
  const Eigen::Matrix3Xd & points) const
{
  Eigen::ArrayXd norm = A.rowwise().norm();
  return (((-A * points).colwise() + b).array().colwise() / norm)
         .colwise().minCoeff();
}

double Polytope::distance(
  const Eigen::Vector3d & point, const Eigen::Vector3d & direction) const
{
  Eigen::ArrayXd scale = (A * direction.normalized());
  return (scale > 0).select(
    ((b - A * point).array() / scale), INFINITY).minCoeff();
}

Eigen::VectorXd Polytope::distanceAll(
  const Eigen::Matrix3Xd & points,
  const Eigen::Vector3d & direction) const
{
  Eigen::ArrayXd scale = A * direction.normalized();
  Eigen::ArrayXXd dist =
    ((-A * points).colwise() + b).array().colwise() / scale;
  return (scale > 0).replicate(1, points.cols())
         .select(dist, INFINITY).colwise().minCoeff();
}

Eigen::Vector3d Polytope::clip(
  const Eigen::Vector3d & point, const Eigen::Vector3d & direction)
{
  double d = distance(point, direction);
  return d >= 0 ? point : point + d * direction.normalized();
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
    b.head(3).array() *= scale.cwiseAbs().array();
    b.tail(3).array() *= scale.cwiseAbs().array();
    for (int i = 0; i < 3; ++i) {
      if (scale(i) < 0) {
        b.row(i).swap(b.row(i + 3));
      }
    }
  } else {
    Eigen::Vector3d s = scale;
    for (int i = 0; i < s.size(); ++i) {
      if (!s(i)) {
        s(i) = 1.0;
        addFacet(Eigen::Vector3d::Unit(i), 0);
        addFacet(-Eigen::Vector3d::Unit(i), 0);
      }
    }
    A = A * s.cwiseInverse().asDiagonal();
  }
}

Polytope Polytope::scaled(const Eigen::Vector3d & scale) const
{
  Polytope result(*this);
  result.scale(scale);
  return result;
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
  b *= abs(scale);
  if (scale < 0) {
    if (box) {
      b.topRows(3).swap(b.bottomRows(3));
    } else {
      A *= -1;
    }
  }
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
