#include "climb_util/geometry_utils.hpp"
#include <vector>

namespace geometry_utils
{
Polytope::Polytope()
{
  A = Eigen::MatrixXd();
  b = Eigen::VectorXd();
}

Polytope::Polytope(
  const Eigen::MatrixXd & A,
  const Eigen::VectorXd & b)
: A(A), b(b)
{
  assert(A.rows() == b.size());
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

void Polytope::addFacet(const Eigen::VectorXd & Ai, double bi)
{
  assert(!A.rows() || Ai.size() == A.cols());
  A.conservativeResize(A.rows() + 1, Ai.size());
  A.row(A.rows() - 1) = Ai;
  b.conservativeResize(b.size() + 1);
  b(b.size() - 1) = bi;
  box = false;
}

bool Polytope::contains(const Eigen::VectorXd & point) const
{
  assert(A.cols() == point.size());
  return (b - A * point).minCoeff() >= 0;
}

Eigen::Vector<bool, Eigen::Dynamic> Polytope::containsAll(
  const Eigen::MatrixXd & points) const
{
  assert(A.cols() == points.rows());
  return ((-A * points).colwise() + b).colwise().minCoeff().array() >= 0;
}

double Polytope::distance(const Eigen::VectorXd & point) const
{
  assert(A.cols() == point.size());
  Eigen::ArrayXd norm = A.rowwise().norm();
  return ((b - A * point).array() / norm).minCoeff();
}

Eigen::VectorXd Polytope::distanceAll(
  const Eigen::MatrixXd & points) const
{
  assert(A.cols() == points.rows());
  Eigen::ArrayXd norm = A.rowwise().norm();
  return (((-A * points).colwise() + b).array().colwise() / norm)
         .colwise().minCoeff();
}

double Polytope::distance(
  const Eigen::VectorXd & point, const Eigen::VectorXd & direction) const
{
  assert(A.cols() == point.size() && A.cols() == direction.size());
  Eigen::ArrayXd scale = (A * direction.normalized());
  return (scale > 0).select(
    ((b - A * point).array() / scale), INFINITY).minCoeff();
}

Eigen::VectorXd Polytope::distanceAll(
  const Eigen::MatrixXd & points,
  const Eigen::VectorXd & direction) const
{
  assert(A.cols() == points.rows() && A.cols() == direction.size());
  Eigen::ArrayXd scale = A * direction.normalized();
  Eigen::ArrayXXd dist =
    ((-A * points).colwise() + b).array().colwise() / scale;
  return (scale > 0).replicate(1, points.cols())
         .select(dist, INFINITY).colwise().minCoeff();
}

Eigen::VectorXd Polytope::clip(
  const Eigen::VectorXd & point, const Eigen::VectorXd & direction)
{
  assert(A.cols() == point.size() && A.cols() == direction.size());
  double d = distance(point, direction);
  return d >= 0 ? point : point + d * direction.normalized();
}

void Polytope::intersect(const Polytope & other)
{
  assert(!A.rows() || !other.A.rows() || A.cols() == other.A.cols());
  if (box && other.box) {
    b = b.cwiseMin(other.b);
  } else if (!A.rows() && other.A.rows()) {
    A = other.A;
    b = other.b;
    box = other.box;
  } else if (other.A.rows()) {
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

void Polytope::scale(const Eigen::VectorXd & scale)
{
  assert(A.cols() == scale.size());
  if (box) {
    b.head(3).array() *= scale.cwiseAbs().array();
    b.tail(3).array() *= scale.cwiseAbs().array();
    for (int i = 0; i < 3; ++i) {
      if (scale(i) < 0) {
        b.row(i).swap(b.row(i + 3));
      }
    }
  } else {
    Eigen::VectorXd s = scale;
    for (int i = 0; i < s.size(); ++i) {
      if (!s(i)) {
        s(i) = 1.0;
        addFacet(Eigen::VectorXd::Unit(s.size(), i), 0);
        addFacet(-Eigen::VectorXd::Unit(s.size(), i), 0);
      }
    }
    A = A * s.cwiseInverse().asDiagonal();
  }
}

void Polytope::eliminate(int index)
{
  assert(index >= 0 && index < A.cols());
  if (box) {
    Eigen::MatrixXd A_new = Eigen::MatrixXd::Zero(4, 2);
    Eigen::VectorXd b_new = Eigen::VectorXd::Zero(4);
    int j = 0;
    for (int i = 0; i < 3; ++i) {
      if (i != index) {
        A_new.row(j) <<
          A.row(i).head(index), A.row(i).tail(A.cols() - index - 1);
        A_new.row(j + 2) <<
          A.row(i + 3).head(index), A.row(i + 3).tail(A.cols() - index - 1);
        b_new(j) = b(i);
        b_new(j + 2) = b(i + 3);
        ++j;
      }
    }
    A = A_new;
    b = b_new;
    return;
  }
  // Sort rows by sign of eliminated variable's coefficient
  std::vector<int> i_p, i_m, i_z;
  for (int i = 0; i < A.rows(); ++i) {
    if (A(i, index) > 0) {
      i_p.push_back(i);
    } else if (A(i, index) < 0) {
      i_m.push_back(i);
    } else {
      i_z.push_back(i);
    }
  }
  // Eliminate variable using Fourier-Motzkin
  Eigen::VectorXd newB(i_p.size() * i_m.size() + i_z.size());
  Eigen::MatrixXd newA(newB.size(), A.cols());
  int j = 0;
  for (int p : i_p) {
    for (int n : i_m) {
      newA.row(j) = (A.row(p) / A(p, index) - A.row(n) / A(n, index));
      newB(j) = b(p) / A(p, index) - b(n) / A(n, index);
      ++j;
    }
  }
  for (int z : i_z) {
    newA.row(j) = A.row(z);
    newB(j) = b(z);
    ++j;
  }
  // Remove the eliminated variable's column
  A = Eigen::MatrixXd(newA.rows(), newA.cols() - 1);
  A << newA.leftCols(index), newA.rightCols(newA.cols() - index - 1);
  b = newB;
}

Polytope Polytope::eliminated(int index) const
{
  Polytope result(*this);
  result.eliminate(index);
  return result;
}

Polytope Polytope::scaled(const Eigen::VectorXd & scale) const
{
  Polytope result(*this);
  result.scale(scale);
  return result;
}

Polytope & Polytope::operator+=(const Polytope & other)
{
  assert(A.cols() == other.A.cols());
  if (box && other.box) {
    b += other.b;
    return *this;
  } else {
    // TODO: Implement Minkowski sum for general polytopes
    throw std::runtime_error("Polytope addition not yet implemented");
  }
}

Polytope & Polytope::operator+=(
  const Eigen::Ref<const Eigen::VectorXd> & translation)
{
  assert(A.cols() == translation.size());
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
  const Eigen::Ref<const Eigen::VectorXd> & translation)
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
  const Polytope & p, const Eigen::Ref<const Eigen::VectorXd> & translation)
{
  return Polytope(p) += translation;
}

Polytope operator+(
  const Eigen::Ref<const Eigen::VectorXd> & translation, const Polytope & p)
{
  return p + translation;
}

Polytope operator-(
  const Polytope & p, const Eigen::Ref<const Eigen::VectorXd> & translation)
{
  return Polytope(p) -= translation;
}

Polytope operator-(
  const Eigen::Ref<const Eigen::VectorXd> & translation, const Polytope & p)
{
  return -p + translation;
}

Polytope operator*(
  const Eigen::Ref<const Eigen::Matrix3d> & rotation, const Polytope & p)
{
  assert(p.A.cols() == 3);
  return Polytope(p.A * rotation.transpose(), p.b);
}

Polytope operator*(const Eigen::Isometry3d & transform, const Polytope & p)
{
  assert(p.A.cols() == 3);
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
