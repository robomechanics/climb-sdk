#include "climb_footstep_planner/terrain_generator.hpp"
#include <pcl/common/transforms.h>

namespace terrain
{
enum class Axes {XYZ, YZX, ZXY};

Isometry3d tranform(Vector3d origin, Axes axes = Axes::XYZ)
{
  Isometry3d pose = Isometry3d::Identity();
  pose.translate(origin);
  if (axes == Axes::YZX) {
    pose.rotate(
      Eigen::AngleAxisd(
        2 * M_PI / 3, Eigen::Vector3d(1, 1, 1).normalized()));
  } else if (axes == Axes::ZXY) {
    pose.rotate(
      Eigen::AngleAxisd(
        4 * M_PI / 3, Eigen::Vector3d(1, 1, 1).normalized()));
  }
  return pose;
}

PointCloud plane(Isometry3d pose, double xsize, double ysize, double res)
{
  PointCloud plane;
  plane.reserve(ceil(xsize / res) * ceil(ysize / res));
  for (double x = -xsize / 2; x < xsize / 2; x += res) {
    for (double y = -ysize / 2; y < ysize / 2; y += res) {
      plane.emplace_back(PointXYZ(x, y, 0));
    }
  }
  pcl::transformPointCloud(plane, plane, pose.matrix());
  return plane;
}

PointCloud planeXY(Vector3d origin, double xsize, double ysize, double res)
{
  return plane(tranform(origin, Axes::XYZ), xsize, ysize, res);
}

PointCloud planeYZ(Vector3d origin, double ysize, double zsize, double res)
{
  return plane(tranform(origin, Axes::YZX), ysize, zsize, res);
}

PointCloud planeZX(Vector3d origin, double zsize, double xsize, double res)
{
  return plane(tranform(origin, Axes::ZXY), zsize, xsize, res);
}

PointCloud disk(Isometry3d pose, double radius, double res)
{
  PointCloud disk;
  disk.reserve(ceil(2 * radius / res) * ceil(2 * radius / res));
  for (double x = -radius; x < radius; x += res) {
    for (double y = -radius; y < radius; y += res) {
      if (x * x + y * y < radius * radius) {
        disk.emplace_back(PointXYZ(x, y, 0));
      }
    }
  }
  pcl::transformPointCloud(disk, disk, pose.matrix());
  return disk;
}

PointCloud diskXY(Vector3d origin, double radius, double res)
{
  return disk(tranform(origin, Axes::XYZ), radius, res);
}

PointCloud diskYZ(Vector3d origin, double radius, double res)
{
  return disk(tranform(origin, Axes::YZX), radius, res);
}

PointCloud diskZX(Vector3d origin, double radius, double res)
{
  return disk(tranform(origin, Axes::ZXY), radius, res);
}

PointCloud box(
  Isometry3d pose, double xsize, double ysize, double zsize, double res)
{
  PointCloud box;
  box += planeXY({0, 0, zsize / 2}, xsize, ysize, res);
  box += planeXY({0, 0, -zsize / 2}, xsize, ysize, res);
  box += planeYZ({xsize / 2, 0, 0}, ysize, zsize, res);
  box += planeYZ({-xsize / 2, 0, 0}, ysize, zsize, res);
  box += planeZX({0, ysize / 2, 0}, zsize, xsize, res);
  box += planeZX({0, -ysize / 2, 0}, zsize, xsize, res);
  pcl::transformPointCloud(box, box, pose.matrix());
  return box;
}

PointCloud box(
  Vector3d origin, double xsize, double ysize, double zsize, double res)
{
  return box(tranform(origin, Axes::XYZ), xsize, ysize, zsize, res);
}

PointCloud tube(
  Isometry3d pose, double radius, double zsize, double res)
{
  PointCloud tube;
  tube.reserve(ceil(2 * M_PI * radius / res) * ceil(zsize / res));
  for (double z = -zsize / 2; z < zsize / 2; z += res) {
    for (double theta = 0; theta < 2 * M_PI; theta += res / radius) {
      tube.emplace_back(
        PointXYZ(radius * cos(theta), radius * sin(theta), z));
    }
  }
  pcl::transformPointCloud(tube, tube, pose.matrix());
  return tube;
}

PointCloud tubeX(Vector3d origin, double radius, double xsize, double res)
{
  return tube(tranform(origin, Axes::YZX), radius, xsize, res);
}

PointCloud tubeY(Vector3d origin, double radius, double ysize, double res)
{
  return tube(tranform(origin, Axes::ZXY), radius, ysize, res);
}

PointCloud tubeZ(Vector3d origin, double radius, double zsize, double res)
{
  return tube(tranform(origin, Axes::XYZ), radius, zsize, res);
}

PointCloud cylinder(
  Isometry3d pose, double radius, double zsize, double res)
{
  PointCloud cylinder;
  cylinder += tubeZ({0, 0, 0}, radius, zsize, res);
  cylinder += diskXY({0, 0, -zsize / 2}, radius, res);
  cylinder += diskXY({0, 0, zsize / 2}, radius, res);
  pcl::transformPointCloud(cylinder, cylinder, pose.matrix());
  return cylinder;
}

PointCloud cylinderX(
  Vector3d origin, double radius, double xsize, double res)
{
  return cylinder(tranform(origin, Axes::YZX), radius, xsize, res);
}

PointCloud cylinderY(
  Vector3d origin, double radius, double ysize, double res)
{
  return cylinder(tranform(origin, Axes::ZXY), radius, ysize, res);
}

PointCloud cylinderZ(
  Vector3d origin, double radius, double zsize, double res)
{
  return cylinder(tranform(origin, Axes::XYZ), radius, zsize, res);
}

PointCloud sphere(Isometry3d pose, double radius, double res)
{
  PointCloud sphere;
  int N = ceil(4 * M_PI * radius * radius / (res * res));
  sphere.reserve(N);
  for (int i = 0; i < N; i++) {
    double phi = acos(1.0 - 2.0 * i / N);
    double theta = M_PI * (1.0 + sqrt(5)) * i;
    double x = radius * cos(theta) * sin(phi);
    double y = radius * sin(theta) * sin(phi);
    double z = radius * cos(phi);
    sphere.emplace_back(PointXYZ(x, y, z));
  }
  pcl::transformPointCloud(sphere, sphere, pose.matrix());
  return sphere;
}

PointCloud sphere(
  Vector3d origin, double radius, double res)
{
  return sphere(tranform(origin, Axes::XYZ), radius, res);
}

static double smoothStep(double x)
{
  return 3 * x * x - 2 * x * x * x;
}

static Eigen::ArrayXXd perlin(int w, int h, int s)
{
  Eigen::ArrayXXd noise = Eigen::ArrayXXd::Zero(w, h);
  Eigen::ArrayXXd theta = Eigen::ArrayXXd::Random(w / s + 2, h / s + 2);
  Eigen::ArrayXXd gx = cos(theta * M_PI);
  Eigen::ArrayXXd gy = sin(theta * M_PI);
  for (int x = 0; x < w; x++) {
    for (int y = 0; y < h; y++) {
      int i = x / s;
      int j = y / s;
      double dx = double(x % s) / s;
      double dy = double(y % s) / s;
      noise(x, y) += (gx(i, j) * dx + gy(i, j) * dy) *
        smoothStep(1 - dx) * smoothStep(1 - dy);
      noise(x, y) += (gx(i + 1, j) * (dx - 1) + gy(i + 1, j) * dy) *
        smoothStep(dx) * smoothStep(1 - dy);
      noise(x, y) += (gx(i, j + 1) * dx + gy(i, j + 1) * (dy - 1)) *
        smoothStep(1 - dx) * smoothStep(dy);
      noise(x, y) += (gx(i + 1, j + 1) * (dx - 1) +
        gy(i + 1, j + 1) * (dy - 1)) *
        smoothStep(dx) * smoothStep(dy);
    }
  }
  return noise * s;
}

static Eigen::ArrayXXd fractal(int w, int h)
{
  Eigen::ArrayXXd noise = Eigen::ArrayXXd::Zero(w, h);
  int s = 8;
  while (s < w && s < h) {
    noise += perlin(w, h, s);
    s *= 2;
  }
  return noise;
}

PointCloud uneven(
  Isometry3d pose, double xsize, double ysize, double amplitude, double res)
{
  PointCloud uneven;
  Eigen::ArrayXXd noise = fractal(ceil(xsize / res), ceil(ysize / res)) * res;
  uneven.reserve(noise.rows() * noise.cols());
  for (int i = 0; i < noise.rows(); i++) {
    for (int j = 0; j < noise.cols(); j++) {
      uneven.emplace_back(
        PointXYZ(
          i * res - xsize / 2, j * res - ysize / 2, noise(i, j) * amplitude));
    }
  }
  pcl::transformPointCloud(uneven, uneven, pose.matrix());
  return uneven;
}

PointCloud unevenXY(
  Vector3d origin, double xsize, double ysize, double amplitude, double res)
{
  return uneven(tranform(origin, Axes::XYZ), xsize, ysize, amplitude, res);
}

PointCloud unevenYZ(
  Vector3d origin, double ysize, double zsize, double amplitude, double res)
{
  return uneven(tranform(origin, Axes::YZX), ysize, zsize, amplitude, res);
}

PointCloud unevenZX(
  Vector3d origin, double zsize, double xsize, double amplitude, double res)
{
  return uneven(tranform(origin, Axes::ZXY), zsize, xsize, amplitude, res);
}

}  // namespace terrain
