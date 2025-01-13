#ifndef CLIMB_FOOTSTEP_PLANNER__TERRAIN_GENERATOR_HPP_
#define CLIMB_FOOTSTEP_PLANNER__TERRAIN_GENERATOR_HPP_

#include <Eigen/Geometry>
#include <pcl/common/common.h>

/**
 * @brief Generate point clouds to represent geometric features
 */
namespace terrain
{
using pcl::PointXYZ;
using Eigen::Isometry3d;
using Eigen::Vector3d;
typedef pcl::PointCloud<PointXYZ> PointCloud;

/**
 * @brief Rectangular surface in the XY plane
 * @param pose Midpoint and orientation of the plane
 * @param xsize Width of the plane
 * @param ysize Length of the plane
 * @param res Resolution of the point cloud
 */
PointCloud plane(Isometry3d pose, double xsize, double ysize, double res);
PointCloud planeXY(Vector3d origin, double xsize, double ysize, double res);
PointCloud planeYZ(Vector3d origin, double ysize, double zsize, double res);
PointCloud planeZX(Vector3d origin, double zsize, double xsize, double res);

/**
 * @brief Circular disk in the XY plane
 * @param pose Midpoint and orientation of the disk
 * @param radius Radius of the disk
 * @param res Resolution of the point cloud
 */
PointCloud disk(Isometry3d pose, double radius, double res);
PointCloud diskXY(Vector3d origin, double radius, double res);
PointCloud diskYZ(Vector3d origin, double radius, double res);
PointCloud diskZX(Vector3d origin, double radius, double res);

/**
 * @brief Rectangular box aligned with the axes
 * @param pose Center and orientation of the box
 * @param xsize Width of the box
 * @param ysize Length of the box
 * @param zsize Height of the box
 * @param res Resolution of the point cloud
 */
PointCloud box(
  Isometry3d pose, double xsize, double ysize, double zsize, double res);
PointCloud box(
  Vector3d origin, double xsize, double ysize, double zsize, double res);

/**
 * @brief Open cylindrical tube oriented along its z-axis
 * @param pose Center and orientation of the tube
 * @param radius Radius of the tube
 * @param zsize Height of the tube
 * @param res Resolution of the point cloud
 */
PointCloud tube(Isometry3d pose, double radius, double zsize, double res);
PointCloud tubeX(Vector3d origin, double radius, double zsize, double res);
PointCloud tubeY(Vector3d origin, double radius, double zsize, double res);
PointCloud tubeZ(Vector3d origin, double radius, double zsize, double res);

/**
 * @brief Closed cylinder oriented along its z-axis
 * @param pose Center and orientation of the cylinder
 * @param radius Radius of the cylinder
 * @param zsize Height of the cylinder
 * @param res Resolution of the point cloud
 */
PointCloud cylinder(Isometry3d pose, double radius, double zsize, double res);
PointCloud cylinderX(Vector3d origin, double radius, double xsize, double res);
PointCloud cylinderY(Vector3d origin, double radius, double ysize, double res);
PointCloud cylinderZ(Vector3d origin, double radius, double zsize, double res);

/**
 * @brief Sphere
 * @param pose Center of the sphere
 * @param radius Radius of the sphere
 * @param res Resolution of the point cloud
 */
PointCloud sphere(Isometry3d pose, double radius, double res);
PointCloud sphere(Vector3d origin, double radius, double res);

/**
 * @brief Heightmap in the XY plane generated from fractal Perlin noise
 * @param pose Midpoint and orientation of the heightmap
 * @param xsize Width of the heightmap
 * @param ysize Length of the heightmap
 * @param amplitude Vertical scale of the heightmap
 * @param res Resolution of the point cloud
 */
PointCloud uneven(
  Isometry3d pose, double xsize, double ysize, double amplitude, double res);
PointCloud unevenXY(
  Vector3d origin, double xsize, double ysize, double amplitude, double res);
PointCloud unevenYZ(
  Vector3d origin, double ysize, double zsize, double amplitude, double res);
PointCloud unevenZX(
  Vector3d origin, double zsize, double xsize, double amplitude, double res);

}  // namespace terrain
#endif  // CLIMB_FOOTSTEP_PLANNER__TERRAIN_GENERATOR_HPP_
