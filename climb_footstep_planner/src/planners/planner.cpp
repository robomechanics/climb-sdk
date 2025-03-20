#include "climb_footstep_planner/planners/planner.hpp"

#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>

typedef pcl::PointCloud<pcl::Normal> NormalCloud;
typedef pcl::PointCloud<pcl::PrincipalCurvatures> CurvatureCloud;

void Planner::computeNormals(Eigen::Matrix3Xd & normals, double radius)
{
  auto nc = std::make_shared<NormalCloud>();
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(map_);
  auto kdtree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>(false);
  kdtree->setInputCloud(map_);
  ne.setSearchMethod(kdtree);
  ne.setRadiusSearch(radius);
  Eigen::Vector3d viewpoint = viewpoint_.translation();
  ne.setViewPoint(viewpoint(0), viewpoint(1), viewpoint(2));
  ne.compute(*nc);
  normals = nc->getMatrixXfMap(3, 8, 0).cast<double>().eval();
}

void Planner::computeCurvatures(
  Eigen::Matrix3Xd & tangents, Eigen::Matrix2Xd & curvatures,
  const Eigen::Matrix3Xd & normals, double radius)
{
  auto cc = std::make_shared<CurvatureCloud>();
  pcl::PrincipalCurvaturesEstimation<
    pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> pc;
  auto nc = std::make_shared<NormalCloud>();
  nc->resize(map_->size());
  nc->getMatrixXfMap(3, 8, 0) = normals.cast<float>();
  pc.setInputCloud(map_);
  pc.setInputNormals(nc);
  auto kdtree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>(false);
  kdtree->setInputCloud(map_);
  pc.setSearchMethod(kdtree);
  pc.setRadiusSearch(radius);
  pc.compute(*cc);
  tangents = cc->getMatrixXfMap().block(
    0, 0, 3, cc->getMatrixXfMap().cols()).cast<double>().eval();
  curvatures = cc->getMatrixXfMap().block(
    3, 0, 2, cc->getMatrixXfMap().cols()).cast<double>().eval();
}

void Planner::orientNormals(Eigen::Matrix3Xd & normals, int nn)
{
  if (nn == 0) {
    return;
  }
  auto kdtree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>(false);
  kdtree->setInputCloud(map_);
  Eigen::Vector3f viewpoint = viewpoint_.translation().cast<float>();
  std::vector<int> indices = {0};
  std::vector<float> distances = {0};
  kdtree->nearestKSearch(
    {viewpoint(0), viewpoint(1), viewpoint(2)}, 1, indices, distances);
  std::vector<int> queue = {indices.front()};
  std::vector<bool> visited(map_->size(), false);
  visited[queue.back()] = true;
  indices.resize(nn);
  distances.resize(nn);
  for (int i = 0; i < normals.cols(); ++i) {
    if (queue.size() == 0) {
      std::cout << "Checked normal orientations of " <<
        i << "/" << normals.cols() << " points" << std::endl;
      break;
    }
    int p = queue.back();
    queue.pop_back();
    kdtree->nearestKSearch(p, nn, indices, distances);
    for (const auto & q : indices) {
      if (visited[q]) {
        continue;
      }
      visited[q] = true;
      queue.push_back(q);
      if (normals.col(p).dot(normals.col(q)) < 0) {
        normals.col(q) *= -1;
      }
    }
  }
}
