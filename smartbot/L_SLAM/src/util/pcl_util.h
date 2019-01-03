#ifndef __PCL_UTIL_H__
#define __PCL_UTIL_H__

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/boundary.h>
#include <pcl/features/eigen.h>
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>

#include <pcl/segmentation/region_growing.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "voxel_grid_partition.hpp"

namespace lidar_slam {

inline pcl::PointXYZI toXYZI(const pcl::PointXYZINormal &pi) {
  pcl::PointXYZI po;
  po.x = pi.x;
  po.y = pi.y;
  po.z = pi.z;
  po.intensity = pi.curvature;
  return po;
}

inline void voxelFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out,
                        double voxelSize, int minumPoints) {
  pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
  voxelGrid.setDownsampleAllData(false);
  voxelGrid.setLeafSize(voxelSize, voxelSize, voxelSize);
  voxelGrid.setMinimumPointsNumberPerVoxel(minumPoints);
  voxelGrid.setInputCloud(cloud_in);
  voxelGrid.filter(*cloud_out);
  PCL_DEBUG("voxelFilter size:%d\n", cloud_out->size());
}

inline void
voxelPartition(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
               std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &pc_vector,
               double voxelSize, int minumPoints) {
  pcl::VoxelGridPartition<pcl::PointXYZ> voxelGridPartition;
  voxelGridPartition.setDownsampleAllData(false);
  voxelGridPartition.setLeafSize(voxelSize, voxelSize, voxelSize);
  voxelGridPartition.setMinimumPointsNumberPerVoxel(minumPoints);
  voxelGridPartition.setInputCloud(cloud_in);
  voxelGridPartition.compute(pc_vector);
  PCL_DEBUG("VoxelGridPartition size:%d\n", pc_vector.size());
}

inline void
approximateVoxelFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out,
                       double voxelSize) {

  pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxelGrid;
  voxelGrid.setDownsampleAllData(false);
  voxelGrid.setLeafSize(voxelSize, voxelSize, voxelSize);
  voxelGrid.setInputCloud(cloud_in);
  voxelGrid.filter(*cloud_out);
  PCL_DEBUG("approximateVoxelFilter size:%d\n", cloud_out->size());
}

inline void radiusOutlierFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out,
                                int k_neighbor, double radius) {
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusOutlierRemoval(false);
  radiusOutlierRemoval.setInputCloud(cloud_in);

  radiusOutlierRemoval.setRadiusSearch(radius);
  radiusOutlierRemoval.setMinNeighborsInRadius(k_neighbor);
  radiusOutlierRemoval.filter(*cloud_out);
  PCL_DEBUG("radiusOutlierFilter size:%d\n", cloud_out->size());
}

inline void normalEstimate(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
                           pcl::PointCloud<pcl::Normal>::Ptr &normals,
                           int k_neighbor,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_origin) {
  pcl::search::Search<pcl::PointXYZ>::Ptr tree =
      boost::shared_ptr<pcl::search::Search<pcl::PointXYZ>>(
          new pcl::search::KdTree<pcl::PointXYZ>);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod(tree);
  normal_estimator.setInputCloud(cloud_in);
  normal_estimator.setKSearch(k_neighbor);
  // Pass the original data (before downsampling) as the search surface
  normal_estimator.setSearchSurface(cloud_origin);
  normal_estimator.compute(*normals);
  PCL_DEBUG("normals size:%d\n", normals->size());
}

inline void normalEstimate(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
                           pcl::PointCloud<pcl::Normal>::Ptr &normals,
                           double radius,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_origin) {
  pcl::search::Search<pcl::PointXYZ>::Ptr tree =
      boost::shared_ptr<pcl::search::Search<pcl::PointXYZ>>(
          new pcl::search::KdTree<pcl::PointXYZ>);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod(tree);
  normal_estimator.setInputCloud(cloud_in);
  // Pass the original data (before downsampling) as the search surface
  normal_estimator.setSearchSurface(cloud_origin);
  normal_estimator.setRadiusSearch(radius);
  normal_estimator.compute(*normals);
  PCL_DEBUG("normals size:%d\n", normals->size());
}

inline void boundaryEstimate(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
                             pcl::PointCloud<pcl::Normal>::Ptr &normals,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out) {
  pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> bound_est;
  pcl::PointCloud<pcl::Boundary> boundaries;

  bound_est.setInputCloud(cloud_in);
  bound_est.setInputNormals(normals);
  bound_est.setAngleThreshold(3.14159 / 2.0 * 0.9);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>());
  bound_est.setSearchMethod(tree);
  // bound_est.setKSearch(50);
  bound_est.setRadiusSearch(0.1);
  bound_est.compute(boundaries);

  int countBoundaries = 0;
  for (int i = 0; i < cloud_in->points.size(); i++) {
    uint8_t x = (boundaries.points[i].boundary_point);
    int a = static_cast<int>(x);
    if (a == 1) {
      cloud_out->push_back(cloud_in->points[i]);
      countBoundaries++;
    }
  }
  PCL_DEBUG("boundaries points size:%d\n", cloud_out->points.size());
}

inline void plannarEstimate(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
                            pcl::PointCloud<pcl::Normal>::Ptr &normals,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out) {
  pcl::search::Search<pcl::PointXYZ>::Ptr tree =
      boost::shared_ptr<pcl::search::Search<pcl::PointXYZ>>(
          new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize(50);
  reg.setMaxClusterSize(1000000);
  reg.setSearchMethod(tree);
  reg.setNumberOfNeighbours(60);
  reg.setInputCloud(cloud_in);
  // reg.setIndices (indices);
  reg.setInputNormals(normals);
  reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold(1.0);

  std::vector<pcl::PointIndices> clusters;
  reg.extract(clusters);
  PCL_DEBUG("clusters size:%d\n", clusters.size());

  for (int i = 0; i < clusters.size(); ++i) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud_in, clusters[i], *cloud);
    *cloud_out += *cloud;
    cloud->clear();
  }
  PCL_DEBUG("plannar points size:%d\n", cloud_out->points.size());
}
} // namepace lidar_slam
#endif