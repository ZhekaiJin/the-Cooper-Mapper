#ifndef __TRAJECTORY_H__
#define __TRAJECTORY_H__

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
#include <ros/ros.h>

namespace lidar_slam {
//---------- Definitions ---------------------
inline void saveTrajectoryFile(int node_id, const ros::Time stamp,
                               const eigen::Isometry3d &pose) {
  g_file << g_frame_count << "\t" << odomAftMapped->header.stamp << "\t"
         << odomAftMapped->pose.pose.position.x << "\t"
         << odomAftMapped->pose.pose.position.y << "\t"
         << odomAftMapped->pose.pose.position.z << "\t"
         << odomAftMapped->pose.pose.orientation.w << "\t"
         << odomAftMapped->pose.pose.orientation.x << "\t"
         << odomAftMapped->pose.pose.orientation.y << "\t"
         << odomAftMapped->pose.pose.orientation.z << std::endl;
  std::cout << "frame && anchor ID: " << g_frame_count << "  " << g_frame_id
            << std::endl;
  g_frame_id++;
}

template <typename PointT>
void saveTrajectoryCloud(pcl::PointCloud<PointT> &cloud,
                         const std::string &directory,
                         const std::string &name) {
  if (!boost::filesystem::is_directory(directory)) {
    boost::filesystem::create_directory(directory);
  }
  std::string path = directory + "/" + name + ".pcd";
  pcl::io::savePCDFileASCII(path, cloud);
}

inline void saveTrajectoryFile(int node_id, const ros::Time stamp,
                               const eigen::Isometry3d &pose) {
  g_file << g_frame_count << "\t" << odomAftMapped->header.stamp << "\t"
         << odomAftMapped->pose.pose.position.x << "\t"
         << odomAftMapped->pose.pose.position.y << "\t"
         << odomAftMapped->pose.pose.position.z << "\t"
         << odomAftMapped->pose.pose.orientation.w << "\t"
         << odomAftMapped->pose.pose.orientation.x << "\t"
         << odomAftMapped->pose.pose.orientation.y << "\t"
         << odomAftMapped->pose.pose.orientation.z << std::endl;
  std::cout << "frame && anchor ID: " << g_frame_count << "  " << g_frame_id
            << std::endl;
  g_frame_id++;
}
} // namepace lidar_slam
#endif //__TRAJECTORY_H__