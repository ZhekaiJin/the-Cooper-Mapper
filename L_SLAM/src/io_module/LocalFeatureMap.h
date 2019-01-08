#ifndef __LOCAL_FEATURE_MAP__
#define __LOCAL_FEATURE_MAP__

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <deque>
#include <fstream>
#include <sstream>
#include <string>

#include "common/math_utils.h"
#include "common/transform_utils.h"
#include "io/DataFrame.h"
#include "io/FrameUpdater.hpp"

namespace lidar_slam {

template <typename PointT> class LocalFeatureMap {
public:
  typedef typename pcl::PointCloud<PointT> PointCloud;
  typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
  typedef typename pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;
  typedef typename pcl::VoxelGrid<PointT> VoxelGrid;

  LocalFeatureMap() : queue_distance_threshold(30.0), _filesDirectory("~") {
    _downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);
    _downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);
    _downSizeFilterMap.setLeafSize(0.6, 0.6, 0.6);
  }
  ~LocalFeatureMap() {}

  inline void setupFilesDirectory(const std::string &filesDirectory) {
    _filesDirectory = filesDirectory;
  }

  void addDataFrame(DataFrame::Ptr &frame);
  void getSurroundFeature(PointCloudPtr &surroundCorner,
                          PointCloudPtr &surroundSurf);
  void clean();

private:
  std::deque<DataFrame::Ptr> data_queue;
  FrameUpdater frame_updater;

  std::string _filesDirectory; // cube_save
  double queue_distance_threshold;

  VoxelGrid
      _downSizeFilterCorner; ///< voxel filter for down sizing corner clouds
  VoxelGrid
      _downSizeFilterSurf; ///< voxel filter for down sizing surface clouds

  VoxelGrid
      _downSizeFilterMap; ///< voxel filter for down sizing full map clouds
};

template <typename PointT>
void LocalFeatureMap<PointT>::addDataFrame(DataFrame::Ptr &frame) {
  frame_updater.update(frame->odom);
  frame->setAccumDistance(frame_updater.get_accum_distance());
  frame->setFrameID(frame_updater.get_unique_id());
  data_queue.push_back(frame);

  clean();
}
template <typename PointT> void LocalFeatureMap<PointT>::clean() {
  int deleteNum = 0;
  for (const auto &frame : data_queue) {
    if (frame->accum_distance >
        (frame_updater.get_accum_distance() - queue_distance_threshold)) {
      break;
    }
    ++deleteNum;
  }
  if (deleteNum > 0)
    data_queue.erase(data_queue.begin(), data_queue.begin() + deleteNum + 1);
}

template <typename PointT>
inline void
LocalFeatureMap<PointT>::getSurroundFeature(PointCloudPtr &surroundCorner,
                                            PointCloudPtr &surroundSurf) {
  surroundCorner->clear();
  surroundSurf->clear();
//downsampling
  for (const auto &frame : data_queue) {
    *surroundCorner += *frame->cornerCloudDS;
    *surroundSurf += *frame->surfCloudDS;
  }
  _downSizeFilterCorner.setInputCloud(surroundCorner);
  _downSizeFilterCorner.filter(*surroundCorner);

  _downSizeFilterSurf.setInputCloud(surroundSurf);
  _downSizeFilterSurf.filter(*surroundSurf);
}
}

#endif //__LOCAL_FEATURE_MAP__
