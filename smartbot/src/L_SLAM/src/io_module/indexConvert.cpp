
#include "common/DynamicFeatureMap.h"
#include "common/pcl_util.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ros/ros.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace lidar_slam;

/** Main node entry point. */
int main(int argc, char **argv) {
  ros::init(argc, argv, "dynamic_index_convertor");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");
  std::string file = argv[1];
  int center_x = atoi(argv[2]);
  int center_y = atoi(argv[3]);
  int center_z = atoi(argv[4]);
  lidar_slam::DynamicFeatureMap<pcl::PointXYZI> _dynamic_feature_map;
  _dynamic_feature_map.convertIndexFile(file, center_x, center_y, center_z);

  return 0;
}
