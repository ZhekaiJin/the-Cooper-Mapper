
#include "common/FeatureMap.h"
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
inline std::string fileNameFormat(const std::string filesDirectory,
                                  int number) {
  std::stringstream ss;
  std::string str;
  ss << number;
  ss >> str;
  str += ".pcd";
  return filesDirectory + str;
}

/** Main node entry point. */
int main(int argc, char **argv) {
  ros::init(argc, argv, "FeatureExtracter");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  std::string _filesDirectory;
  if (privateNode.getParam("filesDirectory", _filesDirectory)) {
    ROS_INFO("Set filesDirectory: %s", _filesDirectory.c_str());
  } else {
    _filesDirectory = ".";
    ROS_INFO("use default filesDirectory: %s", _filesDirectory.c_str());
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1) {
    PCL_ERROR("COULD NOT READ FILE \n");
    return (-1);
  }
  PCL_INFO("Input points size :%d \n", cloud->size());

  FeatureMap<pcl::PointXYZ> _featureCloud(21, 21, 21);
  _featureCloud.setupWorldOrigin(10, 5, 10);
  _featureCloud.setupWorldCubeSize(50.0);
  _featureCloud.setupFilesDirectory(_filesDirectory);

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pc_vector;
  voxelPartition(cloud, pc_vector, 50.0, 1000);
  cloud->clear();
  /*
  for(int i = 0; i< pc_vector.size();++i){
    pcl::io::savePCDFileASCII(fileNameFormat("./slip", i), *pc_vector[i]);
  }*/

  int pc_size = pc_vector.size();
#pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < pc_size; ++i) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtercloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    voxelFilter(pc_vector[i], filtercloud, 0.05, 3);

    /*
    if(!filtercloud->empty())
      pcl::io::savePCDFileASCII( fileNameFormat("./voxelFilter", i),
    *filtercloud);


    pcl::PointCloud<pcl::PointXYZ>::Ptr filtercloud (new
    pcl::PointCloud<pcl::PointXYZ>);
    radiusOutlierFilter(filtercloud, filtercloud, 0.5, 25);
    filtercloud->clear();

    if(!filtercloud->empty())
      pcl::io::savePCDFileASCII( fileNameFormat("./radiusOutlierFilter", i),
    *filtercloud);
    */

    pcl::PointCloud<pcl::Normal>::Ptr normalcloud(
        new pcl::PointCloud<pcl::Normal>);
    normalEstimate(filtercloud, normalcloud, 0.05, pc_vector[i]);
    pc_vector[i]->clear();
    // if(!normalcloud->empty())
    //  pcl::io::savePCDFileASCII( fileNameFormat("./normalEstimate", i),
    //  *normalcloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr plannarcloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    plannarEstimate(filtercloud, normalcloud, plannarcloud);
    // if(!plannarcloud->empty())
    //  pcl::io::savePCDFileASCII( fileNameFormat("./plannarEstimate", i),
    //  *plannarcloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr boundcloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    boundaryEstimate(filtercloud, normalcloud, boundcloud);
    // if(!boundcloud->empty())
    //  pcl::io::savePCDFileASCII( fileNameFormat("./boundaryEstimate", i),
    //  *boundcloud);

    filtercloud->clear();
    normalcloud->clear();
    voxelFilter(plannarcloud, plannarcloud, 0.2, 3);
    voxelFilter(boundcloud, boundcloud, 0.2, 3);

    pcl::PointXYZ point;
    for (int j = 0; j < plannarcloud->points.size(); ++j) {
      point.x = plannarcloud->points[j].y;
      point.y = plannarcloud->points[j].z;
      point.z = plannarcloud->points[j].x;
      _featureCloud.pushSurfPoint(point);
    }

    for (int j = 0; j < boundcloud->points.size(); ++j) {
      point.x = boundcloud->points[j].y;
      point.y = boundcloud->points[j].z;
      point.z = boundcloud->points[j].x;
      _featureCloud.pushCornerPoint(point);
    }
    ROS_INFO("Progress: %d/%d", i, pc_size);
  }

  _featureCloud.saveCloudToFiles();

  return 0;
}
