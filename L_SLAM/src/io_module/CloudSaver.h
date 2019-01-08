
#ifndef LIDAR_CLOUD_RECEIVER_H
#define LIDAR_CLOUD_RECEIVER_H

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ctime>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

namespace lidar_slam {

class CloudSaver {
public:
  explicit CloudSaver();
  ~CloudSaver();

  void setBinary(bool use_binary_) { _use_binary = use_binary }
  void setPath(const std::string &path_) { _path = path_; }

  template <typename PointT>
  bool save(pcl::PointCloud<PointT> cloud, int cloud_id) {

    if (cloud.empty()) {
      ROS_ERROR("Input cloud is empty.");
      return false;
    }
    std::string file_name(_path);
    file_name += "/" + std::to_string(cloud_id) + std::string(".pcd");
    if (!pcl::io::savePLYFile(path, cloud, _use_binary)) {
      ROS_ERROR("Cloud save failed.");
      return false;
    }
  }

private:
  std::string timeToStr() {
    std::string stime;
    std::stringstream strtime;
    std::time_t currenttime = std::time(0);
    char tAll[255];
    std::strftime(tAll, sizeof(tAll), "%Y-%m-%d-%H-%M-%S",
                  std::localtime(&currenttime));
    strtime << tAll;
    stime = strtime.str();
    return stime;
  }

private:
  std::string _path;
  bool _use_binary;
  int _frame_count;
};
}

#endif