#ifndef LIDAR_IO_UTILS_H
#define LIDAR_IO_UTILS_H

#include <boost/filesystem.hpp>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

namespace lidar_slam {

inline std::string fileNameFormat(const std::string directory, int number,
                                  const std::string suffix) {
  if (!boost::filesystem::is_directory(directory)) {
    boost::filesystem::create_directory(directory);
  }
  std::string str = directory + '/' + std::to_string(number) + suffix;
  return str;
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

} // end namespace lidar_slam

#endif // LIDAR_IO_UTILS_H
