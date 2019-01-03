
#ifndef LIDAR_MAPPING_LOCAL_H
#define LIDAR_MAPPING_LOCAL_H

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include "io/LocalFeatureMap.h"
#include "odom/LaserMatcher.h"

namespace lidar_slam {

class LaserMappingLocal : public LaserMatcher {
public:
  explicit LaserMappingLocal();
  ~LaserMappingLocal();

  virtual bool init(ros::NodeHandle &node, ros::NodeHandle &privateNode);
  virtual void process();
  void spin();

  void prepareFeatureSurround();
  void featureMapUpdate();

private:
  LocalFeatureMap<PointI> _local_feature_map;
  std::thread spin_thread;
};

} // end namespace lidar_slam

#endif // LIDAR_MAPPING_LOCAL_H
