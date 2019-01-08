
#ifndef LIDAR_LASERMAPPING_H
#define LIDAR_LASERMAPPING_H

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include "odom/LaserMatcher.h"

namespace lidar_slam {

class LaserMapping : public LaserMatcher {
public:
  explicit LaserMapping();
  ~LaserMapping();

  virtual bool init(ros::NodeHandle &node, ros::NodeHandle &privateNode);

  void spin();

  virtual void process();

private:
  std::thread spin_thread;
};

} // end namespace lidar_slam

#endif // LIDAR_LASERMAPPING_H
