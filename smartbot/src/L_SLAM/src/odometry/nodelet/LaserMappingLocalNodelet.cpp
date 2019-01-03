#include <iostream>
#include <memory>

#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Time.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "odom/LaserMappingLocal.h"

namespace lidar_slam {

class LaserMappingLocalNodelet : public nodelet::Nodelet {
public:
  LaserMappingLocalNodelet() {}
  ~LaserMappingLocalNodelet() {}

private:
  virtual void onInit() {
    laser_mapping_local.reset(new LaserMappingLocal());
    laser_mapping_local->init(getNodeHandle(), getPrivateNodeHandle());
  }

private:
  boost::shared_ptr<LaserMappingLocal> laser_mapping_local;
};

} // namespace lidar_slam

PLUGINLIB_EXPORT_CLASS(lidar_slam::LaserMappingLocalNodelet, nodelet::Nodelet)
