#include <iostream>
#include <memory>

#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Time.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "odom/LaserMapping.h"

namespace lidar_slam {

class LaserMappingNodelet : public nodelet::Nodelet {
public:
  LaserMappingNodelet() {}
  ~LaserMappingNodelet() {}

private:
  virtual void onInit() {
    laser_mapping.reset(new LaserMapping());
    laser_mapping->init(getNodeHandle(), getPrivateNodeHandle());
  }

private:
  boost::shared_ptr<LaserMapping> laser_mapping;
};

} // namespace lidar_slam

PLUGINLIB_EXPORT_CLASS(lidar_slam::LaserMappingNodelet, nodelet::Nodelet)
