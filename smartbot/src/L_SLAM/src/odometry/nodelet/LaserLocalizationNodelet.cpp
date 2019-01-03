#include <iostream>
#include <memory>

#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Time.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "odom/LaserLocalization.h"

namespace lidar_slam {

class LaserLocalizationNodelet : public nodelet::Nodelet {
public:
  LaserLocalizationNodelet() {}
  ~LaserLocalizationNodelet() {}

private:
  virtual void onInit() {
    laser_localization.reset(new LaserLocalization());
    laser_localization->init(getNodeHandle(), getPrivateNodeHandle());
  }

private:
  boost::shared_ptr<LaserLocalization> laser_localization;
};

} // namespace lidar_slam

PLUGINLIB_EXPORT_CLASS(lidar_slam::LaserLocalizationNodelet, nodelet::Nodelet)
