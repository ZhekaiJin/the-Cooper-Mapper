#include <iostream>
#include <memory>

#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Time.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "odom/MultiScanRegistration.h"

namespace lidar_slam {

class MultiScanRegistrationNodelet : public nodelet::Nodelet {
public:
  MultiScanRegistrationNodelet() {}
  ~MultiScanRegistrationNodelet() {}

private:
  virtual void onInit() {
    multi_scan_registration.reset(new MultiScanRegistration());
    multi_scan_registration->setup(getNodeHandle(), getPrivateNodeHandle());
  }

private:
  boost::shared_ptr<MultiScanRegistration> multi_scan_registration;
};

} // namespace lidar_slam

PLUGINLIB_EXPORT_CLASS(lidar_slam::MultiScanRegistrationNodelet,
                       nodelet::Nodelet)
