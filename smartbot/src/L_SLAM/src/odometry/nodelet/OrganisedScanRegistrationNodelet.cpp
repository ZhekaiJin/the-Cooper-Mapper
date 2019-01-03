#include <iostream>
#include <memory>

#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Time.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "odom/OrganisedScanRegistration.h"

namespace lidar_slam {

class OrganisedScanRegistrationNodelet : public nodelet::Nodelet {
public:
  OrganisedScanRegistrationNodelet() {}
  ~OrganisedScanRegistrationNodelet() {}

private:
  virtual void onInit() {
    organ_scan_registration.reset(new OrganisedScanRegistration());
    organ_scan_registration->setup(getNodeHandle(), getPrivateNodeHandle());
  }

private:
  boost::shared_ptr<OrganisedScanRegistration> organ_scan_registration;
};

} // namespace lidar_slam

PLUGINLIB_EXPORT_CLASS(lidar_slam::OrganisedScanRegistrationNodelet,
                       nodelet::Nodelet)
