#include <iostream>
#include <memory>

#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Time.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "odom/TransformMaintenance.h"

namespace lidar_slam {

class TransformMaintenanceNodelet : public nodelet::Nodelet {
public:
  TransformMaintenanceNodelet() {}
  ~TransformMaintenanceNodelet() {}

private:
  virtual void onInit() {
    transform_maintenance.reset(new TransformMaintenance());
    transform_maintenance->setup(getNodeHandle(), getPrivateNodeHandle());
  }

private:
  boost::shared_ptr<TransformMaintenance> transform_maintenance;
};

} // namespace lidar_slam

PLUGINLIB_EXPORT_CLASS(lidar_slam::TransformMaintenanceNodelet,
                       nodelet::Nodelet)
