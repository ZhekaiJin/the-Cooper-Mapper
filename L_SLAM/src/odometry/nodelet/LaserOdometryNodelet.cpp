#include <iostream>
#include <memory>

#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Time.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "odom/LaserOdometry.h"

namespace lidar_slam {

class LaserOdometryNodelet : public nodelet::Nodelet {
public:
  LaserOdometryNodelet() {}
  ~LaserOdometryNodelet() {}

private:
  virtual void onInit() {
    laser_odometry.reset(new LaserOdometry());
    laser_odometry->setup(getNodeHandle(), getPrivateNodeHandle());
  }

private:
  boost::shared_ptr<LaserOdometry> laser_odometry;
};

} // namespace lidar_slam

PLUGINLIB_EXPORT_CLASS(lidar_slam::LaserOdometryNodelet, nodelet::Nodelet)
