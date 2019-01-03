#include "ros/ros.h"
#include <hdmap_msgs/imu.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include "common/math_utils.h"
#include "fusion/transPointCLoud.h"
#include "fusion/utmProjection.h"
#include <exception>
#include <iostream>
#include <signal.h>
#include <sstream>
#include <stdlib.h>
#include <vector>
//#include "loadExtrinsic.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace lidar_slam;

class IMUReceiver {
public:
  struct GPIMU {
    int nGPSWeek;
    double dGPSTime;

    double dVX;
    double dVY;
    double dVZ;
    double dAX;
    double dAY;
    double dAZ;

    float fTemperature;
    char cStatus[4];
  };

  IMUReceiver(ros::NodeHandle &node) {

    subSensorImu = node.subscribe<hdmap_msgs::imu>(
        "/sensor/imu", 2, &IMUReceiver::imuHandler, this);
    pubImu = node.advertise<sensor_msgs::Imu>("/imu/data_raw", 1);
  }
  void imuHandler(hdmap_msgs::imu imu) {
    sensor_msgs::Imu imu_raw;
    imu_raw.header = imu.header;
    imu_raw.header.frame_id = "imu";
    imu_raw.angular_velocity.x = deg2rad(imu.dVX);
    imu_raw.angular_velocity.y = deg2rad(imu.dVY);
    imu_raw.angular_velocity.z = deg2rad(imu.dVZ);
    imu_raw.linear_acceleration.x = imu.dAX * 9.81;
    imu_raw.linear_acceleration.y = imu.dAY * 9.81;
    imu_raw.linear_acceleration.z = imu.dAZ * 9.81;
    pubImu.publish(imu_raw);
  }

private:
  ros::Publisher pubImu;
  ros::Subscriber subSensorImu;
  tf::TransformBroadcaster _tfBroadcaster;
  nav_msgs::Odometry Omg;
  tf::StampedTransform Tmg;

  ros::Publisher pubInitOml;
  nav_msgs::Odometry initOml;
  tf::StampedTransform initTml;

  bool initialize;
  Eigen::Vector3d Vml_init;
  Eigen::Quaterniond Qml_init;

  Eigen::Isometry3d Tmw;
  Eigen::Vector3d Vwm_offset;
  Eigen::Isometry3d Tli;
};

int main(int argc, char **argv) {

  ros::init(argc, argv, "imu_receiver");
  ros::NodeHandle n;
  IMUReceiver imu_receiver(n);
  ros::spin();
  return 0;
}