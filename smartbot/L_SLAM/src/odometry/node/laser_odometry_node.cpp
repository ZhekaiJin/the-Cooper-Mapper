#include "odom/LaserOdometry.h"
#include <ros/ros.h>

/** Main node entry point. */
int main(int argc, char **argv) {
  ros::init(argc, argv, "laserOdometry");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  lidar_slam::LaserOdometry laserOdom;

  if (laserOdom.setup(node, privateNode)) {
    ros::spin();
  }

  return 0;
}
