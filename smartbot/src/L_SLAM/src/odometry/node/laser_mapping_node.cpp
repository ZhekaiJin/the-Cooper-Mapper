#include "odom/LaserMapping.h"
#include <ros/ros.h>

/** Main node entry point. */
int main(int argc, char **argv) {
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  lidar_slam::LaserMapping laserMapping;

  if (laserMapping.init(node, privateNode)) {
    ros::spin();
  }

  return 0;
}
