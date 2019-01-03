#include "odom/MultiScanRegistration.h"
#include <ros/ros.h>

/** Main node entry point. */
int main(int argc, char **argv) {
  ros::init(argc, argv, "scanRegistration");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  lidar_slam::MultiScanRegistration multiScan;

  if (multiScan.setup(node, privateNode)) {
    ros::spin();
  }

  return 0;
}
