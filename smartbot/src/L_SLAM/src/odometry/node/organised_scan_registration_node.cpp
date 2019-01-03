#include "odom/OrganisedScanRegistration.h"
#include <ros/ros.h>

/** Main node entry point. */
int main(int argc, char **argv) {
  ros::init(argc, argv, "OrganisedScanRegistration");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  lidar_slam::OrganisedScanRegistration organisedScan;

  if (organisedScan.setup(node, privateNode)) {
    ros::spin();
  }

  return 0;
}
