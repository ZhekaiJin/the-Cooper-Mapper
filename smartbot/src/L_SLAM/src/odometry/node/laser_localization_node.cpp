#include "odom/LaserLocalization.h"
#include <ros/ros.h>

/** Main node entry point. */
int main(int argc, char **argv) {
  ros::init(argc, argv, "LaserLocalization");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  lidar_slam::LaserLocalization LaserLocalization;

  if (LaserLocalization.init(node, privateNode)) {
    ros::spin();
  }

  return 0;
}
