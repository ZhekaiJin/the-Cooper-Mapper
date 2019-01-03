#include "odom/LaserMappingLocal.h"
#include <ros/ros.h>

/** Main node entry point. */
int main(int argc, char **argv) {
  ros::init(argc, argv, "laserMappingLocal");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  lidar_slam::LaserMappingLocal laserMappingLocal;

  if (laserMappingLocal.init(node, privateNode)) {
    ros::spin();
  }

  return 0;
}
