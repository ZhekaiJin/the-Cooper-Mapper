#include "odom/TransformMaintenance.h"
#include <ros/ros.h>

/** Main node entry point. */
int main(int argc, char **argv) {
  ros::init(argc, argv, "transformMaintenance");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  lidar_slam::TransformMaintenance transMaintenance;

  if (transMaintenance.setup(node, privateNode)) {
    ros::spin();
  }

  return 0;
}
