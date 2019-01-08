#include "evaluation/Evaluation.h"
#include <ros/ros.h>

/** Main node entry point. */
int main(int argc, char **argv) {
  ros::init(argc, argv, "Evaluation");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  lidar_slam::Evaluation Evaluation;

  if (Evaluation.init(node, privateNode)) {
    ros::spin();
  }

  return 0;
}
