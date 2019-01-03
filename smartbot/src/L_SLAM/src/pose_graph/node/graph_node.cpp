#include "pose_graph/graph.h"
#include <ros/ros.h>

/** Main node entry point. */
int main(int argc, char **argv) {
  ros::init(argc, argv, "graph");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  pose_graph::Graph graph;

  if (graph.setup(node, privateNode)) {
    graph.spin();
  }

  return 0;
}
