#include <iostream>
#include <memory>

#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Time.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "pose_graph/graph.h"

namespace pose_graph {

class GraphNodelet : public nodelet::Nodelet {
public:
  GraphNodelet() {}
  ~GraphNodelet() {}

private:
  virtual void onInit() {
    graph.reset(new Graph());
    graph->setup(getNodeHandle(), getPrivateNodeHandle());
  }

private:
  boost::shared_ptr<Graph> graph;
};

} // namespace pose_graph

PLUGINLIB_EXPORT_CLASS(pose_graph::GraphNodelet, nodelet::Nodelet)
