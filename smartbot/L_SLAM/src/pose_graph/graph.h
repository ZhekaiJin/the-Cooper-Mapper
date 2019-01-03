
#include <nav_msgs/Odometry.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Dense>
#include <atomic>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/thread.hpp>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>

#include "common/FeatureMap.h"
#include "common/ros_utils.h"
#include "common/feature_utils.h"
#include "common/math_utils.h"
#include "common/nanoflann_pcl.h"
#include "common/transform_utils.h"

#include "pose_graph/keyframe.h"
#include "pose_graph/keyframe_updater.hpp"
#include "pose_graph/loop_detector.hpp"
#include "pose_graph/solver_g2o.h"

namespace pose_graph {

class Graph {
public:
  typedef pcl::PointXYZI PointI;
  typedef pcl::PointCloud<PointI> CloudI;
  typedef pcl::PointXYZINormal PointIN;
  typedef pcl::PointCloud<PointIN> CloudIN;

  explicit Graph();
  ~Graph();

  bool setup(ros::NodeHandle &node, ros::NodeHandle &privateNode);

  void laserCloudCornerLastHandler(
      const sensor_msgs::PointCloud2ConstPtr &cornerPointsLastMsg);
  void laserCloudSurfLastHandler(
      const sensor_msgs::PointCloud2ConstPtr &surfacePointsLastMsg);
  void laserCloudFullResHandler(
      const sensor_msgs::PointCloud2ConstPtr &laserCloudFullResMsg);
  void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry);

  void generateGraphTrajectoryCloud(const std::vector<KeyFrame::Ptr> &keyframes,
                                    CloudIN &cloud) {
    cloud.clear();
    for (int i = 0; i < keyframes.size(); i++) {
      Eigen::Isometry3f estimate = keyframes[i]->node->estimate().cast<float>();
      Eigen::Quaternionf quat(estimate.rotation());
      PointIN pos;
      pos.getVector3fMap() = estimate.translation();
      pos.data_n[0] = quat.x();
      pos.data_n[1] = quat.y();
      pos.data_n[2] = quat.z();
      // pos.data_n[3] = quat.w();
      pos.intensity = quat.w();
      pos.curvature = i;
      cloud.push_back(pos);
    }
  }
  void generateOdomTrajectoryCloud(const std::vector<KeyFrame::Ptr> &keyframes,
                                   CloudIN &cloud) {
    cloud.clear();
    for (int i = 0; i < keyframes.size(); i++) {
      Eigen::Isometry3f estimate = keyframes[i]->odom.cast<float>();
      Eigen::Quaternionf quat(estimate.rotation());
      PointIN pos;
      pos.getVector3fMap() = estimate.translation();
      pos.data_n[0] = quat.x();
      pos.data_n[1] = quat.y();
      pos.data_n[2] = quat.z();
      // pos.data_n[3] = quat.w();
      pos.intensity = quat.w();
      pos.curvature = i;
      cloud.push_back(pos);
    }
  }

  template <typename PointT>
  void saveTrajectoryCloud(pcl::PointCloud<PointT> &cloud,
                           const std::string &directory,
                           const std::string &name) {
    if (!boost::filesystem::is_directory(directory)) {
      boost::filesystem::create_directory(directory);
    }
    std::string path = directory + "/" + name + ".pcd";
    pcl::io::savePCDFileASCII(path, cloud);
  }

void getFinalFeatureMap();

  bool hasNewData();

  void reset();

  void optimize();

  void spin();

  bool save(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

  void add_frame(KeyFrame::Ptr &keyframe);

  bool flush_keyframe_queue();

  void process();

private:
  std::thread optimize_thread;
  Eigen::Isometry3d tf_odomd;
  std::mutex tf_odom2graph_mutex;
  Eigen::Isometry3d tf_odom2graph;
  std::mutex keyframe_queue_mutex;
  std::deque<KeyFrame::Ptr> keyframe_queue;

  int max_keyframes_per_update;
  std::deque<KeyFrame::Ptr> new_keyframes;

  std::vector<KeyFrame::Ptr> keyframes;
  std::unique_ptr<SolverG2O> solver_g2o;
  std::unique_ptr<LoopDetector> loop_detector;
  std::unique_ptr<KeyframeUpdater> keyframe_updater;

  std::string _filesDirectory;
  // ros sometrhing
  ros::Subscriber
      _subLaserCloudCornerLast2;
  ros::Subscriber
      _subLaserCloudSurfLast2;
  ros::Subscriber
      _subLaserCloudFullRes2;
  ros::Subscriber _subLaserOdometry2;
  ros::Publisher _pubOdomAftGraph;
  nav_msgs::Odometry _odomAftGraph;

  ros::ServiceServer _saveSrv;

  CloudI::Ptr _laserCloudCornerLast2;
  CloudI::Ptr _laserCloudSurfLast2;
  CloudI::Ptr _laserCloudFullRes2;
  CloudI::Ptr _laserCloudFullResStack;

  ros::Time _timeLaserCloudCornerLast2;
  ros::Time _timeLaserCloudSurfLast2;
  ros::Time _timeLaserCloudFullRes2;
  ros::Time _timeLaserOdometry2;

  bool _newLaserCloudCornerLast2;
  bool _newLaserCloudSurfLast2; 
  bool _newLaserCloudFullRes2; 
  bool _newLaserOdometry2;
};
}
