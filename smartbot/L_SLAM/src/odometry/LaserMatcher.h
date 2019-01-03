
#ifndef LIDAR_LASER_MATCHER_H_
#define LIDAR_LASER_MATCHER_H_

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

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <mutex>


#include "common/CircularBuffer.h"
#include "common/FeatureMap.h"
#include "common/DynamicFeatureMap.h"
#include "common/Twist.h"
#include "io/LocalFeatureMap.h"
#include "scan_match/ScanMatch.h"
#include "fusion/imu_queue.h"

namespace lidar_slam {

class LaserMatcher {
public:
  typedef pcl::PointXYZI PointI;
  typedef pcl::PointCloud<PointI> CloudI;
  typedef pcl::PointXYZINormal PointIN;
  typedef pcl::PointCloud<PointIN> CloudIN;

  explicit LaserMatcher();
  ~LaserMatcher();
  virtual bool init(ros::NodeHandle &node, ros::NodeHandle &privateNode) = 0;
  virtual bool setup(ros::NodeHandle &node, ros::NodeHandle &privateNode);
  virtual void process() = 0;

public:
  void laserCloudCornerLastHandler(
      const sensor_msgs::PointCloud2ConstPtr &cornerPointsLastMsg);

  void laserCloudSurfLastHandler(
      const sensor_msgs::PointCloud2ConstPtr &surfacePointsLastMsg);

  void laserCloudFullResHandler(
      const sensor_msgs::PointCloud2ConstPtr &laserCloudFullResMsg);

  void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry);

  bool pubFullMap(std_srvs::Empty::Request &req,
                  std_srvs::Empty::Response &resp);
  bool saveMap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

protected:
  void reset();
  bool hasNewData();
  void prepareFeatureFrame();
  void prepareFeatureSurround();
  void optimizeTransform();
  void transformUpdate();
  void featureMapUpdate();
  void publishResult();
  void transformMerge();
protected:
  IMUQueue imu_que;

  std::string _filesDirectory;
  int _inputFrameSkip;       ///< skip process some frames
  int _surroundMapPubSkip;   ///< skip publish some surround map
  long _inputFrameCount;     ///< counter for all received input frame
  long _surroundMapPubCount; ///< counter for surround map published
  long _scanMatchMapCount;   ///< counter for input frame be scan mathed
  long cloudReceiveCount;

  bool _sendRegisteredCloud;
  bool _sendSurroundCloud;
  bool _useFullCloud;
  bool _dynamicMode; // use dynamic map manager or not

  std::string map_frame;
  
  CloudI::Ptr _laserCloudCornerLast; ///< last corner points cloud
  CloudI::Ptr _laserCloudSurfLast;   ///< last surface points cloud
  CloudI::Ptr _laserCloudFullRes;    ///< last full resolution cloud

  CloudI::Ptr _laserCloudCornerStack;
  CloudI::Ptr _laserCloudSurfStack;
  CloudI::Ptr _laserCloudCornerStackDS; ///< down sampled
  CloudI::Ptr _laserCloudSurfStackDS;   ///< down sampled
  CloudI::Ptr _laserCloudCornerFromMap;
  CloudI::Ptr _laserCloudSurfFromMap;

  ros::Time _timeLaserCloudCornerLast; ///< time of current last corner cloud
  ros::Time _timeLaserCloudSurfLast;   ///< time of current last surface cloud
  ros::Time _timeLaserCloudFullRes; ///< time of current full resolution cloud
  ros::Time _timeLaserOdometry;     ///< time of current laser odometry
  ros::Time _timeLaserOdometryLast;
  ros::Time _timeLaserOdometryMerged;

  bool _newLaserCloudCornerLast; ///< flag if a new last corner cloud has been
                                 /// received
  bool _newLaserCloudSurfLast;   ///< flag if a new last surface cloud has been
                                 /// received
  bool _newLaserCloudFullRes; ///< flag if a new full resolution cloud has been
                              /// received
  bool _newLaserOdometry; ///< flag if a new laser odometry has been received

  std::mutex mtx_transform;

  Twist _laserScanTransform;
  Twist _transformIncre;
  Twist _transformTobeMapped;
  Twist _laserScanTransformLast;
  Twist _transformAftMapped;
  Eigen::Isometry3f _lidarOdomNew;
  Eigen::Isometry3f _lidarOdomLast;
    Eigen::Isometry3f _lidarOdomLastMerged;

  Eigen::Isometry3f _lidarMappedNew;
  Eigen::Isometry3f _lidarMappedLast;
  Eigen::Isometry3f _lidarPoseLast;

  Eigen::Vector3f _velocity;

  std::unique_ptr<FeatureMap<PointI>> _feature_map;
  //FeatureMap<PointI> _feature_map;
  DynamicFeatureMap<PointI> _dynamic_feature_map;
  ScanMatch _scan_match;

  pcl::VoxelGrid<PointI>
      _downSizeFilterCorner; ///< voxel filter for down sizing corner clouds
  pcl::VoxelGrid<PointI>
      _downSizeFilterSurf; ///< voxel filter for down sizing surface clouds

  // ros something
  ros::ServiceServer _mapSaveSrv;
  ros::ServiceServer _fullMapPubSrv;

  nav_msgs::Odometry _odomAftMapped;    ///< mapping odometry message
  tf::StampedTransform _aftMappedTrans; ///< mapping odometry transformation
  tf::StampedTransform _odomTransShift; ///< mapping odometry transformation
  ros::Publisher _pubLidarPoseMerged;

  ros::Publisher _pubLaserCloudSurroundCorner; ///< map cloud message publisher
  ros::Publisher _pubLaserCloudSurroundSurf;   ///< map cloud message publisher
  ros::Publisher _pubFullMap;                  ///< map cloud message publisher

  ros::Publisher _pubLaserCloudFullRes; ///< current full resolution cloud
  ros::Publisher _pubCloudCornerLast2;  ///< map cloud message publisher
  ros::Publisher _pubCloudSurfLast2;    ///< map cloud message publisher
                                        /// message publisher
  ros::Publisher _pubOdomAftMapped;     ///< mapping odometry publisher
  tf::TransformBroadcaster
      _tfBroadcaster; ///< mapping odometry transform broadcaster

  ros::Subscriber
      _subLaserCloudCornerLast; ///< last corner cloud message subscriber
  ros::Subscriber
      _subLaserCloudSurfLast; ///< last surface cloud message subscriber
  ros::Subscriber
      _subLaserCloudFullRes; ///< full resolution cloud message subscriber
  ros::Subscriber _subLaserOdometry; ///< laser odometry message subscriber
};

} // end namespace lidar_slam

#endif // LIDAR_LASER_MATCHER_H_
