
#include "odom/LaserMatcher.h"
#include "common/feature_utils.h"
#include "common/math_utils.h"
#include "common/nanoflann_pcl.h"
#include "common/ros_utils.h"
#include "common/transform_utils.h"

#include <Eigen/Eigenvalues>
#include <Eigen/QR>

namespace lidar_slam {

using std::sqrt;
using std::fabs;
using std::asin;
using std::atan2;
using std::pow;

LaserMatcher::LaserMatcher()
    : _filesDirectory("~"), _sendRegisteredCloud(true),
      _sendSurroundCloud(true), _useFullCloud(true), _inputFrameSkip(1),
      _surroundMapPubSkip(20), _inputFrameCount(0), _surroundMapPubCount(0),
      _dynamic_feature_map(), _laserCloudCornerLast(new CloudI()), // 221 211 221 || 110 105 110
      _laserCloudSurfLast(new CloudI()), _laserCloudFullRes(new CloudI()),
      _laserCloudCornerStack(new CloudI()), _laserCloudSurfStack(new CloudI()),
      _laserCloudCornerStackDS(new CloudI()),
      _laserCloudSurfStackDS(new CloudI()),
      _laserCloudCornerFromMap(new CloudI()),
      _laserCloudSurfFromMap(new CloudI()) {

  _lidarOdomNew = _lidarOdomLast = _lidarOdomLastMerged = _lidarMappedNew =
      _lidarMappedLast = _lidarPoseLast =Eigen::Isometry3f::Identity();

  cloudReceiveCount = 0;
  map_frame = "map";

}

LaserMatcher::~LaserMatcher() {
  ROS_INFO("[LaserMatcher] _inputFrameCount:%ld", _inputFrameCount);
  ROS_INFO("[LaserMatcher] cloudReceiveCount:%ld", cloudReceiveCount);
}

bool LaserMatcher::setup(ros::NodeHandle &node, ros::NodeHandle &privateNode) {

  imu_que.setup(node, privateNode);

  // fetch laser mapping params
  float fParam;
  int iParam;

  if (privateNode.getParam("filesDirectory", _filesDirectory)) {
    ROS_INFO("Set filesDirectory: %s", _filesDirectory.c_str());
  }

  if ( privateNode.getParam("inputFrameSkip", _inputFrameSkip)) {
    ROS_INFO("Set inputFrameSkip: %d", _inputFrameSkip);
  }

  if (privateNode.getParam("surroundMapPubSkip", _surroundMapPubSkip)) {
    ROS_INFO("Set surroundMapPubSkip: %d", _surroundMapPubSkip);
  }

  if (privateNode.getParam("sendRegisteredCloud", _sendRegisteredCloud)) {
    ROS_INFO("Set sendRegisteredCloud: %d", _sendRegisteredCloud);
  }

  if (privateNode.getParam("sendSurroundCloud", _sendSurroundCloud)) {
    ROS_INFO("Set sendSurroundCloud: %d", _sendSurroundCloud);
  }

  if(privateNode.getParam("dynamicMode", _dynamicMode))
  {
    ROS_INFO("Set dynamicMode: %s", _dynamicMode ? "true" : "false");
  }

  privateNode.getParam("useFullCloud", _useFullCloud);

  float filter_corner = 1.0;
  float filter_surf = 1.0;
  privateNode.getParam("filter_corner", filter_corner);
  privateNode.getParam("filter_surf", filter_surf);
  _downSizeFilterCorner.setLeafSize(filter_corner, filter_corner, filter_corner);
  _downSizeFilterSurf.setLeafSize(filter_surf, filter_surf, filter_surf);

  float map_filter_corner = 1.0;
  float map_filter_surf = 1.0;
  float map_filter = 2.0;
  privateNode.getParam("map_filter_corner", map_filter_corner);
  privateNode.getParam("map_filter_surf", map_filter_surf);
  privateNode.getParam("map_filter", map_filter);

  _scan_match.setConvergeThreshold(0.1, 0.1);
  _scan_match.setUseCore(false);



  // initializa feature cloud
  if(_dynamicMode)
  {
    _dynamic_feature_map.setupLidarFov(16, 7);
    _dynamic_feature_map.setupFilesDirectory(_filesDirectory);
    _dynamic_feature_map.setupFilterSize(map_filter_corner, map_filter_surf, map_filter);
  }
  else{
    int map_cube_x = 121;
    int map_cube_y = 121;
    int map_cube_z = 11;
    privateNode.getParam("map_cube_x", map_cube_x);
    privateNode.getParam("map_cube_y", map_cube_y);
    privateNode.getParam("map_cube_z", map_cube_z);
    _feature_map.reset(new FeatureMap<PointI>(map_cube_x, map_cube_y, map_cube_z));

    _feature_map->setupFilesDirectory(_filesDirectory);
    _feature_map->setupFilterSize(map_filter_corner, map_filter_surf, map_filter);
    _feature_map->loadCloudFromFiles();
  }

  _odomAftMapped.header.frame_id = map_frame;
  _odomAftMapped.child_frame_id = "/aft_mapped";
  _aftMappedTrans.frame_id_ = map_frame;
  _aftMappedTrans.child_frame_id_ = "/aft_mapped";
  _odomTransShift.frame_id_ = map_frame;
  _odomTransShift.child_frame_id_ = "/lidar_odom_init";
  // advertise laser mapping topics
  _pubFullMap = node.advertise<sensor_msgs::PointCloud2>("/FullMap", 1);
  _pubLaserCloudSurroundCorner = node.advertise<sensor_msgs::PointCloud2>(
      "/laser_cloud_surround_corner", 1);

  _pubLaserCloudSurroundSurf =
      node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround_surf", 1);

  _pubCloudCornerLast2 =
      node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last2", 1);

  _pubCloudSurfLast2 =
      node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last2", 1);

  _pubLaserCloudFullRes =
      node.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_4", 1);

  _pubOdomAftMapped =
      node.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 1);

  _pubLidarPoseMerged = node.advertise<nav_msgs::Odometry>("/lidar_to_map2", 5);

  _subLaserCloudCornerLast = node.subscribe<sensor_msgs::PointCloud2>(
      "/laser_cloud_corner_last", 2, &LaserMatcher::laserCloudCornerLastHandler,
      this);

  _subLaserCloudSurfLast = node.subscribe<sensor_msgs::PointCloud2>(
      "/laser_cloud_surf_last", 2, &LaserMatcher::laserCloudSurfLastHandler,
      this);

  _subLaserOdometry = node.subscribe<nav_msgs::Odometry>(
      "/laser_odom_to_init", 5, &LaserMatcher::laserOdometryHandler, this);

  if (_useFullCloud) {
    _subLaserCloudFullRes = node.subscribe<sensor_msgs::PointCloud2>(
        "/velodyne_cloud_3", 2, &LaserMatcher::laserCloudFullResHandler, this);
  }

  _fullMapPubSrv = privateNode.advertiseService(
      "pubFullMap", &LaserMatcher::pubFullMap, this);
  _mapSaveSrv =
      privateNode.advertiseService("saveMap", &LaserMatcher::saveMap, this);

  return true;
}

bool LaserMatcher::pubFullMap(std_srvs::Empty::Request &req,
                              std_srvs::Empty::Response &resp) {
  CloudI::Ptr full_map(new CloudI());
  if(_dynamicMode)
    _dynamic_feature_map.getFullMap(full_map);
  else
    _feature_map->getFullMap(full_map);
  publishCloudMsg(_pubFullMap, *full_map, ros::Time::now(), map_frame);
  return true;
}

bool LaserMatcher::saveMap(std_srvs::Empty::Request &req,
                           std_srvs::Empty::Response &resp) {
  if(!_dynamicMode){
    return _feature_map->saveCloudToFiles();
  }
}

void LaserMatcher::laserCloudCornerLastHandler(
    const sensor_msgs::PointCloud2ConstPtr &cornerPointsLastMsg) {
  _timeLaserCloudCornerLast = cornerPointsLastMsg->header.stamp;

  _laserCloudCornerLast->clear();
  pcl::fromROSMsg(*cornerPointsLastMsg, *_laserCloudCornerLast);

  _newLaserCloudCornerLast = true;
}

void LaserMatcher::laserCloudSurfLastHandler(
    const sensor_msgs::PointCloud2ConstPtr &surfacePointsLastMsg) {
  _timeLaserCloudSurfLast = surfacePointsLastMsg->header.stamp;

  _laserCloudSurfLast->clear();
  pcl::fromROSMsg(*surfacePointsLastMsg, *_laserCloudSurfLast);

  _newLaserCloudSurfLast = true;
}

void LaserMatcher::laserCloudFullResHandler(
    const sensor_msgs::PointCloud2ConstPtr &laserCloudFullResMsg) {
  _timeLaserCloudFullRes = laserCloudFullResMsg->header.stamp;

  _laserCloudFullRes->clear();
  pcl::fromROSMsg(*laserCloudFullResMsg, *_laserCloudFullRes);

  _newLaserCloudFullRes = true;
  cloudReceiveCount++;
}

void LaserMatcher::laserOdometryHandler(
    const nav_msgs::Odometry::ConstPtr &laserOdometry) {
  _timeLaserOdometry = laserOdometry->header.stamp;
  Eigen::Isometry3d is3d;
  Odom2Isometry(laserOdometry, is3d);
  _lidarOdomNew = is3d.cast<float>();
  _newLaserOdometry = true;

  mtx_transform.lock();

  Eigen::Isometry3f lidarPoseMerged;
  transformAssociate(_lidarOdomLast, _lidarOdomNew, _lidarMappedLast,
                     lidarPoseMerged);
  mtx_transform.unlock();

  tf::StampedTransform lidarTFMerged;
  lidarTFMerged.frame_id_ = map_frame;
  lidarTFMerged.child_frame_id_ = "/lidar";
  lidarTFMerged.stamp_ = laserOdometry->header.stamp;
  Isometry2TFtransform(lidarPoseMerged.cast<double>(), lidarTFMerged);
  _tfBroadcaster.sendTransform(lidarTFMerged);

  nav_msgs::Odometry lidarOdomMerged;
  Isometry2Odom(lidarPoseMerged.cast<double>(), lidarOdomMerged);
  lidarOdomMerged.header.stamp = laserOdometry->header.stamp;
  lidarOdomMerged.header.frame_id = map_frame;
  lidarOdomMerged.child_frame_id = "/lidar";
  Eigen::Matrix<float, Eigen::Dynamic, 1> VectorXt = imu_que.getMean();
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> MatrixXt = imu_que.getCov();

  lidarOdomMerged.twist.twist.linear.x = _velocity(0);
  lidarOdomMerged.twist.twist.linear.y = _velocity(1);
  lidarOdomMerged.twist.twist.linear.z = _velocity(2);
  lidarOdomMerged.pose.covariance[0] = MatrixXt(0, 0);
  lidarOdomMerged.pose.covariance[7] = MatrixXt(1, 1);
  lidarOdomMerged.pose.covariance[14] = MatrixXt(2, 2);
  lidarOdomMerged.twist.covariance[0] = MatrixXt(3, 3);
  lidarOdomMerged.twist.covariance[7] = MatrixXt(4, 4);
  lidarOdomMerged.twist.covariance[14] = MatrixXt(5, 5);
  _pubLidarPoseMerged.publish(lidarOdomMerged);
}

void LaserMatcher::reset() {
  _newLaserCloudCornerLast = false;
  _newLaserCloudSurfLast = false;
  _newLaserCloudFullRes = false;
  _newLaserOdometry = false;
}

bool LaserMatcher::hasNewData() {
  if (!_useFullCloud) {
    return _newLaserCloudCornerLast && _newLaserCloudSurfLast &&
           _newLaserOdometry &&
           fabs((_timeLaserCloudCornerLast - _timeLaserOdometry).toSec()) <
               0.005 &&
           fabs((_timeLaserCloudSurfLast - _timeLaserOdometry).toSec()) < 0.005;
  } else {
    return _newLaserCloudCornerLast && _newLaserCloudSurfLast &&
           _newLaserCloudFullRes && _newLaserOdometry &&
           fabs((_timeLaserCloudCornerLast - _timeLaserOdometry).toSec()) <
               0.005 &&
           fabs((_timeLaserCloudSurfLast - _timeLaserOdometry).toSec()) <
               0.005 &&
           fabs((_timeLaserCloudFullRes - _timeLaserOdometry).toSec()) < 0.005;
  }
}

void LaserMatcher::prepareFeatureFrame() {

  _laserCloudCornerStack.swap(_laserCloudCornerLast);
  _laserCloudSurfStack.swap(_laserCloudSurfLast);

  // down sample feature stack clouds
  _laserCloudCornerStackDS->clear();
  _downSizeFilterCorner.setInputCloud(_laserCloudCornerStack);
  _downSizeFilterCorner.filter(*_laserCloudCornerStackDS);

  _laserCloudSurfStackDS->clear();
  _downSizeFilterSurf.setInputCloud(_laserCloudSurfStack);
  _downSizeFilterSurf.filter(*_laserCloudSurfStackDS);
}

void LaserMatcher::prepareFeatureSurround() {

  pcl::PointXYZI currentPos;
  currentPos.getVector3fMap() = _lidarMappedNew.translation();

  Eigen::Vector3d directionZNowD;

  if(_dynamicMode)
    _dynamic_feature_map.update(currentPos, directionZNowD);
  else
    _feature_map->update(currentPos);

  if(_dynamicMode)
    _dynamic_feature_map.getSurroundFeature(*_laserCloudCornerFromMap,
                                    *_laserCloudSurfFromMap);
  else
    _feature_map->getSurroundFeature(*_laserCloudCornerFromMap,
                                  *_laserCloudSurfFromMap);

  if(_laserCloudSurfFromMap->points.size()<10){
    ROS_WARN_STREAM("surround map points too few, currentPos:"<<currentPos);
  }
}

void LaserMatcher::optimizeTransform() {
  _scan_match.scanMatchScan(_laserCloudCornerFromMap, _laserCloudSurfFromMap,
                            _laserCloudCornerStackDS, _laserCloudSurfStackDS,
                            _lidarMappedNew);
}

void LaserMatcher::transformMerge() {
  mtx_transform.lock();
  transformAssociate(_lidarOdomLast, _lidarOdomNew, _lidarMappedLast,
                     _lidarMappedNew);
  _lidarOdomLastMerged = _lidarOdomNew;
  _timeLaserOdometryMerged =_timeLaserOdometry;
  mtx_transform.unlock();
}

void LaserMatcher::transformUpdate() {
  mtx_transform.lock();
  _lidarMappedLast = _lidarMappedNew;
  _lidarOdomLast = _lidarOdomLastMerged;
  mtx_transform.unlock();
}

void LaserMatcher::featureMapUpdate() {
    if(!_dynamicMode){
      _feature_map->addFeatureCloud(*_laserCloudCornerStackDS,
                                  *_laserCloudSurfStackDS, _lidarMappedNew);

    }
}

void LaserMatcher::publishResult() {

  //pub aft_map
  _odomAftMapped.header.stamp = _timeLaserOdometryMerged;
  Isometry2Odom(_lidarMappedNew.cast<double>(), _odomAftMapped);
  _pubOdomAftMapped.publish(_odomAftMapped);

  _aftMappedTrans.stamp_ = _timeLaserOdometryMerged;
  Isometry2TFtransform(_lidarMappedNew.cast<double>(), _aftMappedTrans);
  _tfBroadcaster.sendTransform(_aftMappedTrans);

  //pub odom shift
  Eigen::Isometry3f odomShift;
  odomShift = _lidarMappedNew * _lidarOdomLast.inverse();
  _odomTransShift.stamp_ = _timeLaserOdometryMerged;
  Isometry2TFtransform(odomShift.cast<double>(), _odomTransShift);
  _tfBroadcaster.sendTransform(_odomTransShift);

  if (_sendSurroundCloud) {
    _surroundMapPubCount++;
    if (_surroundMapPubCount % (_surroundMapPubSkip + 1) == 1) {
      publishCloudMsg(_pubLaserCloudSurroundCorner, *_laserCloudCornerFromMap,
                      _timeLaserOdometryMerged, map_frame);
      publishCloudMsg(_pubLaserCloudSurroundSurf, *_laserCloudSurfFromMap,
                      _timeLaserOdometryMerged, map_frame);
    }
  }

  if (_sendRegisteredCloud) {

    publishCloudMsg(_pubCloudCornerLast2, *_laserCloudCornerStack,
                    _timeLaserOdometryMerged, "/aft_mapped");
    publishCloudMsg(_pubCloudSurfLast2, *_laserCloudSurfStack,
                    _timeLaserOdometryMerged, "/aft_mapped");
    publishCloudMsg(_pubLaserCloudFullRes, *_laserCloudFullRes,
                    _timeLaserOdometryMerged, "/aft_mapped");
  }
}

} // end namespace lidar_slam
