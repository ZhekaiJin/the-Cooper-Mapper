
#include "LaserLocalization.h"
#include "common/feature_utils.h"
#include "common/math_utils.h"
#include "common/nanoflann_pcl.h"
#include "common/ros_utils.h"
#include "common/transform_utils.h"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/QR>

namespace lidar_slam {

LaserLocalization::LaserLocalization() {}

LaserLocalization::~LaserLocalization() {}

bool LaserLocalization::init(ros::NodeHandle &node,
                             ros::NodeHandle &privateNode) {

  map_frame = "map";
  _isLidarPoseReset = false;
  _lidarPoseReset = Eigen::Isometry3f::Identity();
  if (setup(node, privateNode)) {
    _subInitialPose = node.subscribe(
        "initialpose2", 2, &LaserLocalization::initialPoseHandler2, this);
    _subInitialPose2 = node.subscribe(
        "initialpose", 2, &LaserLocalization::initialPoseHandler, this);

    spin_thread = std::thread(&LaserLocalization::spin, this);

    return true;
  }
  return false;
}

void LaserLocalization::initialPoseHandler2(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {
  handleInitialPoseMessage(*msg);
}

void LaserLocalization::initialPoseHandler(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {
  std::string global_frame_id_ = "/map";
  if (msg->header.frame_id == "") {
    ROS_WARN("Received initial pose with empty frame_id.  You should always "
             "supply a frame_id.");
    return;
  }

  Eigen::Quaternionf Qml;
  Qml.x() = msg->pose.pose.orientation.x;
  Qml.y() = msg->pose.pose.orientation.y;
  Qml.z() = msg->pose.pose.orientation.z * -1.0;
  Qml.w() = msg->pose.pose.orientation.w * -1.0;

  Eigen::Vector3f Vml;
  Vml(0) = msg->pose.pose.position.x;
  Vml(1) = msg->pose.pose.position.y;
  Vml(2) = msg->pose.pose.position.z;

  Eigen::Isometry3f Tml = Eigen::Isometry3f::Identity();
  Tml.rotate(Qml);
  Tml.pretranslate(Vml);

  mtx_transform.lock();
  Eigen::Quaternionf quat(Tml.rotation());
  imu_que.reset(Vml, quat);
  ROS_INFO_STREAM("init:\n" << Tml.matrix());
  _isLidarPoseReset = true;
  _lidarPoseReset = Tml;
  mtx_transform.unlock();
  _initialized = true;
}

void LaserLocalization::handleInitialPoseMessage(
    const geometry_msgs::PoseWithCovarianceStamped &msg) {
  std::string global_frame_id_ = "/map";
  if (msg.header.frame_id == "") {
    ROS_WARN("Received initial pose with empty frame_id.  You should always "
             "supply a frame_id.");
    return;
  }

  Eigen::Quaternionf Qml;
  Qml.x() = msg.pose.pose.orientation.x;
  Qml.y() = msg.pose.pose.orientation.y;
  Qml.z() = msg.pose.pose.orientation.z;
  Qml.w() = msg.pose.pose.orientation.w;

  Eigen::Vector3f Vml;
  Vml(0) = msg.pose.pose.position.x;
  Vml(1) = msg.pose.pose.position.y;
  Vml(2) = msg.pose.pose.position.z;

  Eigen::Isometry3f Tml = Eigen::Isometry3f::Identity();
  Tml.rotate(Qml);
  Tml.pretranslate(Vml);

  mtx_transform.lock();
  Eigen::Quaternionf quat(Tml.rotation());
  imu_que.reset(Vml, quat);
  ROS_INFO_STREAM("init:\n" << Tml.matrix());
  _isLidarPoseReset = true;
  _lidarPoseReset = Tml;
  mtx_transform.unlock();
  _initialized = true;
}

void LaserLocalization::spin() {
  ros::Rate rate(500);
  bool status = ros::ok();

  while (status) {
    ros::spinOnce();
    process();
    status = ros::ok();
    rate.sleep();
  }
}

void LaserLocalization::optimizeTransform() {

  if(_dynamicMode)
    _dynamic_feature_map.scanMatchScan(_laserCloudCornerStackDS, _laserCloudSurfStackDS,
                            _lidarMappedNew);
  else
    _feature_map->scanMatchScan(_laserCloudCornerStackDS, _laserCloudSurfStackDS,
                            _lidarMappedNew);
 /*
  _scan_match.scanMatchScan(_laserCloudCornerFromMap, _laserCloudSurfFromMap,
                            _laserCloudCornerStackDS, _laserCloudSurfStackDS,
                            _lidarMappedNew);

*/ 
}

void LaserLocalization::transformUpdate() {
  mtx_transform.lock();
  if (_isLidarPoseReset) {
    _lidarMappedNew = _lidarPoseReset;
    _isLidarPoseReset = false;
  }
  Eigen::Vector3f pos_last = _lidarMappedLast.translation();
  _lidarMappedLast = _lidarMappedNew;
   _lidarOdomLast = _lidarOdomLastMerged;

  mtx_transform.unlock();


  //imu fusion
  Eigen::Isometry3f lidarPose = _lidarMappedNew;
  Eigen::Isometry3f pre_pos, cor_pos;
  imu_que.predict(_timeLaserOdometryMerged, pre_pos);
  if(!_timeLaserOdometryLast.is_zero()){
    _velocity = (lidarPose.translation() - _lidarPoseLast.translation())/(_timeLaserOdometryMerged-_timeLaserOdometryLast).toSec();
    if(_velocity.norm()>30)
      _velocity = Eigen::Vector3f::Zero();
      Eigen::Vector3f velocity = _velocity;
    imu_que.correct(lidarPose, cor_pos, velocity);    
  }
  _lidarPoseLast = lidarPose;
  _timeLaserOdometryLast = _timeLaserOdometryMerged;
}

void LaserLocalization::process() {
  if (!hasNewData() || !_initialized) {
    return;
  }

  reset();

  _inputFrameCount++;
  if (_inputFrameSkip > 0) {
    if (_inputFrameCount % (_inputFrameSkip + 1) != 1)
      return;
  }

  transformMerge();
  prepareFeatureFrame();
  prepareFeatureSurround();
  optimizeTransform();
  transformUpdate();
  // featureMapUpdate();
  publishResult();
}

} // end namespace lidar_slam
