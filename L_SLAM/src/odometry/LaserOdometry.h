
#ifndef LIDAR_LASERODOMETRY_H
#define LIDAR_LASERODOMETRY_H

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <mutex>
#include <thread>

#include "common/Twist.h"
#include "common/nanoflann_pcl.h"
#include "fusion/imu_queue.h"
namespace lidar_slam {

class LaserOdometry {
public:
  typedef pcl::PointXYZI PointI;
  typedef pcl::PointCloud<PointI> CloudI;
  typedef pcl::PointXYZINormal PointIN;
  typedef pcl::PointCloud<PointIN> CloudIN;

  explicit LaserOdometry();
  ~LaserOdometry();

  virtual bool setup(ros::NodeHandle &node, ros::NodeHandle &privateNode);

  void laserCloudSharpHandler(
      const sensor_msgs::PointCloud2ConstPtr &cornerPointsSharpMsg);

  void laserCloudLessSharpHandler(
      const sensor_msgs::PointCloud2ConstPtr &cornerPointsLessSharpMsg);

  void laserCloudFlatHandler(
      const sensor_msgs::PointCloud2ConstPtr &surfPointsFlatMsg);

  void laserCloudLessFlatHandler(
      const sensor_msgs::PointCloud2ConstPtr &surfPointsLessFlatMsg);

  void laserCloudFullResHandler(
      const sensor_msgs::PointCloud2ConstPtr &laserCloudFullResMsg);

  void spin();

  void process();

  void scanMatch();

protected:
  void reset();

  bool hasNewData();

  void transformToStart(const PointI &pi, PointI &po);

  size_t transformToEnd(CloudI::Ptr &cloud);

  void transformToStart(const PointIN &pi, PointIN &po);

  size_t transformToEnd(CloudIN::Ptr &cloud);

  void transformUpdate();

  void publishResult();


private:
  std::thread spin_thread;
  //IMUQueue imu_que;

  bool _systemInited;    ///< initialization flag
  long _inputFrameCount; ///< counter for input frames
  long cloudReceiveCount;

  size_t _maxIterations; ///< maximum number of iterations
  float _deltaTAbort;    ///< optimization abort threshold for deltaT
  float _deltaRAbort;    ///< optimization abort threshold for deltaR
  float _scale_rot_y;
  float _scale_trans_z; ///< optimization abort threshold for deltaT
  bool _sendRegisteredCloud;
  bool _receiveFullCloud;
  CloudI::Ptr _cornerPointsSharp;     ///< sharp corner points cloud
  CloudI::Ptr _cornerPointsLessSharp; ///< less sharp corner points cloud
  CloudI::Ptr _surfPointsFlat;        ///< flat surface points cloud
  CloudI::Ptr _surfPointsLessFlat;    ///< less flat surface points cloud
  CloudIN::Ptr _laserCloud;           ///< full resolution cloud

  CloudI::Ptr _lastCornerCloud;  ///< last corner points cloud
  CloudI::Ptr _lastSurfaceCloud; ///< last surface points cloud

  CloudI::Ptr _laserCloudOri; ///< point selection
  CloudI::Ptr _coeffSel;      ///< point selection coefficients

  nanoflann::KdTreeFLANN<PointI>
      _lastCornerKDTree; ///< last corner cloud KD-tree
  nanoflann::KdTreeFLANN<PointI>
      _lastSurfaceKDTree; ///< last surface cloud KD-tree

  ros::Time _timeCornerPointsSharp;     ///< time of current sharp corner
  ros::Time _timeCornerPointsLessSharp; ///< time of current less sharp corner
  ros::Time _timeSurfPointsFlat;        ///< time of current flat surface
  ros::Time _timeSurfPointsLessFlat;    ///< time of current less flat surface
  ros::Time _timeLaserCloudFullRes;     ///< time of current full resolution
  ros::Time _timeImuTrans; ///< time of current IMU transformation information

  bool _newCornerPointsSharp;     ///< flag if a new sharp corner cloud has been
                                  /// received
  bool _newCornerPointsLessSharp; ///< flag if a new less sharp corner cloud has
                                  /// been received
  bool _newSurfPointsFlat;        ///< flag if a new flat surface cloud has been
                                  /// received
  bool _newSurfPointsLessFlat;    ///< flag if a new less flat surface cloud has
                                  /// been received
  bool _newLaserCloudFullRes; ///< flag if a new full resolution cloud has been
                              /// received
  bool _newImuTrans; ///< flag if a new IMU transformation information cloud has
                     /// been received

  std::vector<int>
      _pointSearchCornerInd1; ///< first corner point search index buffer
  std::vector<int>
      _pointSearchCornerInd2; ///< second corner point search index buffer

  std::vector<int>
      _pointSearchSurfInd1; ///< first surface point search index buffer
  std::vector<int>
      _pointSearchSurfInd2; ///< second surface point search index buffer
  std::vector<int>
      _pointSearchSurfInd3; ///< third surface point search index buffer

  Twist _transform;    ///< optimized pose transformation
  Eigen::Isometry3f _Tsum; ///< accumulated optimized pose transformation

  nav_msgs::Odometry _laserOdometryMsg;     ///< laser odometry message
  tf::StampedTransform _laserOdometryTrans; ///< laser odometry transformation

  ros::Publisher
      _pubLaserCloudCornerLast; ///< last corner cloud message publisher
  ros::Publisher
      _pubLaserCloudSurfLast; ///< last surface cloud message publisher
  ros::Publisher
      _pubLaserCloudFullRes;        ///< full resolution cloud message publisher
  ros::Publisher _pubLaserOdometry; ///< laser odometry publisher
  tf::TransformBroadcaster
      _tfBroadcaster; ///< laser odometry transform broadcaster

  ros::Subscriber
      _subCornerPointsSharp; ///< sharp corner cloud message subscriber
  ros::Subscriber
      _subCornerPointsLessSharp; ///< less sharp corner cloud message subscriber
  ros::Subscriber _subSurfPointsFlat; ///< flat surface cloud message subscriber
  ros::Subscriber
      _subSurfPointsLessFlat; ///< less flat surface cloud message subscriber
  ros::Subscriber
      _subLaserCloudFullRes; ///< full resolution cloud message subscriber
};

} // end namespace lidar_slam

#endif // LIDAR_LASERODOMETRY_H
