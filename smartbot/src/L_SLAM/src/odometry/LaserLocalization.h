
#ifndef LIDAR_LASERLOCALIZATION_H
#define LIDAR_LASERLOCALIZATION_H

#include "odom/LaserMatcher.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

namespace lidar_slam {

class LaserLocalization : public LaserMatcher {
public:
  explicit LaserLocalization();
  ~LaserLocalization();

  virtual bool init(ros::NodeHandle &node, ros::NodeHandle &privateNode);
  virtual void process();
  void spin();

protected:
  void initialPoseHandler2(
      const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
  void initialPoseHandler(
      const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
  void
  handleInitialPoseMessage(const geometry_msgs::PoseWithCovarianceStamped &msg);

  void optimizeTransform();
  void transformUpdate();
private:
  std::thread spin_thread;

  bool _isLidarPoseReset;
  Eigen::Isometry3f _lidarPoseReset;
  // ros something

  ros::Subscriber _subInitialPose;
  ros::Subscriber _subInitialPose2;

  tf::TransformListener tf_;

  bool _initialized;
};

} // end namespace lidar_slam

#endif // LIDAR_LASERLOCALIZATION_H
