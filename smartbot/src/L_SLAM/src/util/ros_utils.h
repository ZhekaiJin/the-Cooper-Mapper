
#ifndef LIDAR_COMMON_H
#define LIDAR_COMMON_H

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Dense>

namespace lidar_slam {

/** \brief Construct a new point cloud message from the specified information
 * and publish it via the given publisher.
 *
 * @tparam PointT the point type
 * @param publisher the publisher instance
 * @param cloud the cloud to publish
 * @param stamp the time stamp of the cloud message
 * @param frameID the message frame ID
 */
template <typename PointT>
inline void publishCloudMsg(ros::Publisher &publisher,
                            const pcl::PointCloud<PointT> &cloud,
                            const ros::Time &stamp, std::string frameID) {
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.stamp = stamp;
  msg.header.frame_id = frameID;
  publisher.publish(msg);
}

inline void Isometry2TFtransform(const Eigen::Isometry3d& is3d,
                               tf::StampedTransform &tf_trans) {
  Eigen::Quaterniond quat(is3d.rotation());
  Eigen::Vector3d pos(is3d.translation());
  quat.normalize();

  tf_trans.setRotation(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()));
  tf_trans.setOrigin(tf::Vector3(pos(0), pos(1), pos(2)));
}

inline void Isometry2GEOtransform(const Eigen::Isometry3d& is3d,
                               geometry_msgs::TransformStamped &odom_trans) {
  Eigen::Quaterniond quat(is3d.rotation());
  Eigen::Vector3d pos(is3d.translation());

  quat.normalize();
  geometry_msgs::Quaternion odom_quat;
  odom_quat.w = quat.w();
  odom_quat.x = quat.x();
  odom_quat.y = quat.y();
  odom_quat.z = quat.z();
  odom_trans.transform.rotation = odom_quat;

  odom_trans.transform.translation.x = pos(0);
  odom_trans.transform.translation.y = pos(1);
  odom_trans.transform.translation.z = pos(2);
}

inline void Odom2Isometry(const nav_msgs::OdometryConstPtr &odom_msg,
                          Eigen::Isometry3d &is3d) {
  const auto &orientation = odom_msg->pose.pose.orientation;
  const auto &position = odom_msg->pose.pose.position;

  Eigen::Quaterniond quat;
  quat.w() = orientation.w;
  quat.x() = orientation.x;
  quat.y() = orientation.y;
  quat.z() = orientation.z;
  quat.normalize();

  is3d = Eigen::Isometry3d::Identity();
  is3d.rotate(quat);
  is3d.pretranslate( Eigen::Vector3d(position.x, position.y, position.z));
}

inline void Odom2Isometry(const nav_msgs::Odometry &odom,
                          Eigen::Isometry3d &is3d) {
  const auto &orientation = odom.pose.pose.orientation;
  const auto &position = odom.pose.pose.position;

  Eigen::Quaterniond quat;
  quat.w() = orientation.w;
  quat.x() = orientation.x;
  quat.y() = orientation.y;
  quat.z() = orientation.z;
  quat.normalize();

  is3d = Eigen::Isometry3d::Identity();
  is3d.rotate(quat);
  is3d.pretranslate( Eigen::Vector3d(position.x, position.y, position.z));
}

inline void Isometry2Odom(const Eigen::Isometry3d& is3d,
                          nav_msgs::Odometry &odom_msg) {
  auto &orientation = odom_msg.pose.pose.orientation;
  auto &position = odom_msg.pose.pose.position;

  Eigen::Quaterniond quat(is3d.rotation());
  quat.normalize();
  orientation.w = quat.w();
  orientation.x = quat.x();
  orientation.y = quat.y();
  orientation.z = quat.z();

  Eigen::Vector3d pos(is3d.translation());
  position.x = pos(0);
  position.y = pos(1);
  position.z = pos(2);
}

} // end namespace lidar_slam

#endif // LIDAR_COMMON_H
