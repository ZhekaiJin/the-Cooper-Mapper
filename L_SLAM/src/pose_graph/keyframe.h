#ifndef KEY_FRAME_H__
#define KEY_FRAME_H__

#include <boost/optional.hpp>
#include <g2o/types/slam3d/vertex_se3.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

namespace pose_graph {

struct KeyFrame {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using PointI = pcl::PointXYZI;
  using CloudI = pcl::PointCloud<PointI>;

  using Ptr = std::shared_ptr<KeyFrame>;

  KeyFrame(const ros::Time &stamp, const Eigen::Isometry3d &odom,
           CloudI::Ptr &cornerCloud, CloudI::Ptr &surfCloud);
  ~KeyFrame();

  void dump(const std::string &directory);

  inline void setAccumDistance(double accum_distance_) {
    accum_distance = accum_distance_;
  }

  inline void setFrameID(int frame_id_) { frame_id = frame_id_; }

public:
  ros::Time stamp;
  Eigen::Isometry3d odom;
  double accum_distance;
  CloudI::ConstPtr cloud;
  CloudI::Ptr cornerCloud;
  CloudI::Ptr surfCloud;

  boost::optional<Eigen::Vector3d> utm_coord; // UTM coord obtained by GPS
  boost::optional<Eigen::Vector4d> imu_quat;  //  quat obtained by IMU

  size_t frame_id;
  g2o::VertexSE3 *node; // node instance
};
}

#endif // KEY_FRAME_H__
