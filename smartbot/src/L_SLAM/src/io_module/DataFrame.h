#ifndef DATA_FRAME_H__
#define DATA_FRAME_H__

#include <boost/optional.hpp>
#include <g2o/types/slam3d/vertex_se3.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

namespace lidar_slam {

class DataFrame {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // using PointI = pcl::PointXYZI;
  // using CloudI = pcl::PointCloud<PointI>;

  // using Ptr = std::shared_ptr<DataFrame>;
  typedef typename pcl::PointXYZI PointI;
  typedef typename pcl::PointCloud<PointI> Cloud;
  typedef typename pcl::PointCloud<PointI>::Ptr CloudPtr;
  using Ptr = std::shared_ptr<DataFrame>;

  DataFrame(const ros::Time &stamp_, const Eigen::Isometry3d &odom_,
            CloudPtr &cornerCloud_, CloudPtr &surfCloud_)
      : stamp(stamp_), odom(odom_), cornerCloudDS(new Cloud()),
        surfCloudDS(new Cloud()), frame_id(0) {
    cornerCloudDS.swap(cornerCloud_);
    surfCloudDS.swap(surfCloud_);
  }

  ~DataFrame(){};

  inline void setAccumDistance(double accum_distance_) {
    accum_distance = accum_distance_;
  }

  inline void setFrameID(int frame_id_) { frame_id = frame_id_; }

public:
  ros::Time stamp;
  Eigen::Isometry3d odom;
  double accum_distance;
  // CloudPtr cloud;
  // CloudPtr cornerCloud;
  // CloudPtr surfCloud;
  CloudPtr cornerCloudDS;
  CloudPtr surfCloudDS;
  size_t frame_id;
};
}

#endif // DATA_FRAME_H__
