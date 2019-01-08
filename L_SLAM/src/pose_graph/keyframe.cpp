
#include <boost/filesystem.hpp>

#include "keyframe.h"
#include <g2o/types/slam3d/vertex_se3.h>
#include <pcl/io/pcd_io.h>
#include <string>

namespace pose_graph {

KeyFrame::KeyFrame(const ros::Time &stamp_, const Eigen::Isometry3d &odom_,
                   CloudI::Ptr &cornerCloud_, CloudI::Ptr &surfCloud_)
    : stamp(stamp_), odom(odom_), cornerCloud(new CloudI()),
      surfCloud(new CloudI()), node(nullptr), frame_id(0) {
  cornerCloud.swap(cornerCloud_);
  surfCloud.swap(surfCloud_);
}

KeyFrame::~KeyFrame() {}

void KeyFrame::dump(const std::string &directory) {
  if (!boost::filesystem::is_directory(directory)) {
    boost::filesystem::create_directory(directory);
  }

  pcl::io::savePCDFileBinary(
      directory + "/" + std::to_string(frame_id) + "corner.pcd", *cornerCloud);

  pcl::io::savePCDFileBinary(
      directory + "/" + std::to_string(frame_id) + "surf.pcd", *surfCloud);
}

} // namespace pose_graph
