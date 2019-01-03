
#include "LaserMappingLocal.h"

namespace lidar_slam {

LaserMappingLocal::LaserMappingLocal() {}

LaserMappingLocal::~LaserMappingLocal() {}

bool LaserMappingLocal::init(ros::NodeHandle &node,
                               ros::NodeHandle &privateNode) {
  if (setup(node, privateNode)) {

    _inputFrameSkip = 0;
    spin_thread = std::thread(&LaserMappingLocal::spin, this);
    return true;
  }
  return false;
}

void LaserMappingLocal::spin() {
  ros::Rate rate(500);
  bool status = ros::ok();

  while (status) {
    ros::spinOnce();
    process();
    status = ros::ok();
    rate.sleep();
  }
}

void LaserMappingLocal::process() {
  if (!hasNewData()) {
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
  featureMapUpdate();
  publishResult();
}

void LaserMappingLocal::prepareFeatureSurround() {

  _local_feature_map.getSurroundFeature(_laserCloudCornerFromMap,
                                        _laserCloudSurfFromMap);
  return;
}

void LaserMappingLocal::featureMapUpdate() {
  Eigen::Isometry3f transformf;
  convertTransform(_transformTobeMapped, transformf);
  Eigen::Isometry3d transformd;
  transformd.matrix() = transformf.matrix().cast<double>();
  lidar_slam::transformPointCloud(*_laserCloudCornerStackDS,
                                  *_laserCloudCornerStackDS, transformf);
  lidar_slam::transformPointCloud(*_laserCloudSurfStackDS,
                                  *_laserCloudSurfStackDS, transformf);

  DataFrame::Ptr frame(new DataFrame(_timeLaserOdometry, transformd,
                                     _laserCloudCornerStackDS,
                                     _laserCloudSurfStackDS));
  _local_feature_map.addDataFrame(frame);
  return;
}

} // end namespace lidar_slam
