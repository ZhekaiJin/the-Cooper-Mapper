
#include "LaserMapping.h"

namespace lidar_slam {

using std::sqrt;
using std::fabs;
using std::asin;
using std::atan2;
using std::pow;

LaserMapping::LaserMapping() {}

LaserMapping::~LaserMapping() {}

bool LaserMapping::init(ros::NodeHandle &node, ros::NodeHandle &privateNode) {
  if (setup(node, privateNode)) {

    _inputFrameSkip = 0;

    spin_thread = std::thread(&LaserMapping::spin, this);
    return true;
  }
  return false;
}

void LaserMapping::spin() {
  ros::Rate rate(500);
  bool status = ros::ok();

  while (status) {
    ros::spinOnce();
    process();
    status = ros::ok();
    rate.sleep();
  }
}

void LaserMapping::process() {
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

} // end namespace lidar_slam
