
#include "MultiScanRegistration.h"
#include "common/math_utils.h"

#include <pcl_conversions/pcl_conversions.h>

namespace lidar_slam {

MultiScanRegistration::MultiScanRegistration(const RegistrationParams &config)
    : ScanRegistration(config), _systemDelay(SYSTEM_DELAY) {
  cloudReceiveCount = 0;
};

MultiScanRegistration:: ~MultiScanRegistration(){
  ROS_INFO("[MultiScanRegistration] cloudReceiveCount:%d",cloudReceiveCount);
}

bool MultiScanRegistration::setup(ros::NodeHandle &node,
                                  ros::NodeHandle &privateNode) {
  if (!ScanRegistration::setup(node, privateNode)) {
    return false;
  }

  // fetch scan mapping params
  std::string lidarName;

  if (privateNode.getParam("lidar", lidarName)) {
    if (lidarName == "VLP-16") {
      _scanMapper = MultiScanMapper::Velodyne_VLP_16();
    } else if (lidarName == "HDL-32") {
      _scanMapper = MultiScanMapper::Velodyne_HDL_32();
    } else if (lidarName == "HDL-64E") {
      _scanMapper = MultiScanMapper::Velodyne_HDL_64E();
    } else if (lidarName == "Pandar40") {
      _scanMapper = MultiScanMapperP::Pandar40();
    } else {
      ROS_ERROR("Invalid lidar parameter: %s (only \"VLP-16\", \"HDL-32\" ,"
                "\"HDL-64E\ and \"Pandar40\ are supported)",
                lidarName.c_str());
      return false;
    }

    ROS_INFO("Set  %s  scan mapper.", lidarName.c_str());
    if (!privateNode.hasParam("scanPeriod")) {
      _config.scanPeriod = 0.1;
      ROS_INFO("Set scanPeriod: %f", _config.scanPeriod);
    }
  } else {
    float vAngleMin, vAngleMax;
    int nScanRings;

    if (privateNode.getParam("minVerticalAngle", vAngleMin) &&
        privateNode.getParam("maxVerticalAngle", vAngleMax) &&
        privateNode.getParam("nScanRings", nScanRings)) {
      if (vAngleMin >= vAngleMax) {
        ROS_ERROR("Invalid vertical range (min >= max)");
        return false;
      } else if (nScanRings < 2) {
        ROS_ERROR("Invalid number of scan rings (n < 2)");
        return false;
      }

      //_scanMapper.set(vAngleMin, vAngleMax, nScanRings);
      ROS_INFO(
          "Set linear scan mapper from %g to %g degrees with %d scan rings.",
          vAngleMin, vAngleMax, nScanRings);
    }
  }

  // subscribe to input cloud topic
  _subLaserCloud = node.subscribe<sensor_msgs::PointCloud2>(
      "/multi_scan_points", 2, &MultiScanRegistration::handleCloudMessage,
      this);

  return true;
}

void MultiScanRegistration::handleCloudMessage(
    const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
  cloudReceiveCount++;

  if (_systemDelay > 0) {
    _systemDelay--;
    return;
  }

  // fetch new input cloud
  CloudI cloud_in;
  pcl::fromROSMsg(*laserCloudMsg, cloud_in);

  // process(laserCloudIn, laserCloudMsg->header.stamp);
  process(cloud_in, laserCloudMsg->header.stamp);
}

void MultiScanRegistration::process(const CloudI &in,
                                    const ros::Time &scanTime) {
  size_t cloudSize = in.size();

  // reset internal buffers and set IMU start state based on current scan time
  reset(scanTime);

  // determine scan start and end orientations
  float startOri = -std::atan2(in[0].y, in[0].x);
  float endOri =
      -std::atan2(in[cloudSize - 1].y, in[cloudSize - 1].x) + 2 * float(M_PI);
  if (endOri - startOri > 3 * M_PI) {
    endOri -= 2 * M_PI;
  } else if (endOri - startOri < M_PI) {
    endOri += 2 * M_PI;
  }
  //_scanMapper._azimuth_resolution = (endOri -
  // startOri)*_scanMapper.getNumberOfScanRings()/cloudSize;
  // ROS_INFO("_scanMapper._azimuth_resolution:%f,%f,%f,%d",_scanMapper._azimuth_resolution,startOri,endOri,cloudSize);
  bool halfPassed = false;
  PointIN point;
  std::vector<CloudIN> laserCloudScans(_scanMapper->getNumberOfScanRings());

  // extract valid points from input cloud
  for (int i = 0; i < cloudSize; i++) {
    point.x = in[i].y;
    point.y = in[i].z;
    point.z = in[i].x;
    point.intensity = in[i].intensity;

    // skip NaN and INF valued points
    if (!pcl_isfinite(point.x) || !pcl_isfinite(point.y) ||
        !pcl_isfinite(point.z)) {
      continue;
    }

    // skip zero valued points
    if (point.x * point.x + point.y * point.y + point.z * point.z < 0.0001) {
      continue;
    }

    // calculate vertical point angle and scan ID
    float angle =
        std::atan(point.y / std::sqrt(point.x * point.x + point.z * point.z));
    int scanID = _scanMapper->getRingForAngle(angle);
    if (scanID >= _scanMapper->getNumberOfScanRings() || scanID < 0) {
      continue;
    }

    // calculate horizontal point angle
    float ori = -std::atan2(point.x, point.z);
    if (!halfPassed) {
      if (ori < startOri - M_PI / 2) {
        ori += 2 * M_PI;
      } else if (ori > startOri + M_PI * 3 / 2) {
        ori -= 2 * M_PI;
      }

      if (ori - startOri > M_PI) {
        halfPassed = true;
      }
    } else {
      ori += 2 * M_PI;

      if (ori < endOri - M_PI * 3 / 2) {
        ori += 2 * M_PI;
      } else if (ori > endOri + M_PI / 2) {
        ori -= 2 * M_PI;
      }
    }

    // calculate relative scan time based on point orientation
    float relTime = _config.scanPeriod * (ori - startOri) / (endOri - startOri);
    point.curvature = scanID + relTime;

    // project point to the start of the sweep using corresponding IMU data
    if (hasIMUData()) {
      setIMUTransformFor(relTime);
      transformToStartIMU(point);
    }

    laserCloudScans[scanID].push_back(point);
  }

  // construct sorted full resolution cloud
  cloudSize = 0;
  // for (int i = 13; i < 14; i++) {
  for (int i = 0; i < _scanMapper->getNumberOfScanRings(); i++) {
    // std::cout<<i<<":"<<laserCloudScans[i].size()<<std::endl;
    _laserCloud += laserCloudScans[i];

    IndexRange range(cloudSize, 0);
    cloudSize += laserCloudScans[i].size();
    range.second = cloudSize > 0 ? cloudSize - 1 : 0;
    _scanIndices.push_back(range);
  }

  // for(int i = 0; i < _scanIndices.size(); i++) {
  //  std::cout<<i<<":"<<_scanIndices[i].first<<","<<_scanIndices[i].second<<std::endl;
  //}
  // extract features
  extractFeatures();

  // publish result
  publishResult();
}

} // end namespace lidar_slam
