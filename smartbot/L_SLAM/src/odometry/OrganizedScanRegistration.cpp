
#include "OrganisedScanRegistration.h"
#include "common/math_utils.h"

#include <pcl_conversions/pcl_conversions.h>

#include <ctime>

namespace lidar_slam {

OrganisedScanRegistration::OrganisedScanRegistration(const RegistrationParams &config)
    : ScanRegistration(config), _scanRings(64), _systemDelay(SYSTEM_DELAY) {
  cloudReceiveCount = 0;
};

OrganisedScanRegistration:: ~OrganisedScanRegistration(){
  ROS_INFO("[OrganisedScanRegistration] cloudReceiveCount:%d",cloudReceiveCount);
}

bool OrganisedScanRegistration::setup(ros::NodeHandle &node,
                                  ros::NodeHandle &privateNode) {
  if (!ScanRegistration::setup(node, privateNode)) {
    return false;
  }

  if (privateNode.getParam("scanRings", _scanRings)) {
    ROS_INFO("Set scanRings: %d", _scanRings);
  }
  
  _blindRaduis = privateNode.param<float>("blindRaduis", 2.5);
  _checkTimeDelay = privateNode.param<bool>("checkTimeDelay", false);

   // subscribe to input cloud topic
  _subLaserCloud = node.subscribe<sensor_msgs::PointCloud2>(
      "/organised_scan_points", 2, &OrganisedScanRegistration::handleCloudMessage,
      this, ros::TransportHints().tcpNoDelay(true));
  _cloud_new = false;
  spin_thread = std::thread(&OrganisedScanRegistration::spin, this);

  return true;
}

void OrganisedScanRegistration::spin() {
  ros::Rate rate(500);
  bool status = ros::ok();

  while (status) {
    ros::spinOnce();
    if(_cloud_new)
      process(_cloud_in, _cloud_time);
    status = ros::ok();
    rate.sleep();
  }
}

void OrganisedScanRegistration::handleCloudMessage(
    const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
  cloudReceiveCount++;

  // std::cout << "message time interval" << 1.0 * (clock() - _last_time) / CLOCKS_PER_SEC << "s\n";
  //_last_time = clock();
  if (_systemDelay > 0) {
    _systemDelay--;
    return;
  }
  if((laserCloudMsg->header.seq - _last_seq)!=1){
    ROS_WARN("PointCloud seq jump: %d", laserCloudMsg->header.seq - _last_seq);
  }
  // fetch new input cloud
  pcl::fromROSMsg(*laserCloudMsg, _cloud_in);
  _cloud_time = laserCloudMsg->header.stamp;
  _last_seq = laserCloudMsg->header.seq;
  _cloud_new = true;

  //clock_t beginT = clock(); 
  // process(laserCloudIn, laserCloudMsg->header.stamp);
  //process(cloud_in, laserCloudMsg->header.stamp);
  //clock_t endT = clock();
  // std::cout << "process time: " << 1.0 * (endT - beginT) / CLOCKS_PER_SEC << "s\n";
}

void OrganisedScanRegistration::process(const CloudT &in,
                                    const ros::Time &scanTime) {
  _cloud_new = false;
  if(_checkTimeDelay){
    float time_delay = (ros::Time::now().toSec() - scanTime.toSec());
    if(fabs(time_delay)>0.05){
      ROS_WARN("PointCloud delay %f, if not using rosbag, this may be a problem!", time_delay);
    }    
  }

  size_t cloudSize = in.size();
  int height = in.height;
  int width  = in.width;
  // reset internal buffers and set IMU start state based on current scan time
  reset(scanTime);

  PointIN point;
  std::vector<CloudIN> laserCloudScans(height);

  // extract valid points from input cloud
  for(int row =0; row< height; row++)
  for(int col = 0; col<width; col++){
  //for (int i = 0; i < cloudSize; i++) {
    const auto& p= in.at(col, row);
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    point.intensity = p.intensity;
    // calculate relative scan time based on point orientation
    float relTime = _config.scanPeriod * static_cast<double>(col)/width;
    point.curvature = p.ring + relTime;

    // skip NaN and INF valued points
    if (!pcl_isfinite(point.x) || !pcl_isfinite(point.y) ||
        !pcl_isfinite(point.z)) {
      continue;
    }

    // skip zero valued points AND near points
    if (point.x * point.x + point.y * point.y + point.z * point.z < _blindRaduis*_blindRaduis) {
      continue;
    }


    laserCloudScans[row].push_back(point);
  }

  // construct sorted full resolution cloud
  cloudSize = 0;
  // for (int i = 13; i < 14; i++) {
  for (int i = 0; i < height; i++) {
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
