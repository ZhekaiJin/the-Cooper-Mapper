
#ifndef LIDAR_ORGANIZED_SCANREGISTRATION_H
#define LIDAR_ORGANIZED_SCANREGISTRATION_H

#include "ScanRegistration.h"

#include "common/math_utils.h"
#include <sensor_msgs/PointCloud2.h>

#include "point_types.h"
#include <thread>

namespace lidar_slam {

/** \brief Class for registering point clouds received from multi-laser lidars.
 *
 */
class OrganisedScanRegistration : virtual public ScanRegistration {
public:
  typedef pcl::PointXYZINormal PointIN;
  typedef pcl::PointCloud<PointIN> CloudIN;
  typedef driver_pointcloud::PointXYZIT PointT;
  typedef pcl::PointCloud<PointT> CloudT;

  OrganisedScanRegistration(
      const RegistrationParams &config = RegistrationParams());

  ~OrganisedScanRegistration();
  /** \brief Setup component in active mode.
   *
   * @param node the ROS node handle
   * @param privateNode the private ROS node handle
   */
  bool setup(ros::NodeHandle &node, ros::NodeHandle &privateNode);

  /** \brief Handler method for input cloud messages.
   *
   * @param laserCloudMsg the new input cloud message to process
   */
  void
  handleCloudMessage(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);

  /** \brief Process a new input cloud.
   *
   * @param laserCloudIn the new input cloud to process
   * @param scanTime the scan (message) timestamp
   */
  void process(const CloudT &in, const ros::Time &scanTime);

  void spin();

protected:
  int _systemDelay; ///< system startup delay counter
  std::thread spin_thread;

  ros::Subscriber _subLaserCloud; ///< input cloud message subscriber
  int cloudReceiveCount;
  int _scanRings;
  float _blindRaduis;
  bool _checkTimeDelay;
private:
  static const int SYSTEM_DELAY = 2;
  CloudT _cloud_in;
  ros::Time _cloud_time;
  bool _cloud_new;
  int _last_seq;
  clock_t _last_time;
};

} // end namespace lidar_slam

#endif // LIDAR_ORGANIZED_SCANREGISTRATION_H
