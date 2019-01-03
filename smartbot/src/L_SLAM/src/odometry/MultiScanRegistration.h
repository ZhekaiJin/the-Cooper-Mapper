
#ifndef LIDAR_MULTISCANREGISTRATION_H
#define LIDAR_MULTISCANREGISTRATION_H

#include "ScanRegistration.h"

#include <sensor_msgs/PointCloud2.h>
#include "common/math_utils.h"

#include "lidar_type.h"

namespace lidar_slam {

class MultiScanMapperBase {
public:
  MultiScanMapperBase(){};
  virtual inline int getRingForAngle(const float &angle){};

  virtual const float &getLowerBound() { return _lowerBound; }
  virtual const float &getUpperBound() { return _upperBound; }
  virtual const uint16_t &getNumberOfScanRings() { return _nScanRings; }

protected:
  float _lowerBound;    ///< the vertical angle of the first scan ring
  float _upperBound;    ///< the vertical angle of the last scan ring
  uint16_t _nScanRings; ///< number of scan rings
};

class MultiScanMapperP : public MultiScanMapperBase {
public:
  MultiScanMapperP(const float &lowerBound, const float &upperBound,
                   const uint16_t &nScanRings) {
    _lowerBound = lowerBound;
    _upperBound = upperBound;
    _nScanRings = nScanRings;
  };
  int getRingForAngle(const float &angle) { return scanID_pandar(rad2deg(angle)); }

  static inline MultiScanMapperP *Pandar40() {
    return new MultiScanMapperP(-15.444, 6.96, 40);
  };
};

/** \brief Class realizing a linear mapping from
 * vertical point angle to the
 * corresponding scan ring.
 *
 */
class MultiScanMapper : public MultiScanMapperBase {
public:
  /** \brief Construct a new multi scan mapper instance.
   *
   * @param lowerBound - the lower vertical bound (degrees)
   * @param upperBound - the upper vertical bound (degrees)
   * @param nScanRings - the number of scan rings
   */
  MultiScanMapper(const float &lowerBound = -15, const float &upperBound = 15,
                  const uint16_t &nScanRings = 16) {

    _lowerBound = lowerBound;
    _upperBound = upperBound;
    _nScanRings = nScanRings;
    _factor = (nScanRings - 1) / (upperBound - lowerBound);
  };

  /** \brief Set mapping parameters.
   *
   * @param lowerBound - the lower vertical bound (degrees)
   * @param upperBound - the upper vertical bound (degrees)
   * @param nScanRings - the number of scan rings
   */
  void set(const float &lowerBound, const float &upperBound,
           const uint16_t &nScanRings) {
    _lowerBound = lowerBound;
    _upperBound = upperBound;
    _nScanRings = nScanRings;
    _factor = (nScanRings - 1) / (upperBound - lowerBound);
  };

  /** \brief Map the specified vertical point angle to its ring ID.
   *
   * @param angle the vertical point angle (in rad)
   * @return the ring ID
   */
  inline int getRingForAngle(const float &angle) {
    return int(((angle * 180 / M_PI) - _lowerBound) * _factor + 0.5);
  };

  /** Multi scan mapper for Velodyne VLP-16 according to data sheet. */
  static inline MultiScanMapper *Velodyne_VLP_16() {
    return new MultiScanMapper(-15, 15, 16);
  };

  /** Multi scan mapper for Velodyne HDL-32 according to data sheet. */
  static inline MultiScanMapper *Velodyne_HDL_32() {
    return new MultiScanMapper(-30.67f, 10.67f, 32);
  };

  /** Multi scan mapper for Velodyne HDL-64E according to data sheet. */
  static inline MultiScanMapper *Velodyne_HDL_64E() {
    return new MultiScanMapper(-24.9f, 2, 64);
  };

private:
  // float _lowerBound;    ///< the vertical angle of the first scan ring
  // float _upperBound;    ///< the vertical angle of the last scan ring
  // uint16_t _nScanRings; ///< number of scan rings
  float _factor; ///< linear interpolation factor
public:
  float _azimuth_resolution; ///< The horizontal resolution
};

/** \brief Class for registering point clouds received from multi-laser lidars.
 *
 */
class MultiScanRegistration : virtual public ScanRegistration {
public:
  typedef pcl::PointXYZINormal PointIN;
  typedef pcl::PointCloud<PointIN> CloudIN;

  MultiScanRegistration(
      const RegistrationParams &config = RegistrationParams());

  ~MultiScanRegistration();
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
  void process(const CloudI &in, const ros::Time &scanTime);

protected:
  int _systemDelay; ///< system startup delay counter
  MultiScanMapperBase
      *_scanMapper; ///< mapper for mapping vertical point angles to
                    /// scan ring IDs

  ros::Subscriber _subLaserCloud; ///< input cloud message subscriber
  int cloudReceiveCount;
private:
  static const int SYSTEM_DELAY = 2;
};

} // end namespace lidar_slam

#endif // LIDAR_MULTISCANREGISTRATION_H
