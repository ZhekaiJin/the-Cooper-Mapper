
#ifndef LIDAR_SCANREGISTRATION_H
#define LIDAR_SCANREGISTRATION_H

#include "common/Angle.h"
#include "common/CircularBuffer.h"
#include "common/Vector3.h"
#include "common/ros_utils.h"

#include <pcl/point_cloud.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Imu.h>
#include <stdint.h>
#include <vector>
#include <algorithm>

namespace lidar_slam {

/** \brief A pair describing the start end end index of a range. */
typedef std::pair<size_t, size_t> IndexRange;

/** Point label options. */
enum PointLabel {
  SLOP = 8,
  BLOCKED = 7,
  UNKNOW = 6,
  CONER_PICKED_NEAR = 4, ///< point near picked corner feature
  SURF_PICKED_NEAR = 3,  ///< point near picked surf feature
  CORNER_LESS_SHARP = 2, ///< less sharp corner point
  CORNER_SHARP = 1,      ///< sharp corner point
  SURFACE_LESS_FLAT = 0, ///< less flat surface point
  SURFACE_FLAT = -1,     ///< flat surface point

  ONESIDE_FLAT = 5,     ///< flat surface point

  EDGE_BROKEN = -2,
  NEAR_BLOCK = -3,
  BLIND_BLOCK = -4,       ///< point near blind aera
  MESSY = 9,
};

/** Scan Registration configuration parameters. */
class RegistrationParams {
public:
  RegistrationParams(float scanPeriod_ = 0.1, int imuHistorySize_ = 200,
                     int nFeatureRegions_ = 6, int curvatureRegion_ = 5,
                     int maxCornerSharp_ = 2, int maxSurfaceFlat_ = 4,
                     float lessFlatFilterSize_ = 0.2,
                     float surfaceCurvatureThreshold_ = 0.02,
                     float cornerCurvatureThreshold_ = 1.0,
                     bool cornerCheckEnable_ = true,
                     float blindDegreeThreshold_ = 0.5,
                     std::string curvatureEstimateMethod_ = "distance");

  /** \brief initialize node parameter.
   *
   * @param nh the ROS node handle
   * @return true, if all specified parameters are valid, false if at least one
   * specified parameter is invalid
   */
  bool initialize_params(ros::NodeHandle &nh);

  void param_print() {
    std::cout << "RegistrationParams\n"
              << " ,scanPeriod:" << scanPeriod
              << " ,imuHistorySize:" << imuHistorySize
              << " ,nFeatureRegions:" << nFeatureRegions
              << " ,curvatureRegion:" << curvatureRegion
              << " ,maxCornerSharp:" << maxCornerSharp
              << " ,maxCornerLessSharp:" << maxCornerLessSharp
              << " ,maxSurfaceFlat:" << maxSurfaceFlat
              << " ,lessFlatFilterSize:" << lessFlatFilterSize
              << " ,surfaceCurvatureThreshold:" << surfaceCurvatureThreshold
              << " ,cornerCurvatureThreshold:" << cornerCurvatureThreshold
              << " ,cornerCheckEnable:" << cornerCheckEnable << std::endl;
  }
  /** The time per scan. */
  float scanPeriod;

  /** The size of the IMU history state buffer. */
  int imuHistorySize;

  /** The number of (equally sized) regions used to distribute the feature
   * extraction within a scan. */
  int nFeatureRegions;

  /** The number of surrounding points (+/- region around a point) used to
   * calculate a point curvature. */
  int curvatureRegion;

  /** The maximum number of sharp corner points per feature region. */
  int maxCornerSharp;

  /** The maximum number of less sharp corner points per feature region. */
  int maxCornerLessSharp;

  /** The maximum number of flat surface points per feature region. */
  int maxSurfaceFlat;

  /** The voxel size used for down sizing the remaining less flat surface
   * points. */
  float lessFlatFilterSize;

  /** The curvature threshold below / above a point is considered a flat point.
   */
  float surfaceCurvatureThreshold;

  /** The curvature threshold below / above a point is considered a corner
   * point. */
  float cornerCurvatureThreshold;

  float blindDegreeThreshold;
  float blindThreshold;

  bool cornerCheckEnable;

  /** The curvature estimate method. */
  std::string curvatureEstimateMethod;
};

/** IMU state data. */
typedef struct IMUState {
  /** The time of the measurement leading to this state (in seconds). */
  ros::Time stamp;

  /** The current roll angle. */
  Angle roll;

  /** The current pitch angle. */
  Angle pitch;

  /** The current yaw angle. */
  Angle yaw;

  /** The accumulated global IMU position in 3D space. */
  Vector3 position;

  /** The accumulated global IMU velocity in 3D space. */
  Vector3 velocity;

  /** The current (local) IMU acceleration in 3D space. */
  Vector3 acceleration;

  /** \brief Interpolate between two IMU states.
   *
   * @param start the first IMUState
   * @param end the second IMUState
   * @param ratio the interpolation ratio
   * @param result the target IMUState for storing the interpolation result
   */
  static void interpolate(const IMUState &start, const IMUState &end,
                          const float &ratio, IMUState &result) {
    float invRatio = 1 - ratio;

    result.roll = start.roll.rad() * invRatio + end.roll.rad() * ratio;
    result.pitch = start.pitch.rad() * invRatio + end.pitch.rad() * ratio;
    if (start.yaw.rad() - end.yaw.rad() > M_PI) {
      result.yaw =
          start.yaw.rad() * invRatio + (end.yaw.rad() + 2 * M_PI) * ratio;
    } else if (start.yaw.rad() - end.yaw.rad() < -M_PI) {
      result.yaw =
          start.yaw.rad() * invRatio + (end.yaw.rad() - 2 * M_PI) * ratio;
    } else {
      result.yaw = start.yaw.rad() * invRatio + end.yaw.rad() * ratio;
    }

    result.velocity = start.velocity * invRatio + end.velocity * ratio;
    result.position = start.position * invRatio + end.position * ratio;
  };
} IMUState;

/** \brief Base class for LIDAR scan registration implementations.
 *
 * As there exist various sensor devices, producing differently formatted point
 * clouds,
 * specific implementations are needed for each group of sensor devices to
 * achieve an accurate registration.
 * This class provides common configurations, buffering and processing logic.
 */
class ScanRegistration {
public:
  typedef pcl::PointXYZI PointI;
  typedef pcl::PointCloud<PointI> CloudI;
  typedef pcl::PointXYZINormal PointIN;
  typedef pcl::PointCloud<PointIN> CloudIN;

  explicit ScanRegistration(
      const RegistrationParams &config = RegistrationParams());

  /** \brief Setup component.
   *
   * @param node the ROS node handle
   * @param privateNode the private ROS node handle
   */
  virtual bool setup(ros::NodeHandle &node, ros::NodeHandle &privateNode);

  /** \brief Handler method for IMU messages.
   *
   * @param imuIn the new IMU message
   */
  virtual void handleIMUMessage(const sensor_msgs::Imu::ConstPtr &imuIn);

protected:
  /** \brief Prepare for next scan / sweep.
   *
   * @param scanTime the current scan time
   * @param newSweep indicator if a new sweep has started
   */
  void reset(const ros::Time &scanTime, const bool &newSweep = true);

  /** \breif Check is IMU data is available. */
  inline bool hasIMUData() { return _imuHistory.size() > 0; };

  /** \brief Set up the current IMU transformation for the specified relative
   * time.
   *
   * @param relTime the time relative to the scan time
   */
  void setIMUTransformFor(const float &relTime);

  /** \brief Project the given point to the start of the sweep, using the
   * current IMU state and position shift.
   *
   * @param point the point to project
   */
  void transformToStartIMU(PointIN &point);

  /** \brief Extract features from current laser cloud.
   *
   * @param beginIdx the index of the first scan to extract features from
   */
  void extractFeatures(const uint16_t &beginIdx = 0);

  /** \brief handle the region for the specified point range.
   *
   * Set up region buffers for the specified point range.
   * Compute the curvature and sort the region.
   * Initialse the label.
   *
   * @param startIdx the region start index
   * @param endIdx the region end index
   */
  void setRegionBuffersFor(const size_t &startIdx, const size_t &endIdx);

  /** \brief handle the scan for the specified point range.
   *
   * Set up scan buffers for the specified point range.
   * Mark unreliable points as picked.
   *
   * @param startIdx the scan start index
   * @param endIdx the scan start index
   */
  void setScanBuffersFor(const size_t &startIdx, const size_t &endIdx);

  /** \brief Mark a point and its neighbors as picked.
   *
   * This method will mark neighboring points within the curvature region as
   * picked,
   * as long as they remain within close distance to each other.
   *
   * @param cloudIdx the index of the picked point in the full resolution cloud
   * @param scanIdx the index of the picked point relative to the current scan
   */
  void markAsPicked(const size_t &cloudIdx, const size_t &scanIdx,
                    int label = 1);

  /** \brief Check a point if is corner.
*
*This method will asume that a corner points would at least have one plannar
* surface along two side,
*
*@param cloudIdx the index of the picked point in the full resolution cloud
*@param scanIdx the index of the picked point relative to the current scan
*/
  int pointClassify(const size_t &cloudIdx);
  /** \brief Publish the current result via the respective topics. */
  void publishResult();


  void mergeArray(std::vector<size_t> &sortArray, int first, int mid, int last, std::vector<size_t> &tmp)
  {
    int i = first, j = mid + 1;
    int m = mid, n = last;
    int k = 0;
    while(i <= m && j <= n)
    {
      if(_regionCurvature[sortArray[i]] <= _regionCurvature[sortArray[j]])
        tmp[k ++] = sortArray[i ++];
      else
        tmp[k ++] = sortArray[j ++];
    }

    while(i <= m)
      tmp[k ++] = sortArray[i ++];
    while(j <= n)
      tmp[k ++] = sortArray[j ++];

    for(int i = 0; i < k; i ++)
      sortArray[first + i] = tmp[i];
  }

  void mergeSort(std::vector<size_t> &sortArray, int first, int last, std::vector<size_t> &tmp)
  {
    if(first < last)
    {
      int mid = (first + last) / 2;
      mergeSort(sortArray, first, mid, tmp);
      mergeSort(sortArray, mid + 1, last, tmp);
      mergeArray(sortArray, first, mid, last, tmp);
    }
  }

private:
  /** \brief Try to interpolate the IMU state for the given time.
   *
   * @param relTime the time relative to the scan time
   * @param outputState the output state instance
   */
  void interpolateIMUStateFor(const float &relTime, IMUState &outputState);

protected:
  RegistrationParams _config; ///< registration parameter

  ros::Time _sweepStart; ///< time stamp of beginning of current sweep
  ros::Time _scanTime;   ///< time stamp of most recent scan
  IMUState _imuStart; ///< the interpolated IMU state corresponding to the start
                      /// time of the currently processed laser scan
  IMUState _imuCur; ///< the interpolated IMU state corresponding to the time of
                    /// the currently processed laser scan point
  Vector3 _imuPositionShift; ///< position shift between accumulated IMU
                             /// position and interpolated IMU position
  size_t _imuIdx;            ///< the current index in the IMU history
  CircularBuffer<IMUState>
      _imuHistory; ///< history of IMU states for cloud registration

  CloudIN _laserCloud;                  ///< full resolution input cloud
  std::vector<IndexRange> _scanIndices; ///< start and end indices of the
                                        /// individual scans withing the full
  /// resolution cloud

  CloudI _cornerPointsSharp;                ///< sharp corner points cloud
  CloudI _cornerPointsLessSharp;            ///< less sharp corner points cloud
  CloudI _surfacePointsFlat;                ///< flat surface points cloud
  CloudI _surfacePointsLessFlat;            ///< less flat surface points cloud
  pcl::PointCloud<pcl::PointXYZ> _imuTrans; ///< IMU transformation information

  std::vector<float> _regionCurvature;  ///< point curvature buffer
  std::vector<PointLabel> _regionLabel; ///< point label buffer
  std::vector<size_t>
      _regionSortIndices; ///< sorted region indices based on point curvature
  std::vector<size_t>
      _swapRegionSortIndices;
  std::vector<int>
      _scanNeighborPicked; ///< flag if neighboring point was already picked

  ros::Subscriber _subImu; ///< IMU message subscriber

  ros::Publisher _pubLaserCloud; ///< full resolution cloud message publisher
  ros::Publisher
      _pubCornerPointsSharp; ///< sharp corner cloud message publisher
  ros::Publisher
      _pubCornerPointsLessSharp; ///< less sharp corner cloud message publisher
  ros::Publisher _pubSurfPointsFlat; ///< flat surface cloud message publisher
  ros::Publisher
      _pubSurfPointsLessFlat;  ///< less flat surface cloud message publisher
  ros::Publisher _pubImuTrans; ///< IMU transformation message publisher

  // for debug
  CloudI _pointsBlind;
  CloudI _pointsBlock;
  CloudI _pointsSlop;
  CloudI _pointsCurvature;

  ros::Publisher _pubPointsBlind; ///< full resolution cloud message publisher
  ros::Publisher _pubPointsBlock; ///< sharp corner cloud message publisher
  ros::Publisher _pubPointsSlop;  ///< less sharp corner cloud message publisher
  ros::Publisher _pubCurvature;   ///< less sharp corner cloud message publisher
};

} // end namespace lidar_slam

#endif // LIDAR_SCANREGISTRATION_H
