
#include "ScanRegistration.h"
#include "common/math_utils.h"
#include "common/pcl_util.h"
#include <pcl/filters/voxel_grid.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Eigenvalues>
#include <Eigen/QR>

namespace lidar_slam {

RegistrationParams::RegistrationParams(
    float scanPeriod_, int imuHistorySize_, int nFeatureRegions_,
    int curvatureRegion_, int maxCornerSharp_, int maxSurfaceFlat_,
    float lessFlatFilterSize_, float surfaceCurvatureThreshold_,
    float cornerCurvatureThreshold_, bool cornerCheckEnable_,
    float blindDegreeThreshold_, std::string curvatureEstimateMethod_)
    : scanPeriod(scanPeriod_), imuHistorySize(imuHistorySize_),
      nFeatureRegions(nFeatureRegions_), curvatureRegion(curvatureRegion_),
      maxCornerSharp(maxCornerSharp_), maxCornerLessSharp(10 * maxCornerSharp_),
      maxSurfaceFlat(maxSurfaceFlat_), lessFlatFilterSize(lessFlatFilterSize_),
      surfaceCurvatureThreshold(surfaceCurvatureThreshold_),
      cornerCurvatureThreshold(cornerCurvatureThreshold_),
      cornerCheckEnable(cornerCheckEnable_),
      blindDegreeThreshold(blindDegreeThreshold_),
      blindThreshold(cos(deg2rad(blindDegreeThreshold))),
      curvatureEstimateMethod(curvatureEstimateMethod_){

      };

bool RegistrationParams::initialize_params(ros::NodeHandle &nh) {
  scanPeriod = nh.param<float>("scanPeriod", 0.1);
  imuHistorySize = nh.param<int>("imuHistorySize", 200);
  nFeatureRegions = nh.param<int>("nFeatureRegions", 6);
  curvatureRegion = nh.param<int>("curvatureRegion", 5);
  maxCornerSharp = nh.param<int>("maxCornerSharp", 2);
  maxCornerLessSharp = nh.param<int>("maxCornerLessSharp", 20);
  maxSurfaceFlat = nh.param<int>("maxSurfaceFlat", 4);
  surfaceCurvatureThreshold =
      nh.param<float>("surfaceCurvatureThreshold", 0.02);
  cornerCurvatureThreshold = nh.param<float>("cornerCurvatureThreshold", 1.0);
  lessFlatFilterSize = nh.param<float>("lessFlatFilterSize", 0.2);
  cornerCheckEnable = nh.param<bool>("cornerCheckEnable", true);
  blindDegreeThreshold = nh.param<float>("blindDegreeThreshold", 0.5);
  blindThreshold = (cos(deg2rad(blindDegreeThreshold))), param_print();

  return true;
}

ScanRegistration::ScanRegistration(const RegistrationParams &config)
    : _config(config), _sweepStart(), _scanTime(), _imuStart(), _imuCur(),
      _imuIdx(0), _imuHistory(_config.imuHistorySize), _laserCloud(),
      _cornerPointsSharp(), _cornerPointsLessSharp(), _surfacePointsFlat(),
      _surfacePointsLessFlat(), _imuTrans(4, 1), _regionCurvature(),
      _regionLabel(), _regionSortIndices(), _scanNeighborPicked() {}

bool ScanRegistration::setup(ros::NodeHandle &node,
                             ros::NodeHandle &privateNode) {
  if (!_config.initialize_params(privateNode)) {
    return false;
  }
  _imuHistory.ensureCapacity(_config.imuHistorySize);

  // subscribe to IMU topic
  _subImu = node.subscribe<sensor_msgs::Imu>(
      "/imu/data", 50, &ScanRegistration::handleIMUMessage, this);

  // advertise scan registration topics
  _pubLaserCloud =
      node.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 2);
  _pubCornerPointsSharp =
      node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 2);
  _pubCornerPointsLessSharp =
      node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 2);
  _pubSurfPointsFlat =
      node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 2);
  _pubSurfPointsLessFlat =
      node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 2);
  _pubImuTrans = node.advertise<sensor_msgs::PointCloud2>("/imu_trans", 5);
  _pubPointsBlind = node.advertise<sensor_msgs::PointCloud2>("/point_blind", 2);
  _pubPointsBlock = node.advertise<sensor_msgs::PointCloud2>("/point_block", 2);
  _pubPointsSlop = node.advertise<sensor_msgs::PointCloud2>("/point_slop", 2);
  _pubCurvature =
      node.advertise<sensor_msgs::PointCloud2>("/point_curvature", 2);
  return true;
}

void ScanRegistration::handleIMUMessage(
    const sensor_msgs::Imu::ConstPtr &imuIn) {
  double roll, pitch, yaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imuIn->orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  Vector3 acc;
  acc.x() = float(imuIn->linear_acceleration.y - sin(roll) * cos(pitch) * 9.81);
  acc.y() = float(imuIn->linear_acceleration.z - cos(roll) * cos(pitch) * 9.81);
  acc.z() = float(imuIn->linear_acceleration.x + sin(pitch) * 9.81);

  IMUState newState;
  newState.stamp = imuIn->header.stamp;
  newState.roll = roll;
  newState.pitch = pitch;
  newState.yaw = yaw;
  newState.acceleration = acc;

  if (_imuHistory.size() > 0) {
    // accumulate IMU position and velocity over time
    rotateZXY(acc, newState.roll, newState.pitch, newState.yaw);

    const IMUState &prevState = _imuHistory.last();
    float timeDiff = float((newState.stamp - prevState.stamp).toSec());
    newState.position = prevState.position + (prevState.velocity * timeDiff) +
                        (0.5 * acc * timeDiff * timeDiff);
    newState.velocity = prevState.velocity + acc * timeDiff;
  }

  _imuHistory.push(newState);
}

void ScanRegistration::reset(const ros::Time &scanTime, const bool &newSweep) {
  _scanTime = scanTime;

  // re-initialize IMU start index and state
  _imuIdx = 0;
  if (hasIMUData()) {
    interpolateIMUStateFor(0, _imuStart);
  }

  // clear internal cloud buffers at the beginning of a sweep
  if (newSweep) {
    _sweepStart = scanTime;

    // clear cloud buffers
    _laserCloud.clear();
    _cornerPointsSharp.clear();
    _cornerPointsLessSharp.clear();
    _surfacePointsFlat.clear();
    _surfacePointsLessFlat.clear();
    _pointsBlind.clear();
    _pointsBlock.clear();
    _pointsSlop.clear();
    _pointsCurvature.clear();
    // clear scan indices vector
    _scanIndices.clear();
  }
}

void ScanRegistration::setIMUTransformFor(const float &relTime) {
  interpolateIMUStateFor(relTime, _imuCur);

  float relSweepTime = (_scanTime - _sweepStart).toSec() + relTime;
  _imuPositionShift =
      _imuCur.position - _imuStart.position - _imuStart.velocity * relSweepTime;
}

void ScanRegistration::transformToStartIMU(PointIN &point) {
  // rotate point to global IMU system
  rotateZXY(point, _imuCur.roll, _imuCur.pitch, _imuCur.yaw);

  // add global IMU position shift
  point.x += _imuPositionShift.x();
  point.y += _imuPositionShift.y();
  point.z += _imuPositionShift.z();

  // rotate point back to local IMU system relative to the start IMU state
  rotateYXZ(point, -_imuStart.yaw, -_imuStart.pitch, -_imuStart.roll);
}

void ScanRegistration::interpolateIMUStateFor(const float &relTime,
                                              IMUState &outputState) {
  double timeDiff = (_scanTime - _imuHistory[_imuIdx].stamp).toSec() + relTime;
  while (_imuIdx < _imuHistory.size() - 1 && timeDiff > 0) {
    _imuIdx++;
    timeDiff = (_scanTime - _imuHistory[_imuIdx].stamp).toSec() + relTime;
  }

  if (_imuIdx == 0 || timeDiff > 0) {
    outputState = _imuHistory[_imuIdx];
  } else {
    float ratio =
        -timeDiff /
        (_imuHistory[_imuIdx].stamp - _imuHistory[_imuIdx - 1].stamp).toSec();
    IMUState::interpolate(_imuHistory[_imuIdx], _imuHistory[_imuIdx - 1], ratio,
                          outputState);
  }
}

void ScanRegistration::extractFeatures(const uint16_t &beginIdx) {
  // extract features from individual scans

  int blind_points = 0;
  int slope_points = 0;
  int block_points = 0;
  int lessFlat_points = 0;
  int mindis_points = 0;
  size_t nScans = _scanIndices.size();
  for (size_t i = beginIdx; i < nScans; i++) {
    CloudI::Ptr surfPointsLessFlatScan(new CloudI);
    size_t scanStartIdx = _scanIndices[i].first;
    size_t scanEndIdx = _scanIndices[i].second;

    // skip empty scans
    if (scanEndIdx <= scanStartIdx + 2 * _config.curvatureRegion) {
      continue;
    }

    // reset scan buffers
    setScanBuffersFor(scanStartIdx, scanEndIdx);

    // for debug
    /*
    for (int i = scanStartIdx; i <= scanEndIdx; i++) {
      switch (_scanNeighborPicked[i]) {
      case 4: {
        blind_points++;
        break;
      }
      case 3: {
        blind_points++;
        break;
      }
      case 2: {
        blind_points++;
        break;
      }
      case 1: {
        slope_points++;
        break;
      }
      }
    }
    */
    // for debug visulization
    /*
    for (int ii = scanStartIdx; ii <= scanEndIdx; ii++) {
      if (_scanNeighborPicked[ii - scanStartIdx] == 4) {
        _pointsBlind.push_back(toXYZI(_laserCloud[ii]));
      } else if (_scanNeighborPicked[ii - scanStartIdx] > 1) {
        _pointsBlock.push_back(toXYZI(_laserCloud[ii]));
      } else if (_scanNeighborPicked[ii - scanStartIdx] == 1) {
        _pointsSlop.push_back(toXYZI(_laserCloud[ii]));
      }
    }*/

    // extract features from equally sized scan regions
    for (int j = 0; j < _config.nFeatureRegions; j++) {
      size_t sp = ((scanStartIdx + _config.curvatureRegion) *
                       (_config.nFeatureRegions - j) +
                   (scanEndIdx - _config.curvatureRegion) * j) /
                  _config.nFeatureRegions;
      size_t ep = ((scanStartIdx + _config.curvatureRegion) *
                       (_config.nFeatureRegions - 1 - j) +
                   (scanEndIdx - _config.curvatureRegion) * (j + 1)) /
                      _config.nFeatureRegions -
                  1;

      // skip empty regions
      if (ep <= sp) {
        continue;
      }

      size_t regionSize = ep - sp + 1;
      setRegionBuffersFor(sp, ep);

      // extract flat surface features
      int surfPickedNum = 0;
      for (int k = 0; k < regionSize && surfPickedNum < _config.maxSurfaceFlat;
           k++) {
        size_t idx = _regionSortIndices[k];
        size_t scanIdx = idx - scanStartIdx;
        size_t regionIdx = idx - sp;

        if (_scanNeighborPicked[scanIdx] != SURF_PICKED_NEAR &&
            _regionCurvature[regionIdx] < _config.surfaceCurvatureThreshold) {

          surfPickedNum++;
          _regionLabel[regionIdx] = SURFACE_FLAT;
          _surfacePointsFlat.push_back(toXYZI(_laserCloud[idx]));

          markAsPicked(idx, scanIdx, SURF_PICKED_NEAR);
        }
      }

      // extract less flat surface features and edge_broken
      for (int k = 0; k < regionSize; k++) {
        size_t idx = sp + k;
        size_t scanIdx = idx - scanStartIdx;

        if (_regionCurvature[k] < _config.surfaceCurvatureThreshold) {
          surfPointsLessFlatScan->push_back(toXYZI(_laserCloud[idx]));
          mindis_points++;
          if (_regionLabel[k] != SURFACE_FLAT)
            _regionLabel[k] = SURFACE_LESS_FLAT;
        }
        if (_scanNeighborPicked[scanIdx] == EDGE_BROKEN) {
          _cornerPointsSharp.push_back(toXYZI(_laserCloud[idx]));
          _cornerPointsLessSharp.push_back(toXYZI(_laserCloud[idx]));
          _regionLabel[k] = CORNER_SHARP;
          _pointsBlock.push_back(toXYZI(_laserCloud[idx]));
        }
      }

      // extract  features
      int cornerPickedNum = 0;
      surfPickedNum = 0;
      for (size_t k = regionSize; k > 0;) {
        size_t idx = _regionSortIndices[--k];
        size_t scanIdx = idx - scanStartIdx;
        size_t regionIdx = idx - sp;

        if (_regionCurvature[regionIdx] < _config.surfaceCurvatureThreshold)
          break;

        int point_label = pointClassify(idx);
        switch (point_label) {
        case MESSY: {
          //_pointsBlock.push_back(toXYZI(_laserCloud[idx]));
          break;
        }
        case SURFACE_FLAT: {
          _regionLabel[regionIdx] = SURFACE_FLAT;
          if (surfPickedNum < _config.maxSurfaceFlat) {
            surfPickedNum++;
            //_surfacePointsFlat.push_back(toXYZI(_laserCloud[idx]));
          }
          surfPointsLessFlatScan->push_back(toXYZI(_laserCloud[idx]));
          _pointsBlind.push_back(toXYZI(_laserCloud[idx]));
          break;
        }
        case CORNER_SHARP: {
          if (_scanNeighborPicked[scanIdx] > EDGE_BROKEN) {
            _regionLabel[regionIdx] = CORNER_SHARP;
            if (cornerPickedNum < _config.maxCornerSharp) {
              cornerPickedNum++;
              _cornerPointsSharp.push_back(toXYZI(_laserCloud[idx]));
            }
            _cornerPointsLessSharp.push_back(toXYZI(_laserCloud[idx]));
            _pointsSlop.push_back(toXYZI(_laserCloud[idx]));
          }
          break;
        }
        case ONESIDE_FLAT: {
          _regionLabel[regionIdx] = ONESIDE_FLAT;
          if (surfPickedNum < _config.maxSurfaceFlat) {
            surfPickedNum++;
            _surfacePointsFlat.push_back(toXYZI(_laserCloud[idx]));
          }
          surfPointsLessFlatScan->push_back(toXYZI(_laserCloud[idx]));
          _pointsCurvature.push_back(toXYZI(_laserCloud[idx]));
          break;
        }
        }
        /*
        if (point_label == 0) {
          _pointsBlock.push_back(toXYZI(_laserCloud[idx]));
          continue;

        } else if (point_label == 1) {
          _regionLabel[regionIdx] = SURFACE_FLAT;
          //_surfacePointsFlat.push_back(toXYZI(_laserCloud[idx]));
          surfPointsLessFlatScan->push_back(toXYZI(_laserCloud[idx]));
          _pointsBlind.push_back(toXYZI(_laserCloud[idx]));

          // markAsPicked(idx, scanIdx, 12);
        } else if (point_label == 2 && _scanNeighborPicked[scanIdx] == 0) {
          _regionLabel[regionIdx] = CORNER_SHARP;
          _cornerPointsSharp.push_back(toXYZI(_laserCloud[idx]));
          _cornerPointsLessSharp.push_back(toXYZI(_laserCloud[idx]));
          _pointsSlop.push_back(toXYZI(_laserCloud[idx]));

        } else if (point_label == 3) {
          _regionLabel[regionIdx] = SURFACE_FLAT;
          //_surfacePointsFlat.push_back(toXYZI(_laserCloud[idx]));
          surfPointsLessFlatScan->push_back(toXYZI(_laserCloud[idx]));

          if (_scanNeighborPicked[scanIdx] == 0 &&
              _regionCurvature[regionIdx] > 9.0) {
            _regionLabel[regionIdx] = CORNER_SHARP;
            _cornerPointsSharp.push_back(toXYZI(_laserCloud[idx]));
            _cornerPointsLessSharp.push_back(toXYZI(_laserCloud[idx]));
            // markAsPicked(idx, scanIdx, 13);
            _pointsCurvature.push_back(toXYZI(_laserCloud[idx]));
          }
        }*/
      }
    }

    // down size less flat surface point cloud of current scan
    CloudI surfPointsLessFlatScanDS;
    pcl::VoxelGrid<PointI> downSizeFilter;
    downSizeFilter.setInputCloud(surfPointsLessFlatScan);
    downSizeFilter.setLeafSize(_config.lessFlatFilterSize,
                               _config.lessFlatFilterSize,
                               _config.lessFlatFilterSize);
    downSizeFilter.filter(surfPointsLessFlatScanDS);
    lessFlat_points += surfPointsLessFlatScan->points.size();
    _surfacePointsLessFlat += surfPointsLessFlatScanDS;
  }
/*
  std::cout << "[Features:]\n"
            << " ,blind_points:" << blind_points
            << " ,slope_points:" << slope_points
            << " ,block_points:" << block_points
            << " ,totoal_points:" << _scanIndices[nScans - 1].second
            << " ,lessFlat_points:" << lessFlat_points
            << " ,mindis_points:" << mindis_points << std::endl;

  ROS_WARN("sharp:%zu, less sharp:%zu", _cornerPointsSharp.points.size(),
           _cornerPointsLessSharp.points.size());
  ROS_WARN("surf:%zu, less surf:%zu", _surfacePointsFlat.points.size(),
           _surfacePointsLessFlat.points.size());
  ROS_WARN("blind:%zu, slop:%zu", _pointsBlind.points.size(),
           _pointsSlop.points.size());
  ROS_WARN("block:%zu", _pointsBlock.points.size());
  ROS_WARN("Curv:%zu", _pointsCurvature.points.size());*/
}

void ScanRegistration::setRegionBuffersFor(const size_t &startIdx,
                                           const size_t &endIdx) {
  // resize buffers
  size_t regionSize = endIdx - startIdx + 1;
  _regionCurvature.resize(regionSize);
  _regionSortIndices.resize(regionSize);
  _swapRegionSortIndices.resize(regionSize);
  _regionLabel.assign(regionSize, UNKNOW);

  // calculate point curvatures and reset sort indices
  float pointWeight = -2 * _config.curvatureRegion;

  for (size_t i = startIdx, regionIdx = 0; i <= endIdx; i++, regionIdx++) {
    float diffX = pointWeight * _laserCloud[i].x;
    float diffY = pointWeight * _laserCloud[i].y;
    float diffZ = pointWeight * _laserCloud[i].z;

    for (int j = 1; j <= _config.curvatureRegion; j++) {
      diffX += _laserCloud[i + j].x + _laserCloud[i - j].x;
      diffY += _laserCloud[i + j].y + _laserCloud[i - j].y;
      diffZ += _laserCloud[i + j].z + _laserCloud[i - j].z;
    }

    _regionCurvature[regionIdx] = diffX * diffX + diffY * diffY + diffZ * diffZ;
    _regionSortIndices[regionIdx] = i - startIdx;
  }

  // printf("40 * %d * %d * 8.5 = %d\n", _config.nFeatureRegions, regionSize, 40 * _config.nFeatureRegions * regionSize * 8.5);
  // sort point curvatures
  mergeSort(_regionSortIndices, 0, regionSize - 1, _swapRegionSortIndices);
  for(int i = 0; i < regionSize; i ++)
    _regionSortIndices[i] = _regionSortIndices[i] + startIdx;
  // for (size_t i = 1; i < regionSize; i++) {
  //   for (size_t j = i; j >= 1; j--) {
  //     if (_regionCurvature[_regionSortIndices[j] - startIdx] <
  //         _regionCurvature[_regionSortIndices[j - 1] - startIdx]) {
  //       std::swap(_regionSortIndices[j], _regionSortIndices[j - 1]);
  //     }
  //   }
  // }
}

void ScanRegistration::setScanBuffersFor(const size_t &startIdx,
                                         const size_t &endIdx) {
  // resize buffers
  size_t scanSize = endIdx - startIdx + 1;
  _scanNeighborPicked.assign(scanSize, 0);

  for (int i = 0; i < _config.curvatureRegion; ++i) {
    const PointIN &point = (_laserCloud[startIdx + i]);
    const PointIN &nextPoint = (_laserCloud[startIdx + i + 1]);
    if (calcCosAngleDiff(point, nextPoint) < _config.blindThreshold) {
      std::fill_n(&_scanNeighborPicked[i], _config.curvatureRegion + 1,
                  BLIND_BLOCK);
    }
  }
  for (int i = 0; i < _config.curvatureRegion; ++i) {
    const PointIN &point = (_laserCloud[endIdx - i]);
    const PointIN &previousPoint = (_laserCloud[endIdx - i - 1]);
    if (calcCosAngleDiff(point, previousPoint) < _config.blindThreshold) {
      std::fill_n(
          &_scanNeighborPicked[endIdx - i - startIdx - _config.curvatureRegion],
          _config.curvatureRegion + 1, BLIND_BLOCK);
    }
  }

  // mark unreliable points as picked
  for (size_t i = startIdx + _config.curvatureRegion;
       i < endIdx - _config.curvatureRegion; i++) {
    const PointIN &previousPoint = (_laserCloud[i - 1]);
    const PointIN &point = (_laserCloud[i]);
    const PointIN &nextPoint = (_laserCloud[i + 1]);

    float diffNext = calcSquaredDiff(nextPoint, point);
    if (calcCosAngleDiff(point, nextPoint) < _config.blindThreshold) {
      std::fill_n(
          &_scanNeighborPicked[i - startIdx - _config.curvatureRegion + 1],
          _config.curvatureRegion * 2, BLIND_BLOCK);
      continue;
    }

    if (diffNext > 1.0) {
      float depth1 = calcPointDistance(point);
      float depth2 = calcPointDistance(nextPoint);
      float diffPrev = calcSquaredDiff(previousPoint, point);
      if (depth1 > depth2) {
        if (_scanNeighborPicked[i - startIdx + 1] > NEAR_BLOCK &&
            diffPrev / diffNext < 0.2)
          _scanNeighborPicked[i - startIdx + 1] = EDGE_BROKEN;
        std::fill_n(
            &_scanNeighborPicked[i - startIdx - _config.curvatureRegion + 1],
            _config.curvatureRegion, NEAR_BLOCK);

      } else {
        if (_scanNeighborPicked[i - startIdx] > NEAR_BLOCK &&
            diffPrev / diffNext < 0.2)
          _scanNeighborPicked[i - startIdx] = EDGE_BROKEN;
        std::fill_n(&_scanNeighborPicked[i - startIdx + 1],
                    _config.curvatureRegion, NEAR_BLOCK);
      }
    }
  }
}

void ScanRegistration::markAsPicked(const size_t &cloudIdx,
                                    const size_t &scanIdx, int label) {
  _scanNeighborPicked[scanIdx] = label;

  for (int i = 1; i <= _config.curvatureRegion; i++) {
    // if (calcSquaredDiff(_laserCloud[cloudIdx + i],
    //                    _laserCloud[cloudIdx + i - 1]) > 0.05) {
    //  break;
    //}

    _scanNeighborPicked[scanIdx + i] = label;
  }

  for (int i = 1; i <= _config.curvatureRegion; i++) {
    // if (calcSquaredDiff(_laserCloud[cloudIdx - i],
    //                    _laserCloud[cloudIdx - i + 1]) > 0.05) {
    //  break;
    //}

    _scanNeighborPicked[scanIdx - i] = label;
  }
}

int ScanRegistration::pointClassify(const size_t &cloudIdx) {

  Eigen::Matrix3f _lineMatA;
  Eigen::Matrix<float, 1, 3> _lineMatD;
  Eigen::Matrix3f _lineMatV;
  Eigen::Vector3f _lineCentroid;

  Eigen::Vector3f v1, v2;
  bool line1 = false;
  bool line2 = false;
  if (1) {
    // compute mean
    _lineCentroid.setZero();
    for (int j = 0; j <= _config.curvatureRegion; j++) {
      _lineCentroid += _laserCloud[cloudIdx - j].getVector3fMap();
    }
    _lineCentroid /= (_config.curvatureRegion + 1);

    // compute CovarianceMatrix
    _lineMatA.setZero();
    for (int j = 0; j <= _config.curvatureRegion; j++) {
      Eigen::Vector3f a =
          _laserCloud[cloudIdx - j].getVector3fMap() - _lineCentroid;
      _lineMatA(0, 0) += a(0) * a(0);
      _lineMatA(1, 0) += a(0) * a(1);
      _lineMatA(2, 0) += a(0) * a(2);
      _lineMatA(1, 1) += a(1) * a(1);
      _lineMatA(2, 1) += a(1) * a(2);
      _lineMatA(2, 2) += a(2) * a(2);
    }
    _lineMatA /= (_config.curvatureRegion + 1);

    // compute eigenvalues and eigenvectors
    _lineMatD.setZero();
    _lineMatV.setZero();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> esolver(_lineMatA);
    _lineMatD = esolver.eigenvalues().real();
    _lineMatV = esolver.eigenvectors().real();

    // compute corner line and coefficients
    if (_lineMatD(0, 2) > 100 * _lineMatD(0, 1) &&
        _lineMatD(0, 2) > 10000 * _lineMatD(0, 0)) {
      v1 = _lineMatV.col(2);
      line1 = true;
      for (int j = 0; j <= _config.curvatureRegion; j++) {
        Eigen::Vector3f a =
            _laserCloud[cloudIdx - j].getVector3fMap() - _lineCentroid;

        float distance = (a.cross(v1)).norm() / v1.norm();
        if (fabs(distance) > 0.08) {
          line1 = false;
          break;
        }
      }
    }
  }
  if (1) {
    _lineCentroid.setZero();
    for (int j = -1.0 * _config.curvatureRegion; j <= 0; j++) {
      _lineCentroid += _laserCloud[cloudIdx - j].getVector3fMap();
    }
    _lineCentroid /= (_config.curvatureRegion + 1);

    // compute CovarianceMatrix
    _lineMatA.setZero();
    for (int j = -1.0 * _config.curvatureRegion; j <= 0; j++) {
      Eigen::Vector3f a =
          _laserCloud[cloudIdx - j].getVector3fMap() - _lineCentroid;

      _lineMatA(0, 0) += a(0) * a(0);
      _lineMatA(1, 0) += a(0) * a(1);
      _lineMatA(2, 0) += a(0) * a(2);
      _lineMatA(1, 1) += a(1) * a(1);
      _lineMatA(2, 1) += a(1) * a(2);
      _lineMatA(2, 2) += a(2) * a(2);
    }
    _lineMatA /= (_config.curvatureRegion + 1);

    // compute eigenvalues and eigenvectors
    _lineMatD.setZero();
    _lineMatV.setZero();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> esolver(_lineMatA);
    _lineMatD = esolver.eigenvalues().real();
    _lineMatV = esolver.eigenvectors().real();

    // compute corner line and coefficients
    if (_lineMatD(0, 2) > 100 * _lineMatD(0, 1) &&
        _lineMatD(0, 2) > 10000 * _lineMatD(0, 0)) {

      v2 = _lineMatV.col(2);
      line2 = true;
      for (int j = -1.0 * _config.curvatureRegion; j <= 0; j++) {
        Eigen::Vector3f a =
            _laserCloud[cloudIdx - j].getVector3fMap() - _lineCentroid;

        float distance = (a.cross(v2)).norm() / v2.norm();
        if (fabs(distance) > 0.08) {
          line2 = false;
          break;
        }
      }
    }
  }

  if (line1 && line2) {
    float diff = calcCosAngleDiff(v1, v2);
    if (diff < cos(deg2rad(175.0)) || diff > cos(deg2rad(5.0))) {
      return SURFACE_FLAT;
    } else if (diff > cos(deg2rad(135.0)) && diff < cos(deg2rad(45.0))) {
      return CORNER_SHARP;
    }
  }
  if (line1 || line2) {
    return ONESIDE_FLAT;
  } else {
    return MESSY;
  }
  std::cout << "pointClassify error \n";
  return 0;
}

void ScanRegistration::publishResult() {
  // publish full resolution and feature point clouds
  publishCloudMsg(_pubLaserCloud, _laserCloud, _sweepStart, "/lidar");
  publishCloudMsg(_pubCornerPointsSharp, _cornerPointsSharp, _sweepStart,
                  "/lidar");
  publishCloudMsg(_pubCornerPointsLessSharp, _cornerPointsLessSharp,
                  _sweepStart, "/lidar");
  publishCloudMsg(_pubSurfPointsFlat, _surfacePointsFlat, _sweepStart,
                  "/lidar");
  publishCloudMsg(_pubSurfPointsLessFlat, _surfacePointsLessFlat, _sweepStart,
                  "/lidar");
  publishCloudMsg(_pubPointsBlind, _pointsBlind, _sweepStart, "/lidar");
  publishCloudMsg(_pubPointsBlock, _pointsBlock, _sweepStart, "/lidar");
  publishCloudMsg(_pubPointsSlop, _pointsSlop, _sweepStart, "/lidar");
  publishCloudMsg(_pubCurvature, _pointsCurvature, _sweepStart, "/lidar");

  // publish corresponding IMU transformation information
  _imuTrans[0].x = _imuStart.pitch.rad();
  _imuTrans[0].y = _imuStart.yaw.rad();
  _imuTrans[0].z = _imuStart.roll.rad();

  _imuTrans[1].x = _imuCur.pitch.rad();
  _imuTrans[1].y = _imuCur.yaw.rad();
  _imuTrans[1].z = _imuCur.roll.rad();

  Vector3 imuShiftFromStart = _imuPositionShift;
  rotateYXZ(imuShiftFromStart, -_imuStart.yaw, -_imuStart.pitch,
            -_imuStart.roll);

  _imuTrans[2].x = imuShiftFromStart.x();
  _imuTrans[2].y = imuShiftFromStart.y();
  _imuTrans[2].z = imuShiftFromStart.z();

  Vector3 imuVelocityFromStart = _imuCur.velocity - _imuStart.velocity;
  rotateYXZ(imuVelocityFromStart, -_imuStart.yaw, -_imuStart.pitch,
            -_imuStart.roll);

  _imuTrans[3].x = imuVelocityFromStart.x();
  _imuTrans[3].y = imuVelocityFromStart.y();
  _imuTrans[3].z = imuVelocityFromStart.z();

  publishCloudMsg(_pubImuTrans, _imuTrans, _sweepStart, "/lidar");
}

} // end namespace lidar_slam
