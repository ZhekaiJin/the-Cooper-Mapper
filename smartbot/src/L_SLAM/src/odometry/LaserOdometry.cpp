
#include "LaserOdometry.h"
#include "common/feature_utils.h"
#include "common/math_utils.h"
#include "common/ros_utils.h"

#include "common/transform_utils.h"
#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <pcl/common/eigen.h>
#include <pcl/filters/filter.h>

namespace lidar_slam {

using std::sin;
using std::cos;
using std::asin;
using std::atan2;
using std::sqrt;
using std::fabs;
using std::pow;

LaserOdometry::LaserOdometry()
    : _systemInited(false), _inputFrameCount(0), _maxIterations(25),
      _deltaTAbort(0.1), _deltaRAbort(0.1), _scale_rot_y(1.0),
      _scale_trans_z(1.05), _cornerPointsSharp(new CloudI()),
      _cornerPointsLessSharp(new CloudI()), _surfPointsFlat(new CloudI()),
      _surfPointsLessFlat(new CloudI()), _laserCloud(new CloudIN()),
      _lastCornerCloud(new CloudI()), _lastSurfaceCloud(new CloudI()),
      _laserCloudOri(new CloudI()), _coeffSel(new CloudI()) {
  cloudReceiveCount = 0;
  _Tsum = Eigen::Isometry3f::Identity();
}

bool LaserOdometry::setup(ros::NodeHandle &node, ros::NodeHandle &privateNode) {

  //imu_que.setup(node, privateNode);

  // fetch laser odometry params
  float fParam;
  int iParam;
  if (privateNode.getParam("scale_rot_y", fParam)) {
    _scale_rot_y = fParam;
    ROS_INFO("Set scale_rot_y: %f", fParam);
  }

  if (privateNode.getParam("scale_trans_z", fParam)) {
    _scale_trans_z = fParam;
    ROS_INFO("Set scale_trans_z: %f", fParam);
  }

  if (privateNode.getParam("maxIterations", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("Invalid maxIterations parameter: %d (expected > 0)", iParam);
      return false;
    } else {
      _maxIterations = iParam;
      ROS_INFO("Set maxIterations: %d", iParam);
    }
  }

  if (privateNode.getParam("deltaTAbort", fParam)) {
    if (fParam <= 0) {
      ROS_ERROR("Invalid deltaTAbort parameter: %f (expected > 0)", fParam);
      return false;
    } else {
      _deltaTAbort = fParam;
      ROS_INFO("Set deltaTAbort: %g", fParam);
    }
  }

  if (privateNode.getParam("deltaRAbort", fParam)) {
    if (fParam <= 0) {
      ROS_ERROR("Invalid deltaRAbort parameter: %f (expected > 0)", fParam);
      return false;
    } else {
      _deltaRAbort = fParam;
      ROS_INFO("Set deltaRAbort: %g", fParam);
    }
  }

  privateNode.param("sendRegisteredCloud", _sendRegisteredCloud, true);
  privateNode.param("receiveFullCloud", _receiveFullCloud, true);


  // initialize odometry and odometry tf messages
  _laserOdometryMsg.header.frame_id = "/lidar_odom_init";
  _laserOdometryMsg.child_frame_id = "/laser_odom";

  _laserOdometryTrans.frame_id_ = "/lidar_odom_init";
  _laserOdometryTrans.child_frame_id_ = "/laser_odom";

  // advertise laser odometry topics
  _pubLaserCloudCornerLast =
      node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 2);
  _pubLaserCloudSurfLast =
      node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 2);
  _pubLaserCloudFullRes =
      node.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 2);
  _pubLaserOdometry =
      node.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 5);

  // subscribe to scan registration topics
  _subCornerPointsSharp = node.subscribe<sensor_msgs::PointCloud2>(
      "/laser_cloud_sharp", 2, &LaserOdometry::laserCloudSharpHandler, this);

  _subCornerPointsLessSharp = node.subscribe<sensor_msgs::PointCloud2>(
      "/laser_cloud_less_sharp", 2, &LaserOdometry::laserCloudLessSharpHandler,
      this);

  _subSurfPointsFlat = node.subscribe<sensor_msgs::PointCloud2>(
      "/laser_cloud_flat", 2, &LaserOdometry::laserCloudFlatHandler, this);

  _subSurfPointsLessFlat = node.subscribe<sensor_msgs::PointCloud2>(
      "/laser_cloud_less_flat", 2, &LaserOdometry::laserCloudLessFlatHandler,
      this);
  
  if(_receiveFullCloud){
    _subLaserCloudFullRes = node.subscribe<sensor_msgs::PointCloud2>(
        "/velodyne_cloud_2", 2, &LaserOdometry::laserCloudFullResHandler, this);
    
  }
  
  spin_thread = std::thread(&LaserOdometry::spin, this);

  return true;
}

LaserOdometry::~LaserOdometry() {
  ROS_INFO("[LaserOdometry] _inputFrameCount:%ld", _inputFrameCount);
  ROS_INFO("[LaserOdometry] cloudReceiveCount:%ld", cloudReceiveCount);
}


void LaserOdometry::transformToStart(const PointI &pi, PointI &po) {
  float s = 10 * (pi.intensity - int(pi.intensity));
  Twist t = _transform * s;
  Eigen::Isometry3f it;
  convertTransform(t, it);
  po.getVector3fMap() = it * pi.getVector3fMap();
  po.intensity = pi.intensity;
}

void LaserOdometry::transformToStart(const PointIN &pi, PointIN &po) {

  float s = 10 * (pi.curvature - int(pi.curvature));
  Twist t = _transform * s;
  Eigen::Isometry3f it;

  convertTransform(t, it);
  po.getVector3fMap() = it * pi.getVector3fMap();
  po.intensity = pi.intensity;
  po.curvature = pi.curvature;
}

size_t LaserOdometry::transformToEnd(CloudI::Ptr &cloud) {
  size_t cloudSize = cloud->points.size();
  Eigen::Isometry3f it, it_inv;
  convertTransform(_transform, it);
  it_inv = it.inverse();
  for (size_t i = 0; i < cloudSize; i++) {
    PointI &point = cloud->points[i];
    transformToStart(point, point);
    point.getVector3fMap() = it_inv * point.getVector3fMap();
  }

  return cloudSize;
}

size_t LaserOdometry::transformToEnd(CloudIN::Ptr &cloud) {
  size_t cloudSize = cloud->points.size();
  Eigen::Isometry3f it, it_inv;
  convertTransform(_transform, it);
  it_inv = it.inverse();

  for (size_t i = 0; i < cloudSize; i++) {
    PointIN &point = cloud->points[i];
    transformToStart(point, point);
    point.getVector3fMap() = it_inv * point.getVector3fMap();

    // swap to lidar frame
    /*
    float temp_x = point.x;
    point.x = point.z;
    point.z = point.y;
    point.y = temp_x;*/
  }

  return cloudSize;
}

void LaserOdometry::laserCloudSharpHandler(
    const sensor_msgs::PointCloud2ConstPtr &cornerPointsSharpMsg) {
  _timeCornerPointsSharp = cornerPointsSharpMsg->header.stamp;

  _cornerPointsSharp->clear();
  pcl::fromROSMsg(*cornerPointsSharpMsg, *_cornerPointsSharp);

  _newCornerPointsSharp = true;
}

void LaserOdometry::laserCloudLessSharpHandler(
    const sensor_msgs::PointCloud2ConstPtr &cornerPointsLessSharpMsg) {
  _timeCornerPointsLessSharp = cornerPointsLessSharpMsg->header.stamp;

  _cornerPointsLessSharp->clear();
  pcl::fromROSMsg(*cornerPointsLessSharpMsg, *_cornerPointsLessSharp);
  _newCornerPointsLessSharp = true;
}

void LaserOdometry::laserCloudFlatHandler(
    const sensor_msgs::PointCloud2ConstPtr &surfPointsFlatMsg) {
  _timeSurfPointsFlat = surfPointsFlatMsg->header.stamp;

  _surfPointsFlat->clear();
  pcl::fromROSMsg(*surfPointsFlatMsg, *_surfPointsFlat);

  _newSurfPointsFlat = true;
}

void LaserOdometry::laserCloudLessFlatHandler(
    const sensor_msgs::PointCloud2ConstPtr &surfPointsLessFlatMsg) {
  _timeSurfPointsLessFlat = surfPointsLessFlatMsg->header.stamp;

  _surfPointsLessFlat->clear();
  pcl::fromROSMsg(*surfPointsLessFlatMsg, *_surfPointsLessFlat);
  _newSurfPointsLessFlat = true;
}

void LaserOdometry::laserCloudFullResHandler(
    const sensor_msgs::PointCloud2ConstPtr &laserCloudFullResMsg) {
  _timeLaserCloudFullRes = laserCloudFullResMsg->header.stamp;

  _laserCloud->clear();
  pcl::fromROSMsg(*laserCloudFullResMsg, *_laserCloud);
  _newLaserCloudFullRes = true;
  cloudReceiveCount++;
}

void LaserOdometry::spin() {
  ros::Rate rate(500);
  bool status = ros::ok();

  while (status) {
    ros::spinOnce();
    process();
    status = ros::ok();
    rate.sleep();
  }
}

void LaserOdometry::reset() {
  _newCornerPointsSharp = false;
  _newCornerPointsLessSharp = false;
  _newSurfPointsFlat = false;
  _newSurfPointsLessFlat = false;
  _newLaserCloudFullRes = false;
}

bool LaserOdometry::hasNewData() {

  if(_receiveFullCloud){
    return _newCornerPointsSharp && _newCornerPointsLessSharp &&
          _newSurfPointsFlat && _newSurfPointsLessFlat &&
          _newLaserCloudFullRes &&
          fabs((_timeCornerPointsSharp - _timeSurfPointsLessFlat).toSec()) <
              0.005 &&
          fabs((_timeCornerPointsLessSharp - _timeSurfPointsLessFlat).toSec()) <
              0.005 &&
          fabs((_timeSurfPointsFlat - _timeSurfPointsLessFlat).toSec()) <
              0.005 &&
          fabs((_timeLaserCloudFullRes - _timeSurfPointsLessFlat).toSec()) <
              0.005;
  }else{
    return _newCornerPointsSharp && _newCornerPointsLessSharp &&
          _newSurfPointsFlat && _newSurfPointsLessFlat &&
          fabs((_timeCornerPointsSharp - _timeSurfPointsLessFlat).toSec()) <
              0.005 &&
          fabs((_timeCornerPointsLessSharp - _timeSurfPointsLessFlat).toSec()) <
              0.005 &&
          fabs((_timeSurfPointsFlat - _timeSurfPointsLessFlat).toSec()) <
              0.005;

  }

}

void LaserOdometry::process() {
  if (!hasNewData()) {
    return;
  }

  reset();

  if (!_systemInited) {
    _cornerPointsLessSharp.swap(_lastCornerCloud);
    _surfPointsLessFlat.swap(_lastSurfaceCloud);

    _lastCornerKDTree.setInputCloud(_lastCornerCloud);
    _lastSurfaceKDTree.setInputCloud(_lastSurfaceCloud);
    _systemInited = true;
    return;
  }
  Eigen::Isometry3f pre_pos, cor_pos;
  Eigen::Vector3f velocity;
  //imu_que.predict(_timeSurfPointsLessFlat, pre_pos);
  scanMatch();
  transformUpdate();
  //imu_que.correct(_Tsum, cor_pos, velocity);

  transformToEnd(_cornerPointsLessSharp);
  transformToEnd(_surfPointsLessFlat);

  _cornerPointsLessSharp.swap(_lastCornerCloud);
  _surfPointsLessFlat.swap(_lastSurfaceCloud);

  size_t lastCornerCloudSize = _lastCornerCloud->points.size();
  size_t lastSurfaceCloudSize = _lastSurfaceCloud->points.size();

  if (lastCornerCloudSize > 10 && lastSurfaceCloudSize > 100) {
    _lastCornerKDTree.setInputCloud(_lastCornerCloud);
    _lastSurfaceKDTree.setInputCloud(_lastSurfaceCloud);
  }

  publishResult();
}

void LaserOdometry::scanMatch() {
  PointI coeff;
  bool isDegenerate = false;
  Eigen::Matrix<float, 6, 6> matP;

  _inputFrameCount++;
  size_t lastCornerCloudSize = _lastCornerCloud->points.size();
  size_t lastSurfaceCloudSize = _lastSurfaceCloud->points.size();

  if (lastCornerCloudSize > 10 && lastSurfaceCloudSize > 100) {
    std::vector<int> pointSearchInd(1);
    std::vector<float> pointSearchSqDis(1);

    size_t cornerPointsSharpNum = _cornerPointsSharp->points.size();
    size_t surfPointsFlatNum = _surfPointsFlat->points.size();

    _pointSearchCornerInd1.resize(cornerPointsSharpNum);
    _pointSearchCornerInd2.resize(cornerPointsSharpNum);
    _pointSearchSurfInd1.resize(surfPointsFlatNum);
    _pointSearchSurfInd2.resize(surfPointsFlatNum);
    _pointSearchSurfInd3.resize(surfPointsFlatNum);

    for (size_t iterCount = 0; iterCount < _maxIterations; iterCount++) {
      PointI pointSel, pointProj, tripod1, tripod2, tripod3;
      _laserCloudOri->clear();
      _coeffSel->clear();

      for (int i = 0; i < cornerPointsSharpNum; i++) {
        transformToStart(_cornerPointsSharp->points[i], pointSel);

        if (iterCount % 5 == 0) {

          _lastCornerKDTree.nearestKSearch(pointSel, 1, pointSearchInd,
                                           pointSearchSqDis);

          int closestPointInd = -1, minPointInd2 = -1;
          if (pointSearchSqDis[0] < 25) {
            closestPointInd = pointSearchInd[0];
            int closestPointScan =
                int(_lastCornerCloud->points[closestPointInd].intensity);

            float pointSqDis, minPointSqDis2 = 25;
            for (int j = closestPointInd + 1; j < cornerPointsSharpNum; j++) {
              if (int(_lastCornerCloud->points[j].intensity) >
                  closestPointScan + 2.5) {
                break;
              }

              pointSqDis =
                  calcSquaredDiff(_lastCornerCloud->points[j], pointSel);

              if (int(_lastCornerCloud->points[j].intensity) >
                  closestPointScan) {
                if (pointSqDis < minPointSqDis2) {
                  minPointSqDis2 = pointSqDis;
                  minPointInd2 = j;
                }
              }
            }
            for (int j = closestPointInd - 1; j >= 0; j--) {
              if (int(_lastCornerCloud->points[j].intensity) <
                  closestPointScan - 2.5) {
                break;
              }

              pointSqDis =
                  calcSquaredDiff(_lastCornerCloud->points[j], pointSel);

              if (int(_lastCornerCloud->points[j].intensity) <
                  closestPointScan) {
                if (pointSqDis < minPointSqDis2) {
                  minPointSqDis2 = pointSqDis;
                  minPointInd2 = j;
                }
              }
            }
          }

          _pointSearchCornerInd1[i] = closestPointInd;
          _pointSearchCornerInd2[i] = minPointInd2;
        }
        if (_pointSearchCornerInd2[i] >= 0) {
          const PointI &A = _lastCornerCloud->points[_pointSearchCornerInd1[i]];
          const PointI &B = _lastCornerCloud->points[_pointSearchCornerInd2[i]];
          PointI coefficients;
          if (getCornerFeatureCoefficients(A, B, pointSel, iterCount,
                                           coefficients)) {
            _laserCloudOri->push_back(_cornerPointsSharp->points[i]);
            _coeffSel->push_back(coefficients);
          }
        }
      }

      for (int i = 0; i < surfPointsFlatNum; i++) {
        transformToStart(_surfPointsFlat->points[i], pointSel);

        if (iterCount % 5 == 0) {
          _lastSurfaceKDTree.nearestKSearch(pointSel, 1, pointSearchInd,
                                            pointSearchSqDis);
          int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
          if (pointSearchSqDis[0] < 25) {
            closestPointInd = pointSearchInd[0];
            int closestPointScan =
                int(_lastSurfaceCloud->points[closestPointInd].intensity);

            float pointSqDis, minPointSqDis2 = 25, minPointSqDis3 = 25;
            for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++) {
              if (int(_lastSurfaceCloud->points[j].intensity) >
                  closestPointScan + 2.5) {
                break;
              }

              pointSqDis =
                  calcSquaredDiff(_lastSurfaceCloud->points[j], pointSel);

              if (int(_lastSurfaceCloud->points[j].intensity) <=
                  closestPointScan) {
                if (pointSqDis < minPointSqDis2) {
                  minPointSqDis2 = pointSqDis;
                  minPointInd2 = j;
                }
              } else {
                if (pointSqDis < minPointSqDis3) {
                  minPointSqDis3 = pointSqDis;
                  minPointInd3 = j;
                }
              }
            }
            for (int j = closestPointInd - 1; j >= 0; j--) {
              if (int(_lastSurfaceCloud->points[j].intensity) <
                  closestPointScan - 2.5) {
                break;
              }

              pointSqDis =
                  calcSquaredDiff(_lastSurfaceCloud->points[j], pointSel);

              if (int(_lastSurfaceCloud->points[j].intensity) >=
                  closestPointScan) {
                if (pointSqDis < minPointSqDis2) {
                  minPointSqDis2 = pointSqDis;
                  minPointInd2 = j;
                }
              } else {
                if (pointSqDis < minPointSqDis3) {
                  minPointSqDis3 = pointSqDis;
                  minPointInd3 = j;
                }
              }
            }
          }

          _pointSearchSurfInd1[i] = closestPointInd;
          _pointSearchSurfInd2[i] = minPointInd2;
          _pointSearchSurfInd3[i] = minPointInd3;
        }

        if (_pointSearchSurfInd2[i] >= 0 && _pointSearchSurfInd3[i] >= 0) {
          const PointI &A = _lastSurfaceCloud->points[_pointSearchSurfInd1[i]];
          const PointI &B = _lastSurfaceCloud->points[_pointSearchSurfInd2[i]];
          const PointI &C = _lastSurfaceCloud->points[_pointSearchSurfInd3[i]];

          PointI coefficients;
          if (getSurfaceFeatureCoefficients(A, B, C, pointSel, iterCount,
                                            coefficients)) {
            _laserCloudOri->push_back(_surfPointsFlat->points[i]);
            _coeffSel->push_back(coefficients);
          }
        }
      }

      int pointSelNum = _laserCloudOri->points.size();
      //cout << "iterCount,pointSelNum:" << iterCount << "," << pointSelNum << std::endl;
      if (pointSelNum < 10) {
        continue;
      }

      Eigen::Matrix<float, Eigen::Dynamic, 6> matA(pointSelNum, 6);
      Eigen::Matrix<float, 6, Eigen::Dynamic> matAt(6, pointSelNum);
      Eigen::Matrix<float, 6, 6> matAtA;
      Eigen::VectorXf matB(pointSelNum);
      Eigen::Matrix<float, 6, 1> matAtB;
      Eigen::Matrix<float, 6, 1> matX;

      float s = 1;
      /**/
      float srx = sin(s * _transform.rot_x.rad());
      float crx = cos(s * _transform.rot_x.rad());
      float sry = sin(s * _transform.rot_y.rad());
      float cry = cos(s * _transform.rot_y.rad());
      float srz = sin(s * _transform.rot_z.rad());
      float crz = cos(s * _transform.rot_z.rad());
      float tx = s * _transform.pos.x();
      float ty = s * _transform.pos.y();
      float tz = s * _transform.pos.z();

      for (int i = 0; i < pointSelNum; i++) {
        const PointI &pointOri = _laserCloudOri->points[i];
        coeff = _coeffSel->points[i];
        /*
        float arx = (crx * sry * srz * pointOri.x +
                     crx * crz * sry * pointOri.y - srx * sry * pointOri.z) *
                        coeff.x +
                    (-srx * srz * pointOri.x - crz * srx * pointOri.y -
                     crx * pointOri.z) *
                        coeff.y +
                    (crx * cry * srz * pointOri.x +
                     crx * cry * crz * pointOri.y - cry * srx * pointOri.z) *
                        coeff.z;

        float ary = ((cry * srx * srz - crz * sry) * pointOri.x +
                     (sry * srz + cry * crz * srx) * pointOri.y +
                     crx * cry * pointOri.z) *
                        coeff.x +
                    ((-cry * crz - srx * sry * srz) * pointOri.x +
                     (cry * srz - crz * srx * sry) * pointOri.y -
                     crx * sry * pointOri.z) *
                        coeff.z;

        float arz =
            ((crz * srx * sry - cry * srz) * pointOri.x +
             (-cry * crz - srx * sry * srz) * pointOri.y) *
                coeff.x +
            (crx * crz * pointOri.x - crx * srz * pointOri.y) * coeff.y +
            ((sry * srz + cry * crz * srx) * pointOri.x +
             (crz * sry - cry * srx * srz) * pointOri.y) *
                coeff.z;

        */
        float arx = ((crz*sry*crx + srz*srx)* pointOri.y +(srz*crx-crz*sry*srx)* pointOri.z)*coeff.x +
        ((srz*sry*crx-crz*srx)*pointOri.y -(srz*sry*srx+crz*crx)*pointOri.z)*coeff.y +
        (cry*crx*pointOri.y-cry*srx*pointOri.z)*coeff.z;

        float ary = (-crz*sry*pointOri.x+crz*cry*srx*pointOri.y+crz*cry*crx*pointOri.z)*coeff.x +
        (-srz*sry*pointOri.x+srz*cry*srx*pointOri.y +srz*cry*crx*pointOri.z)*coeff.y +
        (-cry*pointOri.x-sry*srx*pointOri.y-sry*crx*pointOri.z)*coeff.z;

        float arz = (-srz*cry*pointOri.x -(srz*sry*srx+crz*crx)*pointOri.y+(crz*srx-srz*sry*crx)*pointOri.z)*coeff.x+
        (crz*cry*pointOri.x+ (crz*sry*srx-srz*crx)*pointOri.y+crz*sry*crx+srz*srx*pointOri.z)*coeff.y+
        0*coeff.z;

        matA(i, 0) = arx;
        matA(i, 1) = ary;
        matA(i, 2) = arz;
        matA(i, 3) = coeff.x;
        matA(i, 4) = coeff.y;
        matA(i, 5) = coeff.z;
        matB(i, 0) = -0.05 * coeff.intensity;
      }
      matAt = matA.transpose();
      matAtA = matAt * matA;
      matAtB = matAt * matB;

      matX = matAtA.colPivHouseholderQr().solve(matAtB);

      if (iterCount == 0) {
        Eigen::Matrix<float, 1, 6> matE;
        Eigen::Matrix<float, 6, 6> matV;
        Eigen::Matrix<float, 6, 6> matV2;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 6, 6>> esolver(
            matAtA);
        matE = esolver.eigenvalues().real();
        matV = esolver.eigenvectors().real();

        matV2 = matV;

        isDegenerate = false;
        float eignThre[6] = {10, 10, 10, 10, 10, 10};
        for (int i = 0; i < 6; i++) {
          if (matE(0, i) < eignThre[i]) {
            for (int j = 0; j < 6; j++) {
              matV2(i, j) = 0;
            }
            isDegenerate = true;
          } else {
            break;
          }
        }
        matP = matV.inverse() * matV2;
      }

      if (isDegenerate) {
        Eigen::Matrix<float, 6, 1> matX2;
        matX2 = matX;
        matX = matP * matX2;
      }
      _transform.rot_x += matX(0, 0);
      _transform.rot_y += matX(1, 0);
      _transform.rot_z += matX(2, 0);
      _transform.pos.x() += matX(3, 0);
      _transform.pos.y() += matX(4, 0);
      _transform.pos.z() += matX(5, 0);

      if (!pcl_isfinite(_transform.rot_x.rad()))
        _transform.rot_x = Angle();
      if (!pcl_isfinite(_transform.rot_y.rad()))
        _transform.rot_y = Angle();
      if (!pcl_isfinite(_transform.rot_z.rad()))
        _transform.rot_z = Angle();

      if (!pcl_isfinite(_transform.pos.x()))
        _transform.pos.x() = 0.0;
      if (!pcl_isfinite(_transform.pos.y()))
        _transform.pos.y() = 0.0;
      if (!pcl_isfinite(_transform.pos.z()))
        _transform.pos.z() = 0.0;

      float deltaR =
          sqrt(pow(rad2deg(matX(0, 0)), 2) + pow(rad2deg(matX(1, 0)), 2) +
               pow(rad2deg(matX(2, 0)), 2));
      float deltaT = sqrt(pow(matX(3, 0) * 100, 2) + pow(matX(4, 0) * 100, 2) +
                          pow(matX(5, 0) * 100, 2));

      if (deltaR < _deltaRAbort && deltaT < _deltaTAbort) {
        break;
      }
    }
  }
}

void LaserOdometry::transformUpdate() {
  Eigen::Isometry3f update;
  convertTransform(_transform, update);
  _Tsum = _Tsum * update;
}

void LaserOdometry::publishResult() {

  Eigen::Quaternionf geoQuat(_Tsum.rotation());
  Eigen::Vector3f pos(_Tsum.translation());

  _laserOdometryMsg.header.stamp = _timeSurfPointsLessFlat;
  Isometry2Odom(_Tsum.cast<double>(), _laserOdometryMsg);
  Twist tw;
  convertTransform(_Tsum, tw);
  _laserOdometryMsg.twist.twist.angular.x = tw.rot_x.rad();
  _laserOdometryMsg.twist.twist.angular.y = tw.rot_y.rad();
  _laserOdometryMsg.twist.twist.angular.z = tw.rot_z.rad();

  _pubLaserOdometry.publish(_laserOdometryMsg);

  _laserOdometryTrans.stamp_ = _timeSurfPointsLessFlat;
  Isometry2TFtransform(_Tsum.cast<double>(), _laserOdometryTrans);
  _tfBroadcaster.sendTransform(_laserOdometryTrans);

  ros::Time sweepTime = _timeSurfPointsLessFlat;
  publishCloudMsg(_pubLaserCloudCornerLast, *_lastCornerCloud, sweepTime,
                  "/laser_odom");
  publishCloudMsg(_pubLaserCloudSurfLast, *_lastSurfaceCloud, sweepTime,
                  "/laser_odom");
  
  if (_receiveFullCloud && _sendRegisteredCloud) {
    transformToEnd(_laserCloud); // transform full resolution cloud to sweep end
                               // before sending it
    publishCloudMsg(_pubLaserCloudFullRes, *_laserCloud, sweepTime, "/laser_odom");
  
  }
}

} // end namespace lidar_slam
