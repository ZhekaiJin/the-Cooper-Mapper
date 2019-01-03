#include "ScanMatch.h"
#include "common/feature_utils.h"
#include "common/math_utils.h"
#include "common/nanoflann_pcl.h"
#include "common/ros_utils.h"
#include "common/transform_utils.h"

#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Eigenvalues>
#include <Eigen/QR>

namespace lidar_slam {

using std::sqrt;
using std::fabs;
using std::asin;
using std::atan2;
using std::pow;

ScanMatch::ScanMatch(const size_t maxIterations)
    : _maxIterations(maxIterations), _deltaTAbort(0.05), _deltaRAbort(0.05),
      _useScore(true), _match_count(0), _fail_match_count(0), _total_score(0),
      _score_threshold(800), _match_percentage_threshold(0.4),
      _referenceCornerCloudDS(new CloudI()),
      _referenceSurfCloudDS(new CloudI()), _CornerCloudDS(new CloudI()),
      _SurfCloudDS(new CloudI()) {

  _downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);
  _downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);

  _fineScore = false;
}

ScanMatch::~ScanMatch() {
  std::cout << "[ScanMatch]\n"
            << " ,match_count:" << _match_count
            << " ,fail_match_count:" << _fail_match_count
            << " ,averageScore:" << getAverageScore() << std::endl;
}

double ScanMatch::getScore(const CloudI &coeffCloud) {
  double score = 0;
  for (int i = 0; i < coeffCloud.points.size(); ++i) {
    const PointI &p = coeffCloud.points[i];
    score += std::exp(-fabs(p.intensity));
  }
  return score;
}

bool ScanMatch::scanMatchScan(const CloudIConPtr &referenceCornerCloud,
                              const CloudIConPtr &referenceSurfCloud,
                              const CloudIConPtr &CornerCloud,
                              const CloudIConPtr &SurfCloud,
                              Twist &transformf) {

  if (referenceCornerCloud->points.size() < 50 ||
      referenceSurfCloud->points.size() < 100) {
    std::cout << "reference cloud points too few." << std::endl;
    return false;
  }
  Twist transform = transformf;

  PointI pointSel, pointOri, pointProj, coeff;
  std::vector<int> pointSearchInd(5, 0);
  std::vector<float> pointSearchSqDis(5, 0);

    nanoflann::KdTreeFLANN<PointI> kdtreeCorner;
    nanoflann::KdTreeFLANN<PointI> kdtreeSurf;
      /*
  pcl::KdTreeFLANN<PointI> kdtreeCorner;
  pcl::KdTreeFLANN<PointI> kdtreeSurf;
  */

  kdtreeCorner.setInputCloud(referenceCornerCloud);
  kdtreeSurf.setInputCloud(referenceSurfCloud);

  bool converge = false;
  bool isDegenerate = false;
  Eigen::Matrix<float, 6, 6> matP;

  size_t CornerNum = CornerCloud->points.size();
  size_t SurfNum = SurfCloud->points.size();

  pcl::PointCloud<PointI> laserCloudOri;
  pcl::PointCloud<PointI> coeffSel;

  int line_match_count = 0;
  int plane_match_count = 0;
  size_t iterCount;
  for (iterCount = 0; iterCount < _maxIterations; iterCount++) {
    laserCloudOri.clear();
    coeffSel.clear();
    line_match_count = 0;
    plane_match_count = 0;

    for (int i = 0; i < CornerNum; i++) {
      pointOri = CornerCloud->points[i];
      pointAssociateToMap(transform, pointOri, pointSel);
      kdtreeCorner.nearestKSearch(pointSel, 5, pointSearchInd,
                                  pointSearchSqDis);
      if (pointSearchSqDis[4] < 5.0) {
        const Eigen::Vector3f &point = pointSel.getVector3fMap();
        Eigen::Vector3f lineA, lineB;
        if (findLine(*referenceCornerCloud, pointSearchInd, lineA, lineB)) {
          PointI coefficients;
          if (getCornerFeatureCoefficients(lineA, lineB, point, coefficients)) {
            laserCloudOri.push_back(pointOri);
            coeffSel.push_back(coefficients);
          }
          line_match_count++;
        }
      }
    }

    for (int i = 0; i < SurfNum; i++) {
      pointOri = SurfCloud->points[i];
      pointAssociateToMap(transform, pointOri, pointSel);
      kdtreeSurf.nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
      if (pointSearchSqDis[4] < 5.0) {
        Eigen::Vector4f planeCoef;
        if (findPlane(*referenceSurfCloud, pointSearchInd, 0.2, planeCoef)) {
          PointI coefficients;
          if (getSurfaceFeatureCoefficients(planeCoef, pointSel,
                                            coefficients)) {
            laserCloudOri.push_back(pointOri);
            coeffSel.push_back(coefficients);
          }
          plane_match_count++;
        }
      }
    }

    float srx = transform.rot_x.sin();
    float crx = transform.rot_x.cos();
    float sry = transform.rot_y.sin();
    float cry = transform.rot_y.cos();
    float srz = transform.rot_z.sin();
    float crz = transform.rot_z.cos();

    size_t laserCloudSelNum = laserCloudOri.points.size();
    if (laserCloudSelNum < 50) {
      ROS_WARN("matched cloud points too few. Matched/Input:  %d / %d", laserCloudSelNum, CornerNum+SurfNum);
      break;
    }

    Eigen::Matrix<float, Eigen::Dynamic, 6> matA(laserCloudSelNum, 6);
    Eigen::Matrix<float, 6, Eigen::Dynamic> matAt(6, laserCloudSelNum);
    Eigen::Matrix<float, 6, 6> matAtA;
    Eigen::VectorXf matB(laserCloudSelNum);
    Eigen::VectorXf matAtB;
    Eigen::VectorXf matX;

    for (int i = 0; i < laserCloudSelNum; i++) {
      pointOri = laserCloudOri.points[i];
      coeff = coeffSel.points[i];
      /*
      float arx = (crx * sry * srz * pointOri.x + crx * crz * sry * pointOri.y -
                   srx * sry * pointOri.z) *
                      coeff.x +
                  (-srx * srz * pointOri.x - crz * srx * pointOri.y -
                   crx * pointOri.z) *
                      coeff.y +
                  (crx * cry * srz * pointOri.x + crx * cry * crz * pointOri.y -
                   cry * srx * pointOri.z) *
                      coeff.z;

      float ary = ((cry * srx * srz - crz * sry) * pointOri.x +
                   (sry * srz + cry * crz * srx) * pointOri.y +
                   crx * cry * pointOri.z) *
                      coeff.x +
                  ((-cry * crz - srx * sry * srz) * pointOri.x +
                   (cry * srz - crz * srx * sry) * pointOri.y -
                   crx * sry * pointOri.z) *
                      coeff.z;

      float arz = ((crz * srx * sry - cry * srz) * pointOri.x +
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
      matB(i, 0) = -coeff.intensity;
    }

    matAt = matA.transpose();
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    matX = matAtA.colPivHouseholderQr().solve(matAtB);

    if (iterCount == 0) {
      Eigen::Matrix<float, 1, 6> matE;
      Eigen::Matrix<float, 6, 6> matV;
      Eigen::Matrix<float, 6, 6> matV2;

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 6, 6>> esolver(matAtA);
      matE = esolver.eigenvalues().real();
      matV = esolver.eigenvectors().real();

      matV2 = matV;

      isDegenerate = false;
      float eignThre[6] = {100, 100, 100, 100, 100, 100};
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
      Eigen::Matrix<float, 6, 1> matX2(matX);
      matX = matP * matX2;
    }

    transform.rot_x += matX(0, 0);
    transform.rot_y += matX(1, 0);
    transform.rot_z += matX(2, 0);
    transform.pos.x() += matX(3, 0);
    transform.pos.y() += matX(4, 0);
    transform.pos.z() += matX(5, 0);

    float deltaR =
        sqrt(pow(rad2deg(matX(0, 0)), 2) + pow(rad2deg(matX(1, 0)), 2) +
             pow(rad2deg(matX(2, 0)), 2));
    float deltaT = sqrt(pow(matX(3, 0) * 100, 2) + pow(matX(4, 0) * 100, 2) +
                        pow(matX(5, 0) * 100, 2));

    // std::cout << "iterator:" << iterCount << std::endl;
    // transform.print();
    if (deltaR < _deltaRAbort && deltaT < _deltaTAbort) {
      converge = true;
      break;
    }
  }

  if (converge && _useScore) {

    double score = getScore(coeffSel);

    double match_count = line_match_count + plane_match_count;
    float percent = match_count / (CornerNum + SurfNum);
    std::cout << "scan match score:" << score << ",per:" << percent
              << std::endl;

    if (_fineScore) {
      laserCloudOri.clear();
      coeffSel.clear();
      line_match_count = 0;
      plane_match_count = 0;
      for (int i = 0; i < CornerNum; i++) {
        pointOri = CornerCloud->points[i];
        pointAssociateToMap(transform, pointOri, pointSel);
        kdtreeCorner.nearestKSearch(pointSel, 5, pointSearchInd,
                                    pointSearchSqDis);
        if (pointSearchSqDis[0] < 0.02) {
          const Eigen::Vector3f &point = pointSel.getVector3fMap();
          Eigen::Vector3f lineA, lineB;
          if (findLine(*referenceCornerCloud, pointSearchInd, lineA, lineB)) {
            PointI coefficients;
            if (getCornerFeatureCoefficients(lineA, lineB, point,
                                             coefficients)) {
              laserCloudOri.push_back(pointOri);
              coeffSel.push_back(coefficients);
            }
            line_match_count++;
          }
        }
      }

      for (int i = 0; i < SurfNum; i++) {
        pointOri = SurfCloud->points[i];
        pointAssociateToMap(transform, pointOri, pointSel);
        kdtreeSurf.nearestKSearch(pointSel, 5, pointSearchInd,
                                  pointSearchSqDis);
        if (pointSearchSqDis[0] < 0.05) {
          Eigen::Vector4f planeCoef;
          if (findPlane(*referenceSurfCloud, pointSearchInd, 0.2, planeCoef)) {
            PointI coefficients;
            if (getSurfaceFeatureCoefficients(planeCoef, pointSel,
                                              coefficients)) {
              laserCloudOri.push_back(pointOri);
              coeffSel.push_back(coefficients);
            }
            plane_match_count++;
          }
        }
      }

      double score2 = getScore(coeffSel);
      match_count = line_match_count + plane_match_count;
      float percent2 = match_count / (CornerNum + SurfNum);
      std::cout << "scan match score2:" << score2 << " ,per2:" << percent2
                << std::endl;
    }

    if (score < _score_threshold) {
      transformf = transform;
      _fail_match_count++;
      std::cout << "low score!!" << score << ",per:" << percent << std::endl;
      return false;
    }

    if (percent < _match_percentage_threshold) {
      transformf = transform;
      _fail_match_count++;
      std::cout << "low percent!!" << score << ",per:" << percent << std::endl;
      return false;
    }
    _total_score += score;
    _match_count++;
    transformf = transform;

    return true;
  }
  //??? should we take the result when not converge?
  transformf = transform;
  _fail_match_count++;

  return false;
}

bool ScanMatch::scanMatchScan(const CloudIConPtr &referenceCornerCloud,
                              const CloudIConPtr &referenceSurfCloud,
                              const CloudIConPtr &CornerCloud,
                              const CloudIConPtr &SurfCloud,
                              Eigen::Isometry3f &relative_pose) {
  Twist transform;
  convertTransform(relative_pose, transform);
  bool success = scanMatchScan(referenceCornerCloud, referenceSurfCloud,
                               CornerCloud, SurfCloud, transform);
  convertTransform(transform, relative_pose);
  return success;
}

bool ScanMatch::scanMatchLocal(const CloudIConPtr &referenceCornerCloud,
                               const CloudIConPtr &referenceSurfCloud,
                               const CloudIConPtr &CornerCloud,
                               const CloudIConPtr &SurfCloud,
                               Eigen::Isometry3f &relative_pose) {
  Twist transform;
  convertTransform(relative_pose, transform);
  bool success = scanMatchLocal(referenceCornerCloud, referenceSurfCloud,
                                CornerCloud, SurfCloud, transform);
  convertTransform(transform, relative_pose);
  return success;
}

bool ScanMatch::scanMatchLocal(const CloudIConPtr &referenceCornerCloud,
                               const CloudIConPtr &referenceSurfCloud,
                               const CloudIConPtr &CornerCloud,
                               const CloudIConPtr &SurfCloud,
                               Twist &transform) {
  _referenceCornerCloudDS->clear();
  _downSizeFilterCorner.setInputCloud(referenceCornerCloud);
  _downSizeFilterCorner.filter(*_referenceCornerCloudDS);

  _referenceSurfCloudDS->clear();
  _downSizeFilterSurf.setInputCloud(referenceSurfCloud);
  _downSizeFilterSurf.filter(*_referenceSurfCloudDS);

  _CornerCloudDS->clear();
  _downSizeFilterCorner.setInputCloud(CornerCloud);
  _downSizeFilterCorner.filter(*_CornerCloudDS);

  _SurfCloudDS->clear();
  _downSizeFilterSurf.setInputCloud(SurfCloud);
  _downSizeFilterSurf.filter(*_SurfCloudDS);

  return scanMatchScan(_referenceCornerCloudDS, _referenceSurfCloudDS,
                       _CornerCloudDS, _SurfCloudDS, transform);
}

} // end namespace lidar_slam
