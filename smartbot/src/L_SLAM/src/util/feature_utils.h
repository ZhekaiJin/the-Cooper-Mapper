#ifndef LIDAR_FEATURE_UTILS_H
#define LIDAR_FEATURE_UTILS_H

#include "Angle.h"
#include "Vector3.h"
#include "pcl_util.h"

#include <cmath>

namespace lidar_slam {

/**
* Line is given by points AB.
* The result is the distance and the direction to closest point from the third
* point X.
*/
inline float getLinePointDistance(const Eigen::Vector3f &A,
                                  const Eigen::Vector3f &B,
                                  const Eigen::Vector3f &X,
                                  Eigen::Vector3f &unit_direction) {
  Eigen::Vector3f BXcrossAX = (X - B).cross(X - A);
  float BXcrossAXnorm = BXcrossAX.norm();
  float lengthAB = (A - B).norm();
  unit_direction = -BXcrossAX.cross(B - A) / (BXcrossAXnorm * lengthAB);
  return BXcrossAXnorm / lengthAB;
}

inline float getSurfacePointDistance(const Eigen::Vector3f &A,
                                     const Eigen::Vector3f &B,
                                     const Eigen::Vector3f &C,
                                     const Eigen::Vector3f &X,
                                     Eigen::Vector3f &surfNormal) {
  surfNormal = (B - A).cross(C - A);
  surfNormal.normalize();
  float distance = (X - A).dot(surfNormal);
  float cos = distance / surfNormal.norm() / (A - X).norm();
  if (cos < 0)
    surfNormal *= -1.0;
  return fabs(distance);
}

template <typename PointT>
inline bool getCornerFeatureCoefficients(const PointT &A, const PointT &B,
                                         const PointT &X, int iterration,
                                         PointT &coeff) {
  Eigen::Vector3f direction;
  float distance = getLinePointDistance(A.getVector3fMap(), B.getVector3fMap(),
                                        X.getVector3fMap(), direction);
  if (distance < 0) {
    std::cout << "error getCornerFeatureCoefficients " << std::endl;
  }

  float weight = 1.0;
  if (iterration >= 5) {
    weight = 1 - 1.8 * fabs(distance); // 1.8
  }

  coeff.getVector3fMap() = direction * weight;
  coeff.intensity = distance * weight;
  return (weight > 0.1 && distance != 0);
}

template <typename PointT>
inline bool
getCornerFeatureCoefficients(const Eigen::Vector3f &A, const Eigen::Vector3f &B,
                             const Eigen::Vector3f &X, PointT &coeff) {
  Eigen::Vector3f direction;
  float distance = getLinePointDistance(A, B, X, direction);

  float weight = 1 - 0.9f * fabs(distance);
  coeff.getVector3fMap() = direction * weight;
  coeff.intensity = distance * weight;

  return (weight > 0.1);
}

template <typename PointT>
inline bool getSurfaceFeatureCoefficients(const PointT &A, const PointT &B,
                                          const PointT &C, const PointT &X,
                                          int iterration,
                                          PointT &coefficients) {

  Eigen::Vector3f surfNormal;
  float distance = getSurfacePointDistance(
      A.getVector3fMap(), B.getVector3fMap(), C.getVector3fMap(),
      X.getVector3fMap(), surfNormal);

  float weight = 1;
  if (iterration >= 5) {
    weight = 1 - 1.8 * fabs(distance) / sqrt(X.getVector3fMap().norm()); // 1.8
  }
  coefficients.getVector3fMap() = weight * surfNormal;
  coefficients.intensity = weight * distance;
  return (weight > 0.1 && distance != 0);
}

template <typename PointT>
inline bool getSurfaceFeatureCoefficients(const Eigen::Vector4f &planeCoef,
                                          const PointT &X,
                                          PointT &coefficients) {
  float distance = planeCoef.head(3).dot(X.getVector3fMap()) + planeCoef(3);
  float weight = 1 - 0.9 * fabs(distance) / sqrt(X.getVector3fMap().norm());
  coefficients.getVector3fMap() = planeCoef.head(3) * weight;
  coefficients.intensity = distance * weight;
  return (weight > 0.1);
}

template <typename PointT>
bool findLine(const pcl::PointCloud<PointT> &cloud,
              const std::vector<int> &indices, Eigen::Vector3f &lineA,
              Eigen::Vector3f &lineB) {

  Eigen::Matrix3f _lineMatA;
  Eigen::Matrix<float, 1, 3> _lineMatD;
  Eigen::Matrix3f _lineMatV;
  Eigen::Vector3f _lineCentroid;

  // compute mean
  _lineCentroid.setZero();
  for (int j = 0; j < 5; j++) {
    _lineCentroid += cloud[indices[j]].getVector3fMap();
  }
  _lineCentroid /= 5.0;

  // compute CovarianceMatrix
  _lineMatA.setZero();
  for (int j = 0; j < 5; j++) {
    Eigen::Vector3f a = cloud[indices[j]].getVector3fMap() - _lineCentroid;
    _lineMatA(0, 0) += a(0) * a(0);
    _lineMatA(1, 0) += a(0) * a(1);
    _lineMatA(2, 0) += a(0) * a(2);
    _lineMatA(1, 1) += a(1) * a(1);
    _lineMatA(2, 1) += a(1) * a(2);
    _lineMatA(2, 2) += a(2) * a(2);
  }
  _lineMatA /= 5.0;

  // compute eigenvalues and eigenvectors
  //_lineMatD.setZero();
  //_lineMatV.setZero();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> esolver(_lineMatA);
  _lineMatD = esolver.eigenvalues().real();
  _lineMatV = esolver.eigenvectors().real();
  // compute corner line and coefficients
  if (_lineMatD(0, 2) > 5 * _lineMatD(0, 1)) {
    Eigen::Vector3f largestEigenVect = _lineMatV.col(2);
    lineA = _lineCentroid - largestEigenVect * 0.1;
    lineB = _lineCentroid + largestEigenVect * 0.1;

    return true;
  }

  return false;
}

// get AX+BY+CZ+D=0 with planeCoef(A,B,C,D)
template <typename PointT>
bool findPlane(const pcl::PointCloud<PointT> &cloud,
               const std::vector<int> &indices, float maxDistance,
               Eigen::Vector4f &planeCoef) {
  Eigen::Matrix<float, 5, 3> _planeMatA;
  Eigen::Matrix<float, 5, 1> _planeMatB;
  _planeMatB.setConstant(-1);

  Eigen::Vector3f _planeMatX;
  _planeMatA.setZero();
  _planeMatX.setZero();

  Eigen::Vector3f _planeCentroid;

  // compute mean
  _planeCentroid.setZero();

  for (int j = 0; j < 5; j++) {
    _planeCentroid += cloud[indices[j]].getVector3fMap();
    _planeMatA(j, 0) = cloud[indices[j]].x;
    _planeMatA(j, 1) = cloud[indices[j]].y;
    _planeMatA(j, 2) = cloud[indices[j]].z;
  }
  _planeCentroid /= 5.0;

  _planeMatX = _planeMatA.colPivHouseholderQr().solve(_planeMatB);

  planeCoef(0) = _planeMatX(0, 0);
  planeCoef(1) = _planeMatX(1, 0);
  planeCoef(2) = _planeMatX(2, 0);

  planeCoef(3) = 0;
  float norm = planeCoef.norm();
  planeCoef /= norm;
  planeCoef(3) = -planeCoef.head(3).dot(_planeCentroid);

  // check if any closet point far away with plane
  for (int j = 0; j < 5; j++) {
    float distance = planeCoef.head(3).dot(cloud[indices[j]].getVector3fMap()) +
                     planeCoef(3);
    // float distance = planeCoef.dot(cloud[indices[j]].getVector4fMap()) +
    // planeCoef(3);
    if (fabs(distance) > maxDistance) {
      return false;
    }
  }
  return true;
}

} // end namespace lidar_slam

#endif // LIDAR_FEATURE_UTILS_H
