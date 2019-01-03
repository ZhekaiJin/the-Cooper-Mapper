
#ifndef LIDAR_TRANSFORM_H
#define LIDAR_TRANSFORM_H

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

#include "Twist.h"
#include "math_utils.h"

namespace lidar_slam {

/*
描述从旧坐标系到新坐标系的变换更新顺序（从左到右记符号，与矩阵乘法顺序相同，先变换的矩阵在最左边，后更新的矩阵在右边）

*/
using namespace std;
template <typename Scalar>
inline void getRotationYaw(Scalar yaw, Eigen::Matrix<Scalar, 3, 3> &t) {
  t(0, 0) = cos(yaw);
  t(0, 1) = -sin(yaw);
  t(0, 2) = 0;
  t(1, 0) = sin(yaw);
  t(1, 1) = cos(yaw);
  t(1, 2) = 0;
  t(2, 0) = 0;
  t(2, 1) = 0;
  t(2, 2) = 1;
}

template <typename Scalar>
inline void getRotationPitch(Scalar pitch, Eigen::Matrix<Scalar, 3, 3> &t) {
  t(0, 0) = cos(pitch);
  t(0, 1) = 0;
  t(0, 2) = sin(pitch);
  t(1, 0) = 0;
  t(1, 1) = 1;
  t(1, 2) = 0;
  t(2, 0) = -sin(pitch);
  t(2, 1) = 0;
  t(2, 2) = cos(pitch);
}

template <typename Scalar>
inline void getRotationRoll(Scalar roll, Eigen::Matrix<Scalar, 3, 3> &t) {
  t(0, 0) = 1;
  t(0, 1) = 0;
  t(0, 2) = 0;
  t(1, 0) = 0;
  t(1, 1) = cos(roll);
  t(1, 2) = -sin(roll);
  t(2, 0) = 0;
  t(2, 1) = sin(roll);
  t(2, 2) = cos(roll);
}

template <typename Scalar>
void getEulerAngles(const Eigen::Matrix<Scalar, 3, 3> &t, Scalar &roll,
                    Scalar &pitch, Scalar &yaw) {
  roll = atan2(t(2, 1), t(2, 2));
  pitch = asin(-t(2, 0));
  yaw = atan2(t(1, 0), t(0, 0));
}

// getRotationYaw(yaw)*getRotationPitch(pitch)*getRotationRoll(roll)
template <typename Scalar>
inline void getTransformation(Scalar x, Scalar y, Scalar z, Scalar roll,
                         Scalar pitch, Scalar yaw,
                         Eigen::Transform<Scalar, 3, Eigen::Affine> &t) {
  Scalar A = cos(yaw), B = sin(yaw), C = cos(pitch), D = sin(pitch),
         E = cos(roll), F = sin(roll), DE = D * E, DF = D * F;

  t(0, 0) = A * C;
  t(0, 1) = A * DF - B * E;
  t(0, 2) = B * F + A * DE;
  t(0, 3) = x;
  t(1, 0) = B * C;
  t(1, 1) = A * E + B * DF;
  t(1, 2) = B * DE - A * F;
  t(1, 3) = y;
  t(2, 0) = -D;
  t(2, 1) = C * F;
  t(2, 2) = C * E;
  t(2, 3) = z;
  t(3, 0) = 0;
  t(3, 1) = 0;
  t(3, 2) = 0;
  t(3, 3) = 1;
}
//-------transform------------
inline Eigen::Isometry3f getTransformationTRyRxRz(float tx, float ty, float tz,
                                                float rx, float ry, float rz) {
  float srx = sin(rx);
  float crx = cos(rx);
  float sry = sin(ry);
  float cry = cos(ry);
  float srz = sin(rz);
  float crz = cos(rz);

  float crycrz = cry * crz;
  float crysrz = cry * srz;
  float srxsry = srx * sry;

  Eigen::Isometry3f t;
  t(0, 0) = crycrz + srxsry * srz;
  t(0, 1) = -crysrz+ srxsry*crz;
  t(0, 2) = sry*crx;
  t(0, 3) = tx;
  t(1, 0) = crx*srz;
  t(1, 1) = crx * crz;
  t(1, 2) = -srx;
  t(1, 3) = ty;
  t(2, 0) = -crz * sry+srx*crysrz;
  t(2, 1) = sry*srz+srx*crycrz;
  t(2, 2) = crx * cry;
  t(2, 3) = tz;
  t(3, 0) = 0;
  t(3, 1) = 0;
  t(3, 2) = 0;
  t(3, 3) = 1;

  return t;
}

template <typename Scalar>
void getEulerAnglesRyRxRz(const Eigen::Matrix<Scalar, 3, 3> &t, Scalar &roll,
                    Scalar &pitch, Scalar &yaw) {
  pitch = atan2(t(0, 2), t(2, 2));
  roll = asin(-t(1, 2));
  yaw = atan2(t(1, 0), t(1, 1));
}

inline Eigen::Affine3f getTransformationTRzRxRy(float tx, float ty, float tz,
                                                float rx, float ry, float rz) {
  float srx = sin(rx);
  float crx = cos(rx);
  float sry = sin(ry);
  float cry = cos(ry);
  float srz = sin(rz);
  float crz = cos(rz);

  float crycrz = cry * crz;
  float crysrz = cry * srz;
  float srxsry = srx * sry;

  Eigen::Affine3f t;
  t(0, 0) = crycrz - srxsry * srz;
  t(0, 1) = -crx * srz;
  t(0, 2) = crz * sry + crysrz * srx;
  t(0, 3) = tx;
  t(1, 0) = crysrz + crz * srxsry;
  t(1, 1) = crx * crz;
  t(1, 2) = sry * srz - crycrz * srx;
  t(1, 3) = ty;
  t(2, 0) = -crx * sry;
  t(2, 1) = srx;
  t(2, 2) = crx * cry;
  t(2, 3) = tz;
  t(3, 0) = 0;
  t(3, 1) = 0;
  t(3, 2) = 0;
  t(3, 3) = 1;

  return t;
}

inline Eigen::Affine3f getTransformationRyRxRzT(float tx, float ty, float tz,
                                                float rx, float ry, float rz) {
  float srx = sin(rx);
  float crx = cos(rx);
  float sry = sin(ry);
  float cry = cos(ry);
  float srz = sin(rz);
  float crz = cos(rz);

  float crycrz = cry * crz;
  float crycrzsrx = crycrz * srx;
  float srysrz = sry * srz;
  float srxsrysrz = srx * sry * srz;
  float srxsrycrz = srx * sry * crz;
  float crxsrz = crx * srz;
  float crysrz = cry * srz;
  float crxsry = crx * sry;
  float crxcrz = crx * crz;
  float crxcry = crx * cry;
  float crzsry = crz * sry;

  float crysrxsrz = cry * srx * srz;

  Eigen::Affine3f t;
  t(0, 0) = crycrz + srxsrysrz;
  t(0, 1) = srxsrycrz - crysrz;
  t(0, 2) = crxsry;
  t(0, 3) = tx * (crycrz + srxsrysrz) - ty * (crysrz - srxsrycrz) + crxsry * tz;
  t(1, 0) = crxsrz;
  t(1, 1) = crxcrz;
  t(1, 2) = -srx;
  t(1, 3) = crxcrz * ty - srx * tz + crxsrz * tx;
  t(2, 0) = crysrxsrz - crzsry;
  t(2, 1) = srysrz + crycrzsrx;
  t(2, 2) = crxcry;
  t(2, 3) = ty * (srysrz + crycrzsrx) - tx * (crzsry - crysrxsrz) + crxcry * tz;
  t(3, 0) = 0;
  t(3, 1) = 0;
  t(3, 2) = 0;
  t(3, 3) = 1;

  return t;
}


// use Eigen
inline Eigen::Isometry3f getTransformationYXZT(float tx, float ty, float tz,
                                               float roll, float pitch,
                                               float yaw) {
  // Roll pitch and yaw in Radians
  Eigen::Quaternionf q;
  q = Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) *
      Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
  Eigen::Isometry3f t = Eigen::Isometry3f::Identity();
  t.translate(Eigen::Vector3f(tx, ty, tz));
  t.prerotate(q);
  return t;
}
inline Eigen::Isometry3f getTransformationTZXY(float tx, float ty, float tz,
                                               float roll, float pitch,
                                               float yaw) {
  Eigen::Quaternionf q;
  q = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
      Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) *
      Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY());
  Eigen::Isometry3f t = Eigen::Isometry3f::Identity();
  t.pretranslate(Eigen::Vector3f(tx, ty, tz));
  t.rotate(q);
  return t;
}

inline Eigen::Isometry3f getTransformationTYXZ(float tx, float ty, float tz,
                                               float roll, float pitch,
                                               float yaw) {
  Eigen::Quaternionf q;
  q = Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) *
      Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
  Eigen::Isometry3f t = Eigen::Isometry3f::Identity();
  t.rotate(q);
  t.pretranslate(Eigen::Vector3f(tx, ty, tz));
  return t;
}

inline Eigen::Isometry3f getTransformationZXYT(float tx, float ty, float tz,
                                               float roll, float pitch,
                                               float yaw) {
  Eigen::Quaternionf q;
  q = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
      Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) *
      Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY());
  Eigen::Isometry3f t = Eigen::Isometry3f::Identity();
  t.translate(Eigen::Vector3f(tx, ty, tz));
  t.prerotate(q);
  return t;
}

inline Eigen::Isometry3f getTransformationTXYZ(float tx, float ty, float tz,
                                               float roll, float pitch,
                                               float yaw) {
  Eigen::Quaternionf q;
  q = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) *
      Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
  Eigen::Isometry3f t = Eigen::Isometry3f::Identity();
  t.rotate(q);
  t.pretranslate(Eigen::Vector3f(tx, ty, tz));
  return t;
}

inline Eigen::Isometry3f getTransformationXYZT(float tx, float ty, float tz,
                                               float roll, float pitch,
                                               float yaw) {
  Eigen::Quaternionf q;
  q = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) *
      Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
  Eigen::Isometry3f t = Eigen::Isometry3f::Identity();
  t.prerotate(q);
  t.translate(Eigen::Vector3f(tx, ty, tz));
  return t;
}

inline Eigen::Isometry3f getTransformationTZYX(float tx, float ty, float tz,
                                               float roll, float pitch,
                                               float yaw) {
  Eigen::Quaternionf q;
  q = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
      Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());
  Eigen::Isometry3f t = Eigen::Isometry3f::Identity();
  t.rotate(q);
  t.pretranslate(Eigen::Vector3f(tx, ty, tz));
  return t;
}

// use PCL
inline Eigen::Affine3f getTransformation(float tx, float ty, float tz, float rx,
                                         float ry, float rz) {
  Eigen::Affine3f t;
  t = pcl::getTransformation(tx, ty, tz, rx, ry, rz);
  return t;
}
inline void convertTransform(Twist &tt, Eigen::Isometry3f &ti) {
  ti = getTransformationTZYX(tt.pos.x(), tt.pos.y(), tt.pos.z(), tt.rot_x.rad(),
                             tt.rot_y.rad(), tt.rot_z.rad());
}

inline void convertTransform(Eigen::Isometry3f &ti, Twist &tt) {
  Eigen::Vector3f t = ti.translation();
  tt.pos.x() = t(0);
  tt.pos.y() = t(1);
  tt.pos.z() = t(2);
  Eigen::Matrix3f r = ti.rotation();
  Eigen::Vector3f angle;
  getEulerAngles(r, angle[0], angle[1], angle[2]);
  tt.rot_x = angle[0];
  tt.rot_y = angle[1];
  tt.rot_z = angle[2];
/*
  Eigen::Vector3f angle = r.eulerAngles(1, 0, 2);
  tt.rot_x = angle[1];
  tt.rot_y = angle[0];
  tt.rot_z = angle[2];
  */

}

inline void convertTransform2(Twist &tt, Eigen::Isometry3f &ti) {
  ti = getTransformationTYXZ(tt.pos.x(), tt.pos.y(), tt.pos.z(), tt.rot_x.rad(),
                             tt.rot_y.rad(), tt.rot_z.rad());
}

inline void convertTransform2(Eigen::Isometry3f &ti, Twist &tt) {
  Eigen::Vector3f t = ti.translation();
  tt.pos.x() = t(0);
  tt.pos.y() = t(1);
  tt.pos.z() = t(2);
  Eigen::Matrix3f r = ti.rotation();
  Eigen::Vector3f angle;
  getEulerAnglesRyRxRz(r, angle[0], angle[1], angle[2]);
    tt.rot_x = angle[0];
  tt.rot_y = angle[1];
  tt.rot_z = angle[2];
/*
  Eigen::Vector3f angle = r.eulerAngles(1, 0, 2);
  tt.rot_x = angle[1];
  tt.rot_y = angle[0];
  tt.rot_z = angle[2];
  */

}

inline void transformToStart(Twist _transform, const pcl::PointXYZI &pi,
                             pcl::PointXYZI &po) {
  po = pi;
  float s = 10 * (pi.intensity - int(pi.intensity));

  po.x -= s * _transform.pos.x();
  po.y -= s * _transform.pos.y();
  po.z -= s * _transform.pos.z();
  po.intensity = pi.intensity;

  Angle rx = -s * _transform.rot_x.rad();
  Angle ry = -s * _transform.rot_y.rad();
  Angle rz = -s * _transform.rot_z.rad();
  rotateZXY(po, rz, rx, ry);
}

inline void transformToStart2(Twist _transform, const pcl::PointXYZI &pi,
                              pcl::PointXYZI &po) {
  float s = 10 * (pi.intensity - int(pi.intensity));
  Twist t = _transform * s;

  po.getVector3fMap() =
      getTransformationTZXY(t.pos.x(), t.pos.y(), t.pos.z(), t.rot_x.rad(),
                            t.rot_y.rad(), t.rot_z.rad())
          .inverse() *
      pi.getVector3fMap();
  po.intensity = pi.intensity;
}

inline void transformToStart3(Twist _transform, const pcl::PointXYZI &pi,
                              pcl::PointXYZI &po) {
  float s = 10 * (pi.intensity - int(pi.intensity));
  Twist t = _transform * s;
  Eigen::Isometry3f it;
  convertTransform(t, it);
  po.getVector3fMap() = it * pi.getVector3fMap();
  po.intensity = pi.intensity;
}

inline void transformToStart(Eigen::Isometry3f _transform,
                             const pcl::PointXYZI &pi, pcl::PointXYZI &po) {
  po.getVector3fMap() = _transform * pi.getVector3fMap();
  po.intensity = pi.intensity;
}

inline void transformToEnd(Twist _transform, const pcl::PointXYZI &pi,
                           pcl::PointXYZI &po) {
  po = pi;
  float s = 10 * (po.intensity - int(po.intensity));

  po.x -= s * _transform.pos.x();
  po.y -= s * _transform.pos.y();
  po.z -= s * _transform.pos.z();

  Angle rx = -s * _transform.rot_x.rad();
  Angle ry = -s * _transform.rot_y.rad();
  Angle rz = -s * _transform.rot_z.rad();
  rotateZXY(po, rz, rx, ry);
  //---------------------//

  rotateYXZ(po, _transform.rot_y, _transform.rot_x, _transform.rot_z);
  po.x += _transform.pos.x();
  po.y += _transform.pos.y();
  po.z += _transform.pos.z();
  po.intensity = pi.intensity;
}

inline void transformToEnd2(Twist _tf, const pcl::PointXYZI &pi,
                            pcl::PointXYZI &po) {
  float s = 10 * (pi.intensity - int(pi.intensity));
  Twist t = _tf * s;

  po.getVector3fMap() =
      getTransformationTZXY(_tf.pos.x(), _tf.pos.y(), _tf.pos.z(),
                            _tf.rot_x.rad(), _tf.rot_y.rad(), _tf.rot_z.rad()) *
      getTransformationTZXY(t.pos.x(), t.pos.y(), t.pos.z(), t.rot_x.rad(),
                            t.rot_y.rad(), t.rot_z.rad())
          .inverse() *
      pi.getVector3fMap();
  po.intensity = pi.intensity;
}

inline void transformToEnd3(Twist _tf, const pcl::PointXYZI &pi,
                            pcl::PointXYZI &po) {
  float s = 10 * (pi.intensity - int(pi.intensity));
  Twist t = _tf * s;
  Eigen::Isometry3f it, it2;
  convertTransform(t, it);
  po.getVector3fMap() = it * pi.getVector3fMap();
  convertTransform(_tf, it2);
  po.getVector3fMap() = it2.inverse() * po.getVector3fMap();

  po.intensity = pi.intensity;
}

// warnning, not use this function
template <typename PointT>
inline void transformCloud(Eigen::Isometry3f trans,
                           const pcl::PointCloud<PointT> &cloud_in,
                           pcl::PointCloud<PointT> &cloud_out) {
  const Eigen::Affine3f transform = trans;
  pcl::transformPointCloud(cloud_in, cloud_out, transform);
}

inline void pointAssociateToMap2(Twist _tf, const pcl::PointXYZI &pi,
                                 pcl::PointXYZI &po) {
  po.x = pi.x;
  po.y = pi.y;
  po.z = pi.z;
  po.intensity = pi.intensity;

  rotateZXY(po, _tf.rot_z, _tf.rot_x, _tf.rot_y);

  po.x += _tf.pos.x();
  po.y += _tf.pos.y();
  po.z += _tf.pos.z();
}

inline void pointAssociateToMap(Twist _tf, const pcl::PointXYZI &pi,
                                pcl::PointXYZI &po) {
  Eigen::Isometry3f it;
  convertTransform(_tf, it);
  po.getVector3fMap() = it * pi.getVector3fMap();
  po.intensity = pi.intensity;
}

inline void pointAssociateTobeMapped2(Twist _tf, const pcl::PointXYZI &pi,
                                      pcl::PointXYZI &po) {
  po.x = pi.x - _tf.pos.x();
  po.y = pi.y - _tf.pos.y();
  po.z = pi.z - _tf.pos.z();
  po.intensity = pi.intensity;

  rotateYXZ(po, -_tf.rot_y, -_tf.rot_x, -_tf.rot_z);
}

inline void pointAssociateTobeMapped(Twist _tf, const pcl::PointXYZI &pi,
                                     pcl::PointXYZI &po) {
  Eigen::Isometry3f it;
  convertTransform(_tf, it);
  po.getVector3fMap() = it.inverse() * pi.getVector3fMap();
  po.intensity = pi.intensity;
}

inline void transformAssociate(Eigen::Isometry3f &Lold, Eigen::Isometry3f &Lnew,
                               Eigen::Isometry3f &Wold,
                               Eigen::Isometry3f &Wnew) {
  Eigen::Isometry3f L2W = Wold * Lold.inverse();
  Wnew = L2W * Lnew;
}

inline void transformAssociate(Twist &Lold, Twist &Lnew, Twist &Wold,
                               Twist &Wnew) {
  Twist _transformIncre;
  _transformIncre.pos = Lold.pos - Lnew.pos;
  rotateYXZ(_transformIncre.pos, -(Lnew.rot_y), -(Lnew.rot_x), -(Lnew.rot_z));

  float sbcx = Lnew.rot_x.sin();
  float cbcx = Lnew.rot_x.cos();
  float sbcy = Lnew.rot_y.sin();
  float cbcy = Lnew.rot_y.cos();
  float sbcz = Lnew.rot_z.sin();
  float cbcz = Lnew.rot_z.cos();

  float sblx = Lold.rot_x.sin();
  float cblx = Lold.rot_x.cos();
  float sbly = Lold.rot_y.sin();
  float cbly = Lold.rot_y.cos();
  float sblz = Lold.rot_z.sin();
  float cblz = Lold.rot_z.cos();

  float salx = Wold.rot_x.sin();
  float calx = Wold.rot_x.cos();
  float saly = Wold.rot_y.sin();
  float caly = Wold.rot_y.cos();
  float salz = Wold.rot_z.sin();
  float calz = Wold.rot_z.cos();

  float srx = -sbcx * (salx * sblx + calx * cblx * salz * sblz +
                       calx * calz * cblx * cblz) -
              cbcx * sbcy * (calx * calz * (cbly * sblz - cblz * sblx * sbly) -
                             calx * salz * (cbly * cblz + sblx * sbly * sblz) +
                             cblx * salx * sbly) -
              cbcx * cbcy * (calx * salz * (cblz * sbly - cbly * sblx * sblz) -
                             calx * calz * (sbly * sblz + cbly * cblz * sblx) +
                             cblx * cbly * salx);
  Wnew.rot_x = -asin(srx);

  float srycrx = sbcx * (cblx * cblz * (caly * salz - calz * salx * saly) -
                         cblx * sblz * (caly * calz + salx * saly * salz) +
                         calx * saly * sblx) -
                 cbcx * cbcy * ((caly * calz + salx * saly * salz) *
                                    (cblz * sbly - cbly * sblx * sblz) +
                                (caly * salz - calz * salx * saly) *
                                    (sbly * sblz + cbly * cblz * sblx) -
                                calx * cblx * cbly * saly) +
                 cbcx * sbcy * ((caly * calz + salx * saly * salz) *
                                    (cbly * cblz + sblx * sbly * sblz) +
                                (caly * salz - calz * salx * saly) *
                                    (cbly * sblz - cblz * sblx * sbly) +
                                calx * cblx * saly * sbly);
  float crycrx = sbcx * (cblx * sblz * (calz * saly - caly * salx * salz) -
                         cblx * cblz * (saly * salz + caly * calz * salx) +
                         calx * caly * sblx) +
                 cbcx * cbcy * ((saly * salz + caly * calz * salx) *
                                    (sbly * sblz + cbly * cblz * sblx) +
                                (calz * saly - caly * salx * salz) *
                                    (cblz * sbly - cbly * sblx * sblz) +
                                calx * caly * cblx * cbly) -
                 cbcx * sbcy * ((saly * salz + caly * calz * salx) *
                                    (cbly * sblz - cblz * sblx * sbly) +
                                (calz * saly - caly * salx * salz) *
                                    (cbly * cblz + sblx * sbly * sblz) -
                                calx * caly * cblx * sbly);
  Wnew.rot_y = atan2(srycrx / Wnew.rot_x.cos(), crycrx / Wnew.rot_x.cos());

  float srzcrx = (cbcz * sbcy - cbcy * sbcx * sbcz) *
                     (calx * salz * (cblz * sbly - cbly * sblx * sblz) -
                      calx * calz * (sbly * sblz + cbly * cblz * sblx) +
                      cblx * cbly * salx) -
                 (cbcy * cbcz + sbcx * sbcy * sbcz) *
                     (calx * calz * (cbly * sblz - cblz * sblx * sbly) -
                      calx * salz * (cbly * cblz + sblx * sbly * sblz) +
                      cblx * salx * sbly) +
                 cbcx * sbcz * (salx * sblx + calx * cblx * salz * sblz +
                                calx * calz * cblx * cblz);
  float crzcrx = (cbcy * sbcz - cbcz * sbcx * sbcy) *
                     (calx * calz * (cbly * sblz - cblz * sblx * sbly) -
                      calx * salz * (cbly * cblz + sblx * sbly * sblz) +
                      cblx * salx * sbly) -
                 (sbcy * sbcz + cbcy * cbcz * sbcx) *
                     (calx * salz * (cblz * sbly - cbly * sblx * sblz) -
                      calx * calz * (sbly * sblz + cbly * cblz * sblx) +
                      cblx * cbly * salx) +
                 cbcx * cbcz * (salx * sblx + calx * cblx * salz * sblz +
                                calx * calz * cblx * cblz);
  Wnew.rot_z = atan2(srzcrx / Wnew.rot_x.cos(), crzcrx / Wnew.rot_x.cos());

  Vector3 v = _transformIncre.pos;
  rotateZXY(v, Wnew.rot_z, Wnew.rot_x, Wnew.rot_y);
  Wnew.pos = Wold.pos - v;
}

template <typename PointT>
void transformPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                         pcl::PointCloud<PointT> &cloud_out,
                         const Eigen::Isometry3f &tf) {
  if (&cloud_in != &cloud_out) {
    cloud_out = cloud_in;
  }
  if (cloud_in.is_dense) {
    for (int i = 0; i < cloud_in.points.size(); i++) {
      cloud_out.points[i].getVector3fMap() =
          tf * cloud_in.points[i].getVector3fMap();
    }
  }
}

template <typename PointT>
void transformPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                         pcl::PointCloud<PointT> &cloud_out,
                         const Eigen::Isometry3d &tfd) {
  Eigen::Isometry3f tf;
  tf.matrix() = tfd.matrix().cast<float>();
  if (&cloud_in != &cloud_out) {
    cloud_out = cloud_in;
  }
  if (cloud_in.is_dense) {
    for (int i = 0; i < cloud_in.points.size(); i++) {
      cloud_out.points[i].getVector3fMap() =
          tf * cloud_in.points[i].getVector3fMap();
    }
  }
}

} // end namespace lidar_slam

#endif // LIDAR_TRANSFORM_H