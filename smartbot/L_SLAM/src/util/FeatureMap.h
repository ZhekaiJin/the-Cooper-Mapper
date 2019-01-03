#ifndef __FEATURE_MAP_H__
#define __FEATURE_MAP_H__

#include <Eigen/Eigenvalues>
#include <Eigen/QR>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>


#include <fstream>
#include <sstream>
#include <string>

#include "Twist.h"
#include "math_utils.h"
#include "transform_utils.h"
#include "feature_utils.h"
#include "ros_utils.h"
#include "nanoflann_pcl.h"

namespace lidar_slam {

template <typename PointT> class PointCloudCube {
public:
  typedef typename pcl::PointCloud<PointT> PointCloud;
  typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;

  PointCloudCube(int cubeNum) : _cube(cubeNum) {
    for (int i = 0; i < cubeNum; i++) {
      _cube[i].reset(new PointCloud);
    }
  }

  PointCloudPtr &operator[](int index) { return _cube[index]; }

  const PointCloudPtr &operator[](int index) const { return _cube[index]; }

private:
  std::vector<PointCloudPtr> _cube;
};

//---------- FeatureMap ---------------------

template <typename PointT> class FeatureMap {
public:
  typedef typename pcl::PointCloud<PointT> PointCloud;
  typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
  typedef typename pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;
  typedef typename pcl::VoxelGrid<PointT> VoxelGrid;

  FeatureMap(int cubeWidth_ = 21, int cubeHeight_ = 11, int cubeDepth_ = 21)
      : _cubeWidth(cubeWidth_), _cubeHeight(cubeHeight_),
        _cubeDepth(cubeDepth_), _cubeNum(_cubeWidth * _cubeHeight * _cubeDepth),
        _cornerCube(_cubeNum), _surfCube(_cubeNum),
        _cubeOriginWidth(std::round(--cubeWidth_ / 2.0)),
        _cubeOriginHeight(std::round(--cubeHeight_ / 2.0)),
        _cubeOriginDepth(std::round(--cubeDepth_ / 2.0)), _worldCubeSize(50.0),
        _lidarValidDistance(150.0), _cloudCornerSwap(new PointCloud()),
        _cloudSurfSwap(new PointCloud()), _filesDirectory("~") {
    _downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);
    _downSizeFilterSurf.setLeafSize(0.2, 0.2, 0.2);
    _downSizeFilterMap.setLeafSize(0.6, 0.6, 0.6);
    _kdtreeCorner.resize(_cubeNum);
    _kdtreeSurf.resize(_cubeNum);

  }
  ~FeatureMap() {}

  inline void setupFilterSize(float corner, float surf, float map) {
    _downSizeFilterCorner.setLeafSize(corner, corner, corner);
    _downSizeFilterSurf.setLeafSize(surf, surf, surf);
    _downSizeFilterMap.setLeafSize(map, map, map);
  }

  inline void setupWorldOrigin(float cubeOriginWidth, float cubeOriginHeight,
                               float cubeOriginDepth) {
    _cubeOriginWidth = cubeOriginWidth;
    _cubeOriginHeight = cubeOriginHeight;
    _cubeOriginDepth = cubeOriginDepth;
  }

  inline void setupWorldCubeSize(float worldCubeSize) {
    _worldCubeSize = worldCubeSize;
  }

  inline void setupLidarValidDistance(float lidarValidDistance) {
    _lidarValidDistance = lidarValidDistance;
  }

  inline void setupFilesDirectory(const std::string &filesDirectory) {
    _filesDirectory = filesDirectory;
  }

  inline bool isIndexValid(int i, int j, int k) {
    if (0 <= i && i < _cubeWidth && 0 <= j && j < _cubeHeight && 0 <= k &&
        k < _cubeDepth)
      return true;
    else
      return false;
  }

  void addFeatureCloud(const PointCloud &cornerCloud,
                       const PointCloud &surfCloud,
                       const Eigen::Isometry3f &tf);
  void addCornerCloud(const PointCloud &cloud);
  void addSurfCloud(const PointCloud &cloud);

  void pushCornerPoint(const PointT &point);
  void pushSurfPoint(const PointT &point);

  void update(const PointT &sensorPose);
  void getSurroundFeature(PointCloud &surroundCorner, PointCloud &surroundSurf);
  void downsizeValidCloud();
  bool getFullMap(PointCloudPtr &mapCloud);

  void computeActiveAera(const PointT sensorPose);
  bool saveCloudToFiles();
  bool loadCloudFromFiles();

  inline bool scanMatchScan(const PointCloudConstPtr &CornerCloud,
                              const PointCloudConstPtr &SurfCloud,
                              Twist &transformf);
  inline bool scanMatchScan(const PointCloudConstPtr &CornerCloud,
                              const PointCloudConstPtr &SurfCloud,
                              Eigen::Isometry3f &relative_pose);

  inline std::string fileNameFormat(const std::string filesDirectory,
                                    int number) {
    std::stringstream ss;
    std::string str;
    ss << number;
    ss >> str;
    str += ".pcd";
    return filesDirectory + '/' + str;
  }

private:
  inline int toIndex(int i, int j, int k) {
    return i + j * _cubeWidth + k * _cubeWidth * _cubeHeight;
  }

  void shift(int dIndexI, int dIndexJ, int dIndexK);
  int worldToIndex(float world_x, float world_y, float world_z);
  int worldToCube(float world_x, float world_y, float world_z, int &gridI,
                  int &gridJ, int &gridK);

private:
  int _cubeWidth, _cubeHeight, _cubeDepth; // the size of cubic gird

  int _cubeOriginWidth, _cubeOriginHeight, _cubeOriginDepth;
  int _curCubeWidth, _curCubeHeight, _curCubeDepth;
  double _curCubePosX, _curCubePosY, _curCubePosZ; // the center pos of the cube

  int _cubeNum;         // cube块总个数
  float _worldCubeSize; // cube块的尺度，立方体 m*m*m
  float _lidarValidDistance;
  std::string _filesDirectory; // cube块保存、加载路径

  PointCloudCube<PointT> _cornerCube;
  PointCloudCube<PointT> _surfCube;
  PointCloudPtr _cloudCornerSwap;
  PointCloudPtr _cloudSurfSwap;

  std::vector<size_t> _cubeValidInd;
  std::vector<size_t> _cubeUpdated;

  std::vector< nanoflann::KdTreeFLANN<PointT> > _kdtreeCorner;
  std::vector< nanoflann::KdTreeFLANN<PointT> > _kdtreeSurf;

  VoxelGrid
      _downSizeFilterCorner; ///< voxel filter for down sizing corner clouds
  VoxelGrid
      _downSizeFilterSurf; ///< voxel filter for down sizing surface clouds

  VoxelGrid
      _downSizeFilterMap; ///< voxel filter for down sizing full map clouds
};

//---------- Definitions ---------------------

template <typename PointT>
inline void FeatureMap<PointT>::pushCornerPoint(const PointT &point) {
  int gridI, gridJ, gridK;
  worldToCube(point.x, point.y, point.z, gridI, gridJ, gridK);
  if (isIndexValid(gridI, gridJ, gridK)) {
    _cornerCube[toIndex(gridI, gridJ, gridK)]->push_back(point);
  }
}
template <typename PointT>
inline void FeatureMap<PointT>::pushSurfPoint(const PointT &point) {
  int gridI, gridJ, gridK;
  worldToCube(point.x, point.y, point.z, gridI, gridJ, gridK);
  if (isIndexValid(gridI, gridJ, gridK)) {
    _surfCube[toIndex(gridI, gridJ, gridK)]->push_back(point);
  }
}

template <typename PointT>
void FeatureMap<PointT>::addCornerCloud(const PointCloud &cloud) {
  for (int i = 0; i < cloud.points.size(); ++i) {
    pushCornerPoint(cloud[i]);
  }
}
template <typename PointT>
void FeatureMap<PointT>::addSurfCloud(const PointCloud &cloud) {
  for (int i = 0; i < cloud.points.size(); ++i) {
    pushSurfPoint(cloud[i]);
  }
}
template <typename PointT>
void FeatureMap<PointT>::addFeatureCloud(const PointCloud &cornerCloud,
                                         const PointCloud &surfCloud,
                                         const Eigen::Isometry3f &tf) {
  _cloudCornerSwap->clear();
  lidar_slam::transformPointCloud(cornerCloud, *_cloudCornerSwap, tf);
  addCornerCloud(*_cloudCornerSwap);
  _cloudSurfSwap->clear();
  lidar_slam::transformPointCloud(surfCloud, *_cloudSurfSwap, tf);
  addSurfCloud(*_cloudSurfSwap);

  downsizeValidCloud();
}
template <typename PointT>
inline void FeatureMap<PointT>::update(const PointT &sensorPose) {
  int gridI, gridJ, gridK;
  worldToCube(sensorPose.x, sensorPose.y, sensorPose.z, gridI, gridJ, gridK);
  int MIN_PADDING = 3;
  int newGridI =
      std::min(std::max(gridI, MIN_PADDING), _cubeWidth - MIN_PADDING - 1);
  int newGridJ =
      std::min(std::max(gridJ, MIN_PADDING), _cubeHeight - MIN_PADDING - 1);
  int newGridK =
      std::min(std::max(gridK, MIN_PADDING), _cubeDepth - MIN_PADDING - 1);

  shift(newGridI - gridI, newGridJ - gridJ, newGridK - gridK);

  _cubeOriginWidth += newGridI - gridI;
  _cubeOriginHeight += newGridJ - gridJ;
  _cubeOriginDepth += newGridK - gridK;

  _curCubeWidth = newGridI;
  _curCubeHeight = newGridJ;
  _curCubeDepth = newGridK;

  computeActiveAera(sensorPose);
}
template <typename PointT>
inline void FeatureMap<PointT>::getSurroundFeature(PointCloud &surroundCorner,
                                                   PointCloud &surroundSurf) {
  surroundCorner.clear();
  surroundSurf.clear();
  size_t validNum = _cubeValidInd.size();
  for (int i = 0; i < validNum; i++) {
    surroundCorner += *_cornerCube[_cubeValidInd[i]];
    surroundSurf += *_surfCube[_cubeValidInd[i]];
  }
}

template <typename PointT>
inline bool FeatureMap<PointT>::getFullMap(PointCloudPtr &mapCloud) {
  mapCloud->clear();
  for (int i = 0; i < _cubeNum; i++) {
    _cloudCornerSwap->clear();
    if (!_cornerCube[i]->empty()) {
      _downSizeFilterMap.setInputCloud(_cornerCube[i]);
      _downSizeFilterMap.filter(*_cloudCornerSwap);
      *mapCloud += *_cloudCornerSwap;
    }

    _cloudSurfSwap->clear();
    if (!_surfCube[i]->empty()) {
      _downSizeFilterMap.setInputCloud(_surfCube[i]);
      _downSizeFilterMap.filter(*_cloudSurfSwap);
      *mapCloud += *_cloudSurfSwap;
    }
  }
  return true;
}

template <typename PointT>
inline void FeatureMap<PointT>::downsizeValidCloud() {
  // down size all valid (within field of view) feature cube clouds
  size_t validNum = _cubeValidInd.size();
  for (int i = 0; i < validNum; i++) {
    size_t ind = _cubeValidInd[i];
    _cloudCornerSwap->clear();
    _downSizeFilterCorner.setInputCloud(_cornerCube[ind]);
    _downSizeFilterCorner.filter(*_cloudCornerSwap);

    _cloudSurfSwap->clear();
    _downSizeFilterSurf.setInputCloud(_surfCube[ind]);
    _downSizeFilterSurf.filter(*_cloudSurfSwap);

    // swap cube clouds for next processing
    _cornerCube[ind].swap(_cloudCornerSwap);
    _surfCube[ind].swap(_cloudSurfSwap);
  }
}
template <typename PointT>
inline void FeatureMap<PointT>::computeActiveAera(const PointT sensorPose) {
  _cubeValidInd.clear();
  int window_size = std::ceil(_lidarValidDistance / _worldCubeSize);
  for (int i = _curCubeWidth - window_size; i <= _curCubeWidth + window_size;
       i++) {
    for (int j = _curCubeHeight - window_size;
         j <= _curCubeHeight + window_size; j++) {
      for (int k = _curCubeDepth - window_size;
           k <= _curCubeDepth + window_size; k++) {
        if (isIndexValid(i, j, k)) {

          float centerX = _worldCubeSize * (i - _cubeOriginWidth);
          float centerY = _worldCubeSize * (j - _cubeOriginHeight);
          float centerZ = _worldCubeSize * (k - _cubeOriginDepth);

          bool isInLaserFOV = false;
          for (int ii = -1; ii <= 1; ii += 2) {
            for (int jj = -1; jj <= 1; jj += 2) {
              for (int kk = -1; kk <= 1; kk += 2) {
                pcl::PointXYZI corner;
                corner.x = centerX + _worldCubeSize / 2.0 * ii;
                corner.y = centerY + _worldCubeSize / 2.0 * jj;
                corner.z = centerZ + _worldCubeSize / 2.0 * kk;

                float squareDist = calcSquaredDiff(sensorPose, corner);

                if (sqrt(squareDist) < _lidarValidDistance) {
                  isInLaserFOV = true;
                  break;
                }
              }
              if (isInLaserFOV)
                break;
            }
            if (isInLaserFOV)
              break;
          }
          if (isInLaserFOV) {
            _cubeValidInd.push_back(toIndex(i, j, k));
          }
        }
      }
    }
  }
}
template <typename PointT>
inline void FeatureMap<PointT>::shift(int dIndexI, int dIndexJ, int dIndexK) {
  if (dIndexI != 0 || dIndexJ != 0 || dIndexK != 0) {
    for (int i = 0; i < _cubeWidth; i++) {
      for (int j = 0; j < _cubeHeight; j++) {
        for (int k = 0; k < _cubeDepth; k++) {
          int oldI = i - dIndexI;
          int oldJ = j - dIndexJ;
          int oldK = k - dIndexK;
          if (isIndexValid(oldI, oldJ, oldK)) {
            std::swap(_cornerCube[toIndex(i, j, k)],
                      _cornerCube[toIndex(oldI, oldJ, oldK)]);
            std::swap(_surfCube[toIndex(i, j, k)],
                      _surfCube[toIndex(oldI, oldJ, oldK)]);
          } else {
            //@TODO: savetofiles...
            _cornerCube[toIndex(i, j, k)]->clear();
            _surfCube[toIndex(i, j, k)]->clear();
          }
        }
      }
    }
  }
}

template <typename PointT> inline bool FeatureMap<PointT>::saveCloudToFiles() {
  std::string indexFilePath = _filesDirectory;
  indexFilePath += "/index.txt";
  std::ofstream fout(indexFilePath.c_str());

  if (!fout) {
    std::cout << "save files error!" << std::endl;
    return false;
  }

  int count = 0;
  for (int i = 0; i < _cubeWidth; i++) {
    for (int j = 0; j < _cubeHeight; j++) {
      for (int k = 0; k < _cubeDepth; k++) {
        if (_cornerCube[toIndex(i, j, k)]->points.size() > 0) {
          std::string pcdFilePath = fileNameFormat(_filesDirectory, count);
          pcl::io::savePCDFileBinary(pcdFilePath,
                                     *_cornerCube[toIndex(i, j, k)]);
          fout << count << " " << 0 << " " << i << " " << j << " " << k << " "
               << _cornerCube[toIndex(i, j, k)]->points.size() << std::endl;
          count++;
        }
        if (_surfCube[toIndex(i, j, k)]->points.size() > 0) {
          std::string pcdFilePath = fileNameFormat(_filesDirectory, count);
          pcl::io::savePCDFileBinary(pcdFilePath, *_surfCube[toIndex(i, j, k)]);
          fout << count << " " << 1 << " " << i << " " << j << " " << k << " "
               << _surfCube[toIndex(i, j, k)]->points.size() << std::endl;
          count++;
        }
      }
    }
  }
  std::cout << "Save PCD files done!" << std::endl;
  return true;
}

template <typename PointT>
inline bool FeatureMap<PointT>::loadCloudFromFiles() {
  std::string indexFilePath = _filesDirectory;
  indexFilePath += "/index.txt";
  std::ifstream fin(indexFilePath.c_str());
  if (!fin)
    return false;

  int count, type, i, j, k, size = 0;
  while (!fin.eof()) {
    fin >> count >> type >> i >> j >> k >> size;

    std::string pcdFilePath = fileNameFormat(_filesDirectory, count);

    if (type == 0) {
      PointCloudPtr cloudCornerPointer(new PointCloud());
      if (pcl::io::loadPCDFile<PointT>(pcdFilePath, *cloudCornerPointer) ==
          -1) {
        std::cout << pcdFilePath << " not exist!" << std::endl;
      } else {
        _cloudCornerSwap->clear();
        _downSizeFilterCorner.setInputCloud(cloudCornerPointer);
        _downSizeFilterCorner.filter(*_cloudCornerSwap);
        _cornerCube[toIndex(i, j, k)].swap(_cloudCornerSwap);
        _kdtreeCorner[toIndex(i, j, k)].setInputCloud(_cornerCube[toIndex(i, j, k)]);
        //*_cornerCube[toIndex(i, j, k)] = *cloudCornerPointer;
      }

      cloudCornerPointer->clear();
    }
    if (type == 1) {
      PointCloudPtr cloudSurfPointer(new PointCloud());
      if (pcl::io::loadPCDFile<PointT>(pcdFilePath, *cloudSurfPointer) == -1) {
        std::cout << pcdFilePath << " not exist!" << std::endl;
      } else {
        _cloudSurfSwap->clear();
        _downSizeFilterSurf.setInputCloud(cloudSurfPointer);
        _downSizeFilterSurf.filter(*_cloudSurfSwap);
        _surfCube[toIndex(i, j, k)].swap(_cloudSurfSwap);
        _kdtreeSurf[toIndex(i, j, k)].setInputCloud(_surfCube[toIndex(i, j, k)]);
        //*_surfCube[toIndex(i, j, k)] = *cloudSurfPointer;
      }
      cloudSurfPointer->clear();
    }
  }

  std::cout << "Load cloud files Done!" << std::endl;
  return true;
}

template <typename PointT>
inline int FeatureMap<PointT>::worldToIndex(float world_x, float world_y,
                                            float world_z) {
  int gridI, gridJ, gridK;
  if (worldToCube(world_x, world_y, world_z, gridI, gridJ, gridK)) {
    return toIndex(gridI, gridJ, gridK);
  } else
    return -1;
}

template <typename PointT>
inline int FeatureMap<PointT>::worldToCube(float world_x, float world_y,
                                           float world_z, int &gridI,
                                           int &gridJ, int &gridK) {

  gridI = std::round(world_x / _worldCubeSize) + _cubeOriginWidth;
  gridJ = std::round(world_y / _worldCubeSize) + _cubeOriginHeight;
  gridK = std::round(world_z / _worldCubeSize) + _cubeOriginDepth;

  if (isIndexValid(gridI, gridJ, gridK))
    return true;
  else
    return false;
}

template <typename PointT>
inline bool FeatureMap<PointT>::scanMatchScan(const PointCloudConstPtr &CornerCloud,
                              const PointCloudConstPtr &SurfCloud,
                              Twist &transformf)
{
  Twist transform = transformf;

  PointT pointSel, pointOri, pointProj, coeff;
  std::vector<int> pointSearchInd(5, 0);
  std::vector<float> pointSearchSqDis(5, 0);

  bool converge = false;
  bool isDegenerate = false;
  Eigen::Matrix<float, 6, 6> matP;

  size_t CornerNum = CornerCloud->points.size();
  size_t SurfNum = SurfCloud->points.size();
  // printf("size: %d %d\n", CornerNum, SurfNum);

  pcl::PointCloud<PointT> laserCloudOri;
  pcl::PointCloud<PointT> coeffSel;


  int line_match_count = 0;
  int plane_match_count = 0;
  size_t iterCount;
  for (iterCount = 0; iterCount < 10; iterCount++) {
    laserCloudOri.clear();
    coeffSel.clear();
    line_match_count = 0;
    plane_match_count = 0;
    for (int i = 0; i < CornerNum; i++) {
      pointOri = CornerCloud->points[i];
      pointAssociateToMap(transform, pointOri, pointSel);
      int idx = worldToIndex(pointSel.x, pointSel.y, pointSel.z);
      if(idx<0) continue;
      if(_cornerCube[idx]->points.size() < 5) continue;
      _kdtreeCorner[idx].nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
      if (pointSearchSqDis.size() >= 5 && pointSearchSqDis[4] < 5.0) {
        const Eigen::Vector3f &point = pointSel.getVector3fMap();
        Eigen::Vector3f lineA, lineB;
        if (findLine(*_cornerCube[idx], pointSearchInd, lineA, lineB)) {
          PointT coefficients;
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
      int idx = worldToIndex(pointSel.x, pointSel.y, pointSel.z);
      if(idx<0) continue;
      if(_surfCube[idx]->points.size() < 5) continue;
      _kdtreeSurf[idx].nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
      if (pointSearchSqDis.size() >= 5 && pointSearchSqDis[4] < 5.0) {
        Eigen::Vector4f planeCoef;
        if (findPlane(*_surfCube[idx], pointSearchInd, 0.2, planeCoef)) {
          PointT coefficients;
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
    // printf("matched number: %d\n", laserCloudSelNum);
    if (laserCloudSelNum < 50) {
      ROS_WARN("matched cloud points too few. Matched/Input:  %zd / %zd", laserCloudSelNum, CornerNum+SurfNum);
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

    if (deltaR < 0.05 && deltaT < 0.05) {
      converge = true;
      break;
    }
  }
  transformf = transform;
}

template <typename PointT>
inline bool FeatureMap<PointT>::scanMatchScan(const PointCloudConstPtr &CornerCloud,
                              const PointCloudConstPtr &SurfCloud,
                              Eigen::Isometry3f &relative_pose){
  Twist transform;
  convertTransform(relative_pose, transform);
  bool success = scanMatchScan(CornerCloud, SurfCloud, transform);
  convertTransform(transform, relative_pose);
  return success;
}

}//namespace lidar_slam

#endif //__FEATURE_MAP_H__
