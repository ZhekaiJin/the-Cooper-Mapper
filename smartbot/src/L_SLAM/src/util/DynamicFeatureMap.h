#ifndef __DYNAMIC_FEATURE_MAP_H__
#define __DYNAMIC_FEATURE_MAP_H__

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

#include <fstream>
#include <sstream>
#include <string>

// #include "sub_map/math_utils.h"
#include "transform_utils.h"
#include "FeatureMap.h"



namespace lidar_slam {

class Point3d
{
public:
  Point3d(int width = 0, int height = 0, int depth = 0)
    : _width(width), _height(height), _depth(depth) {}
  int &operator[](int index)
  {
    if(index == 0) return _width;
    else if(index == 1) return _height;
    else return _depth;
  }
  ~Point3d(){};
  bool operator < (const Point3d & rhs) const
  {
    if(_width == rhs._width && _height == rhs._height)
      return _depth < rhs._depth;
    else if(_width == rhs._width)
      return _height < rhs._height;
    else
      return _width < rhs._width;
  }
  int _width, _height, _depth;
};

// template <typename PointT> class PointCloudCube {
// public:
//   typedef typename pcl::PointCloud<PointT> PointCloud;
//   typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;

//   PointCloudCube(int cubeNum) : _cube(cubeNum) {
//     for (int i = 0; i < cubeNum; i++) {
//       _cube[i].reset(new PointCloud);
//     }
//   }

//   PointCloudPtr &operator[](int index) { return _cube[index]; }

//   const PointCloudPtr &operator[](int index) const { return _cube[index]; }

// private:
//   std::vector<PointCloudPtr> _cube;
// };

//---------- DynamicFeatureMap ---------------------

template <typename PointT> class DynamicFeatureMap {
public:
  typedef typename pcl::PointCloud<PointT> PointCloud;
  typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
  typedef typename pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;
  typedef typename pcl::VoxelGrid<PointT> VoxelGrid;

  DynamicFeatureMap(int cubeWidth_ = 21, int cubeHeight_ = 11, int cubeDepth_ = 21)
      : _cube(cubeWidth_, cubeHeight_, cubeDepth_),
        _cubeNum(cubeWidth_ * cubeHeight_ * cubeDepth_),
        _sensorGloId(0, 0, 0),
        _cornerCube(cubeWidth_ * cubeHeight_ * cubeDepth_), _surfCube(cubeWidth_ * cubeHeight_ * cubeDepth_),
        _oldCornerCube(cubeWidth_ * cubeHeight_ * cubeDepth_), _oldSurfCube(cubeWidth_ * cubeHeight_ * cubeDepth_),
        _worldCubeSize(50.0),
        _firstRead(1),
        _lidarMaxUpDegree(20.0),
        _lidarMaxDownDegree(20.0),
        _lidarValidDistance(100.0), _cloudCornerSwap(new PointCloud()),
        _cloudSurfSwap(new PointCloud()), _filePath("~") {
    _downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);
    _downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);
    _downSizeFilterMap.setLeafSize(0.6, 0.6, 0.6);
    _kdtreeCorner.resize(_cubeNum);
    _kdtreeSurf.resize(_cubeNum);
    for (int i = 0; i < _cubeNum; i ++)
      _indexMap.push_back(i);
  }
  ~DynamicFeatureMap() {}

  inline void setupFilterSize(float corner, float surf, float map) {
    _downSizeFilterCorner.setLeafSize(corner, corner, corner);
    _downSizeFilterSurf.setLeafSize(surf, surf, surf);
    _downSizeFilterMap.setLeafSize(map, map, map);
  }

  inline void setupWorldOrigin(int sensorGloWidth, int sensorGloHeight,
                               int sensorGloDepth) {
    _sensorGloId._width = sensorGloWidth;
    _sensorGloId._height = sensorGloHeight;
    _sensorGloId._depth = sensorGloDepth;
  }

  inline void setupWorldCubeSize(float worldCubeSize) {
    _worldCubeSize = worldCubeSize;
  }

  inline void setupLidarValidDistance(float lidarValidDistance) {
    _lidarValidDistance = lidarValidDistance;
  }

  inline void setupFilesDirectory(const std::string &filePath) {
    _filePath = filePath;
    if(!setupPCDFileName(_filePath))
      ROS_INFO("Fail to open index.txt!");
  }
  inline void setupLidarFov(float lidarMaxUpDegree, float lidarMaxDownDegree)
  {
    _lidarMaxUpDegree = lidarMaxUpDegree;
    _lidarMaxDownDegree = lidarMaxDownDegree;
  }
  inline bool setupPCDFileName(const std::string &filePath)
  {
    _surfGloIdxPCDName.clear();
    _cornerGloIdxPCDName.clear();
    std::string tmpFilePath = filePath;// outFilePath = filePath;
    tmpFilePath += "/index2.txt";
    // outFilePath += "/index2.txt";
    std::ifstream fin(tmpFilePath.c_str());
    // std::ofstream fout(outFilePath);
    if(!fin)
      return false;
    
    int count, type, i, j, k, size = 0;
    while(!fin.eof())
    {
      fin >> count >> type >> i >> j >> k >> size;
      // fout << count << " " << type << " " << i - 110 << " " << j - 105 << " " << k - 110 << " " << size << "\n";

      if(!type)
      {
        _cornerGloIdxPCDName[Point3d(i, j, k)] = count;
      }
      else
      {
        _surfGloIdxPCDName[Point3d(i, j, k)] = count;
      }
    }

    fin.close();

    std::cout << "setup PCD name done!\n";
    return true;
  }

  inline bool convertIndexFile(const std::string &filePath, int x, int y, int z){

    std::ifstream fin(filePath.c_str());
    // std::ofstream fout(outFilePath);
    if(!fin){
      std::cout << "input file error!" << std::endl;
      return false;
    }
    std::ofstream fout("index2.txt");
    if (!fout) {
      std::cout << "save file error!" << std::endl;
      return false;
    }
    int count, type, i, j, k, size;
    while(!fin.eof()){
      fin >> count >> type >> i >> j >> k >> size;
      fout << count << " " << type << " " << i-x << " " << j-y << " " << k-z << " "<<size << std::endl;;
    }
  }
  
  inline bool isIndexValid(int i, int j, int k) {
    if (0 <= i && i < _cube._width && 0 <= j && j < _cube._height && 0 <= k &&
        k < _cube._depth)
      return true;
    else
      return false;
  }

  //out 
  inline void update(const PointT &sensorGlo,  const Eigen::Vector3d &sensorUpDir);
  inline void getSurroundFeature(PointCloud &surroundCorner, PointCloud &surroundSurf);
  inline bool getFullMap(PointCloudPtr &mapCloud);
  inline void getLocIdxCloud(PointCloud &cubeCloud, const Eigen::Vector3d &locIdx);

  inline void addCornerCloud(const PointCloud &cloud);
  inline void addSurfCloud(const PointCloud &cloud);
  inline void pushCornerPoint(const PointT &point);
  inline void pushSurfPoint(const PointT &point);
  inline void downsizeValidCloud();
  inline void saveCloudToFiles(const Point3d &sensorGloId);


  inline void testAddFeatureCloud(const PointCloud &cornerCloud, const PointCloud  &surfCloud);
  inline void addFeatureCloud(const PointCloud &cornerCloud,
                       const PointCloud &surfCloud,
                       const Eigen::Isometry3f &tf);

  inline bool scanMatchScan(const PointCloudConstPtr &CornerCloud,
                              const PointCloudConstPtr &SurfCloud,
                              Twist &transformf);
  inline bool scanMatchScan(const PointCloudConstPtr &CornerCloud,
                              const PointCloudConstPtr &SurfCloud,
                              Eigen::Isometry3f &relative_pose);
                              
  //* global coordinate -> global coordinate index. e.g. (CellN, CellN, CellN) -> (1, 1, 1)
  inline void Glo2GloIdx(Point3d &gloIdx, const Eigen::Vector3d &glo);

  inline void Glo2GloIdx(Point3d &gloIdx, const PointT &glo);

  //* golbal coordinate index -> local positive index 
  inline void GloIdx2LocIdx(Point3d &locIdx, const Point3d &gloIdx, const Point3d &sensorGloIdx);
  //* local index -> local positive index
  inline void locIdx2LocPosIdx(Point3d &locPosIdx, const Point3d &locIdx);

  //* local positive index -> index[][][]
  inline int locPosIdx2IndexValue(const Point3d &locPosIdx);

  inline int locPosIdx2IndexValue(const int &locPosIdxWidth, const int &locPosIdxHeight, const int &locPosIdxDepth);

  inline void computeActiveAera(const PointT &sensorGlo, const Eigen::Vector3d &sensorUpDir);

  inline double VectorAngleDeg(const Eigen::Vector3d &vec_a, const Eigen::Vector3d &vec_b);

  inline bool InVerticalFov(const Point3d &locIdx, const Eigen::Vector3d &senserLocRealIdx, const Eigen::Vector3d &sensorUpDir);

  inline std::string CloudFileName(const Point3d &gloIdx, const int &flag);

private:
  inline int toIndex(const int &i, const int &j, const int &k) {
    return i + j * _cube._width + k * _cube._width * _cube._height;
  }
  inline int toIndex(const Point3d &locPosIdx)
  {
    return locPosIdx._width + locPosIdx._height * _cube._width + locPosIdx._depth * _cube._width * _cube._height;
  }
  // inline void toLocPosIdx(Point3d & locPosIdx, int ind)
  // {
  //   locPosIdx._width = ind % _cube._width;
  //   locPosIdx._height = (ind % (_cube._width * _cube._height)) / _cube._width;
  //   locPosIdx._depth = ind / (_cube._width * _cube._height);
  // }

  bool OutRange(const Point3d &point, const Point3d &center)
  {
    if(point._width < center._width - _cube._width / 2 || point._width > center._width + _cube._width / 2 || point._height < center._height - _cube._height / 2 
      || point._height > center._height + _cube._height / 2 || point._depth < center._depth - _cube._depth / 2 || point._depth > center._depth + _cube._depth / 2)
    {
      return 1;
    }
    return 0;
  }
  
private:
  Point3d _cube; // cube的边长
  Point3d _sensorGloId; // 传感器全局id

  int _cubeNum;         // cube块总个数
  float _worldCubeSize; // cube块的尺度，立方体 m*m*m
  float _lidarValidDistance; //雷达射程
  float _lidarMaxUpDegree; // 雷达仰角最大值
  float _lidarMaxDownDegree; // 雷达俯角最大值
  std::string _filePath; // cube块保存、加载路径

  bool _firstRead;
  PointCloudCube<PointT> _cornerCube; // 雷达新读入的角特征点
  PointCloudCube<PointT> _surfCube; // 雷达新读入的面特征点
  PointCloudCube<PointT> _oldCornerCube; // 从硬盘中读入的角特征点
  PointCloudCube<PointT> _oldSurfCube; // 从硬盘中读入的面特征点 

  map<Point3d, int> _surfGloIdxPCDName;
  map<Point3d, int> _cornerGloIdxPCDName;


  PointCloudPtr _cloudCornerSwap;
  PointCloudPtr _cloudSurfSwap;

  std::vector<size_t> _cubeValidInd;
  std::vector<size_t> _indexMap;

  VoxelGrid
      _downSizeFilterCorner; ///< voxel filter for down sizing corner clouds
  VoxelGrid
      _downSizeFilterSurf; ///< voxel filter for down sizing surface clouds

  VoxelGrid
      _downSizeFilterMap; ///< voxel filter for down sizing full map clouds

  std::vector< nanoflann::KdTreeFLANN<PointT> > _kdtreeCorner;
  std::vector< nanoflann::KdTreeFLANN<PointT> > _kdtreeSurf;
};

//---------- Definitions ---------------------


template <typename PointT>
inline void DynamicFeatureMap<PointT>::Glo2GloIdx(Point3d &gloIdx, const Eigen::Vector3d &glo)
{
  gloIdx._width  = (int) round(glo(0) / _worldCubeSize);
  gloIdx._height = (int) round(glo(1) / _worldCubeSize);
  gloIdx._depth  = (int) round(glo(2) / _worldCubeSize);  
}

template <typename PointT>
inline void DynamicFeatureMap<PointT>::Glo2GloIdx(Point3d &gloIdx, const PointT &glo)
{
  gloIdx._width  = (int) round(glo.x / _worldCubeSize);
  gloIdx._height = (int) round(glo.y / _worldCubeSize);
  gloIdx._depth  = (int) round(glo.z / _worldCubeSize);  
}

template <typename PointT>
inline void DynamicFeatureMap<PointT>::GloIdx2LocIdx(Point3d &locIdx, const Point3d &gloIdx, const Point3d &sensorGloIdx)
{
  locIdx._width  = gloIdx._width  - sensorGloIdx._width;
  locIdx._height = gloIdx._height - sensorGloIdx._height;
  locIdx._depth  = gloIdx._depth  - sensorGloIdx._depth;  
}

template <typename PointT>
inline void DynamicFeatureMap<PointT>::locIdx2LocPosIdx(Point3d & locPosIdx, const Point3d &locIdx)
{
  locPosIdx._width  = locIdx._width  + _cube._width  / 2;
  locPosIdx._height = locIdx._height + _cube._height / 2;
  locPosIdx._depth  = locIdx._depth  + _cube._depth  / 2;
}

template <typename PointT>
inline int DynamicFeatureMap<PointT>::locPosIdx2IndexValue(const Point3d &locPosIdx)
{
  int ind = this->toIndex(locPosIdx);
  return _indexMap[ind];
}

template <typename PointT>
inline int DynamicFeatureMap<PointT>::locPosIdx2IndexValue(const int &locPosIdxWidth, const int &locPosIdxHeight, const int &locPosIdxDepth)
{
  int ind = this->toIndex(locPosIdxWidth, locPosIdxHeight, locPosIdxDepth);
  return _indexMap[ind];
}

template <typename PointT>
inline void DynamicFeatureMap<PointT>::pushCornerPoint(const PointT &point) {
  Point3d gloIdx, locIdx, locPosIdx;
  this->Glo2GloIdx(gloIdx, point);
  this->GloIdx2LocIdx(locIdx, gloIdx, _sensorGloId);
  this->locIdx2LocPosIdx(locPosIdx, locIdx);

  if (isIndexValid(locPosIdx._width, locPosIdx._height, locPosIdx._depth)) {
    _cornerCube[locPosIdx2IndexValue(locPosIdx._width, locPosIdx._height, locPosIdx._depth)]->push_back(point);
  }
}
template <typename PointT>
inline void DynamicFeatureMap<PointT>::pushSurfPoint(const PointT &point) {
  Point3d gloIdx, locIdx, locPosIdx;
  this->Glo2GloIdx(gloIdx, point);
  this->GloIdx2LocIdx(locIdx, gloIdx, _sensorGloId);
  this->locIdx2LocPosIdx(locPosIdx, locIdx);

  if (isIndexValid(locPosIdx._width, locPosIdx._height, locPosIdx._depth)) {
    _surfCube[locPosIdx2IndexValue(locPosIdx._width, locPosIdx._height, locPosIdx._depth)]->push_back(point);
  }
}

template <typename PointT>
inline void DynamicFeatureMap<PointT>::addCornerCloud(const PointCloud &cloud) {
  for (int i = 0; i < cloud.points.size(); ++i) {
    pushCornerPoint(cloud[i]);
  }
}
template <typename PointT>
inline void DynamicFeatureMap<PointT>::addSurfCloud(const PointCloud &cloud) {
  for (int i = 0; i < cloud.points.size(); ++i) {
    pushSurfPoint(cloud[i]);
  }
}

template <typename PointT>
inline void DynamicFeatureMap<PointT>::addFeatureCloud(const PointCloud &cornerCloud,
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

// template <typename PointT>
// inline void DynamicFeatureMap<PointT>::testValidCloudGloIdx(vector<Eigen::Vector3> &validCloudGloIdx)
// {
//   validCloudGloIdx.clear();
//   for(int i = 0; i < _cubeValidInd.size(); i ++)
//   {

//   }
// }

template <typename PointT>
inline void DynamicFeatureMap<PointT>::testAddFeatureCloud(const PointCloud &cornerCloud, const PointCloud  &surfCloud)
{
  addCornerCloud(cornerCloud);
  addSurfCloud(surfCloud);
  downsizeValidCloud();
}

template <typename PointT>
inline void DynamicFeatureMap<PointT>::getLocIdxCloud(PointCloud &cubeCloud, const Eigen::Vector3d &locIdxVec)
{
  cubeCloud.clear();
  Point3d locIdx((int)locIdxVec(0), (int)locIdxVec(1), (int)locIdxVec(2));
  Point3d locPosIdx;
  this->locIdx2LocPosIdx(locPosIdx, locIdx);
  cubeCloud = *_oldCornerCube[locPosIdx2IndexValue(locPosIdx)];
}


template <typename PointT>
inline void DynamicFeatureMap<PointT>::saveCloudToFiles(const Point3d &sensorGloId)
{
  for(int i = _sensorGloId._width - _cube._width / 2; i <= _sensorGloId._width + _cube._width / 2; i ++)
  {
    for(int j = _sensorGloId._height - _cube._height / 2; j <= _sensorGloId._height + _cube._height / 2; j ++)
    {
      for(int k = _sensorGloId._depth - _cube._depth / 2; k <= _sensorGloId._depth + _cube._depth / 2; k ++)
      {
        Point3d gloIdx(i, j, k);
        if(OutRange(gloIdx, sensorGloId))
        {
          Point3d locIdx, locPosIdx;
          this->GloIdx2LocIdx(locIdx, gloIdx, _sensorGloId);
          this->locIdx2LocPosIdx(locPosIdx, locIdx);
          int idxVal = locPosIdx2IndexValue(locPosIdx);
          for(int x = 0; x < 2; x ++)
          {
            std::string name = CloudFileName(gloIdx, x);//* pcd flie name;
            // printf("to_depose_glo_idx: %d %d %d || center_glo_dix: %d %d %d\n", glo_idx.x, glo_idx.y, glo_idx.z, last_pos_glo_idx.x, last_pos_glo_idx.y, last_pos_glo_idx.z);
            // if(nmap[idx_val.x][idx_val.y][idx_val.z]->points.size())
            // {
            //   printf("deposed_glo_ind %d %d %d || pointcloud: %.0f %.0f %.0f || size: %d \n", glo_idx.x, glo_idx.y, glo_idx.z, nmap[idx_val.x][idx_val.y][idx_val.z]->points[0].x, nmap[idx_val.x][idx_val.y][idx_val.z]->points[0].y, nmap[idx_val.x][idx_val.y][idx_val.z]->points[0].z, nmap[idx_val.x][idx_val.y][idx_val.z]->points.size());
            // }
            // TO DO: recovery this annotation
            if(x)
            {
              if(!_surfCube[idxVal]->points.size()) continue;
              // printf("Add global Index Surface Cloud: %d %d %d\n", i, j, k);
              // pcl::io::savePCDFileASCII(name, *_surfCube[idxVal]);
              _surfCube[idxVal]->clear();
            }
            else
            {
              if(!_cornerCube[idxVal]->points.size()) continue;
              // printf("Add global Index Corner Cloud: %d %d %d\n", i, j, k);
              // pcl::io::savePCDFileASCII(name, *_cornerCube[idxVal]);
              _cornerCube[idxVal]->clear();
            }
          }
        }
      }
    }
  }
}

template <typename PointT>
inline std::string DynamicFeatureMap<PointT>::CloudFileName(const Point3d &gloIdx, const int &flag)
{
  std::string s;
  char fileName[1000];
  if(!flag)
  {
    if(_cornerGloIdxPCDName.count(gloIdx) > 0)
      sprintf(fileName, "/%d.pcd", _cornerGloIdxPCDName[gloIdx]);
    else
      return "";
  }
    
  else
  {
    if(_surfGloIdxPCDName.count(gloIdx) > 0)
      sprintf(fileName, "/%d.pcd", _surfGloIdxPCDName[gloIdx]);
    else
      return "";
  }
    
  s = _filePath + fileName;
  return s;
}

template <typename PointT>
inline void DynamicFeatureMap<PointT>::update(const PointT &sensorGlo, const Eigen::Vector3d &sensorUpDir) {
  Point3d sensorGloIdx;
  this->Glo2GloIdx(sensorGloIdx, sensorGlo);
  PointCloudPtr cloudCornerPointer(new PointCloud());
  PointCloudPtr cloudSurfPointer(new PointCloud());
  if(_firstRead)
  {
    for(int i = -_cube._width / 2; i <= _cube._width / 2; i ++)
    {
      for(int j = -_cube._height / 2; j <= _cube._height / 2; j ++)
      {
        for(int k = -_cube._depth / 2; k <= _cube._depth / 2; k ++)
        {
          Point3d gloIdx, locIdx, locPosIdx;
          gloIdx[0] = sensorGloIdx[0] + i;
          gloIdx[1] = sensorGloIdx[1] + j;
          gloIdx[2] = sensorGloIdx[2] + k; 
          this->GloIdx2LocIdx(locIdx, gloIdx, sensorGloIdx);
          this->locIdx2LocPosIdx(locPosIdx, locIdx);
          int idxVal = locPosIdx2IndexValue(locPosIdx);
          _oldCornerCube[idxVal]->clear();
          _oldSurfCube[idxVal]->clear();
          std::fstream file;
          for(int j = 0; j < 2; j ++)
          {
            std::string s = CloudFileName(gloIdx, j);
            if(s == "") continue;
            file.open(s.c_str());
            if(!file) continue;
            file.close();
            if(j)
            {
              cloudSurfPointer->clear();
              pcl::io::loadPCDFile<PointT> (s, *cloudSurfPointer);
              _downSizeFilterSurf.setInputCloud(cloudSurfPointer);
              _downSizeFilterSurf.filter(*_oldSurfCube[idxVal]);
              _kdtreeSurf[idxVal].setInputCloud(_oldSurfCube[idxVal]);
            }
            else
            {
              cloudCornerPointer->clear();
              pcl::io::loadPCDFile<PointT> (s, *cloudCornerPointer);
              _downSizeFilterCorner.setInputCloud(cloudCornerPointer);
              _downSizeFilterCorner.filter(*_oldCornerCube[idxVal]);
              _kdtreeCorner[idxVal].setInputCloud(_oldCornerCube[idxVal]);
            }
          }
        }
      }
    }
    _firstRead = 0;
  }
  else if(sensorGloIdx[0] != _sensorGloId._width  || 
      sensorGloIdx[1] != _sensorGloId._height || 
      sensorGloIdx[2] != _sensorGloId._depth)
  {
    // TO DO: recover this sentence.
    //saveCloudToFiles(sensorGloIdx);

    //* newGloIdx: record global index of new added cube
    //* oldRAM: record RAM(index of _cornerCube[][][]) of old cube
    //* dx, dy, dz: chagne of pose
    std::vector<Point3d> newGloIdx;
    std::vector<size_t> oldRAM;
    newGloIdx.clear();
    oldRAM.clear();
    int dx = sensorGloIdx[0] - _sensorGloId._width;
    int dy = sensorGloIdx[1] - _sensorGloId._height;
    int dz = sensorGloIdx[2] - _sensorGloId._depth;
    for (int i = sensorGloIdx[0] - _cube._width / 2; i <= sensorGloIdx[0] + _cube._width / 2; i ++)
    {
      for (int j = sensorGloIdx[1] - _cube._height / 2; j <= sensorGloIdx[1] + _cube._height / 2; j ++)
      {
        for (int k = sensorGloIdx[2] - _cube._depth / 2; k <= sensorGloIdx[2] + _cube._depth / 2; k ++)
        {
          Point3d gloIdx(i, j, k);
          if (OutRange(gloIdx, _sensorGloId))
          {
            newGloIdx.push_back(gloIdx);
          }
        }
      }
    }

    //* Update index array and store newly detected piont cloud in the removed gird into hard disk
    for(int i = _sensorGloId._width - _cube._width / 2; i <= _sensorGloId._width + _cube._width / 2; i ++)
    {
      for(int j = _sensorGloId._height - _cube._height / 2; j <= _sensorGloId._height + _cube._height / 2; j ++)
      {
        for(int k = _sensorGloId._depth - _cube._depth / 2; k <= _sensorGloId._depth + _cube._depth / 2; k ++)
        {
          Point3d gloIdx(i, j, k);
          if(OutRange(gloIdx, sensorGloIdx))
          {
            Point3d locIdx, locPosIdx;
            this->GloIdx2LocIdx(locIdx, gloIdx, _sensorGloId);
            this->locIdx2LocPosIdx(locPosIdx, locIdx);
            int idxVal = locPosIdx2IndexValue(locPosIdx);
            oldRAM.push_back(idxVal);
          }
        }
      }
    }
    std::vector<size_t> tIndexMap;
    for(int i = 0; i < _cubeNum; i ++)
      tIndexMap.push_back(-1);
    for(int i = 0; i < _cube._width; i ++)
    {
      for(int j = 0; j < _cube._height; j ++)
      {
        for(int k = 0; k < _cube._depth; k ++)
        {
          if(!OutRange(Point3d(i - dx, j - dy, k - dz), Point3d(_cube._width / 2, _cube._height / 2, _cube._depth / 2)))
          {
            tIndexMap[toIndex(i - dx, j - dy, k - dz)] = _indexMap[toIndex(i, j, k)];
          }
        }
      }
    }
    for(int i = 0; i < _cube._width; i ++)
      for(int j = 0; j < _cube._height; j ++)
        for(int k = 0; k < _cube._depth; k ++)
          if(tIndexMap[toIndex(i, j, k)] != -1)
            _indexMap[toIndex(i, j, k)] = tIndexMap[toIndex(i, j, k)];
    for(int i = 0; i < oldRAM.size(); i ++)//* arrange old RAM for new data 
    {
      Point3d locIdx, locPosIdx;
      this->GloIdx2LocIdx(locIdx, newGloIdx[i], sensorGloIdx);
      this->locIdx2LocPosIdx(locPosIdx, locIdx);
      _indexMap[toIndex(locPosIdx)]= oldRAM[i];
    }

    // printf("newGloNum: %d\n", newGloIdx.size());
    //* Read new data and update smap array
    for(int i = 0; i < newGloIdx.size(); i ++)
    {
      //index's arrary index
      Point3d gloIdx, locIdx, locPosIdx;
      this->GloIdx2LocIdx(locIdx, newGloIdx[i], sensorGloIdx);
      this->locIdx2LocPosIdx(locPosIdx, locIdx);
      int idxVal = locPosIdx2IndexValue(locPosIdx);
      _oldSurfCube[idxVal]->clear();
      _oldCornerCube[idxVal]->clear();
      std::fstream file;
      for(int j = 0; j < 2; j ++)
      {
        // std::cout << newGloIdx[i][0] << " " << newGloIdx[i][1] << " " << newGloIdx[i][2] << "\n";
        std::string s = CloudFileName(newGloIdx[i], j);
        if(s == "") continue;
        file.open(s.c_str());
        if(!file) continue;
        file.close();
        if(j)
        {
          cloudSurfPointer->clear();
          pcl::io::loadPCDFile<PointT> (s, *cloudSurfPointer);
          _downSizeFilterSurf.setInputCloud(cloudSurfPointer);
          _downSizeFilterSurf.filter(*_oldSurfCube[idxVal]);
          _kdtreeSurf[idxVal].setInputCloud(_oldSurfCube[idxVal]);
        }
        else
        {
          cloudCornerPointer->clear();
          pcl::io::loadPCDFile<PointT> (s, *cloudCornerPointer);
          _downSizeFilterCorner.setInputCloud(cloudCornerPointer);
          _downSizeFilterCorner.filter(*_oldCornerCube[idxVal]);
          _kdtreeCorner[idxVal].setInputCloud(_oldCornerCube[idxVal]);
        }
      }
    }
  }
  _sensorGloId = sensorGloIdx;
  this->computeActiveAera(sensorGlo, sensorUpDir);
}

// 得到视野内的雷达读入点云
template <typename PointT>
inline void DynamicFeatureMap<PointT>::getSurroundFeature(PointCloud &surroundCorner,
                                                   PointCloud &surroundSurf) {
  surroundCorner.clear();
  surroundSurf.clear();
  size_t validNum = _cubeValidInd.size();
  for (int i = 0; i < validNum; i++) {
    surroundCorner += *_oldCornerCube[_cubeValidInd[i]];
    surroundSurf += *_oldSurfCube[_cubeValidInd[i]];
    // printf("Corner Points Number: %d, Surf Points Number: %d\n",  _oldCornerCube[_cubeValidInd[i]]->points.size(), _oldSurfCube[_cubeValidInd[i]]->points.size());
  }
  // printf("%d Points Number: %d %d\n", validNum, surroundCorner.size(), surroundSurf.size());
}

// 得到cube内所有下采样后的角特征点云
template <typename PointT>
inline bool DynamicFeatureMap<PointT>::getFullMap(PointCloudPtr &mapCloud) {
  mapCloud->clear();
  for (int i = 0; i < _cubeNum; i++) {
    _cloudCornerSwap->clear();
    if (!_cornerCube[i]->empty()) {
      _downSizeFilterMap.setInputCloud(_oldCornerCube[i]);
      _downSizeFilterMap.filter(*_cloudCornerSwap);
      *mapCloud += *_cloudCornerSwap;
    }

    _cloudSurfSwap->clear();
    if (!_surfCube[i]->empty()) {
      _downSizeFilterMap.setInputCloud(_oldSurfCube[i]);
      _downSizeFilterMap.filter(*_cloudSurfSwap);
      *mapCloud += *_cloudSurfSwap;
    }
  }
  return true;
}

//下采样
template <typename PointT>
inline void DynamicFeatureMap<PointT>::downsizeValidCloud() {
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

// 向量夹角
template <typename PointT>
inline double DynamicFeatureMap<PointT>::VectorAngleDeg(const Eigen::Vector3d &vec_a, const Eigen::Vector3d &vec_b)
{
  double cos_theta = vec_a.dot(vec_b) / ( sqrt(vec_a.dot(vec_a)) * sqrt(vec_b.dot(vec_b)) );
  double theta = acos(cos_theta);
  return theta / M_PI * 180;
}

//以locIdx为中心的小cube是否在雷达视场内
template <typename PointT>
inline bool DynamicFeatureMap<PointT>::InVerticalFov(const Point3d &locIdx, 
                              const Eigen::Vector3d &senserLocRealIdx, 
                              const Eigen::Vector3d &sensorUpDir)
{
  double d[2] = {-0.5, 0.5};
  int upNum = 0, downNum = 0;//* up_num/down_num: The number of points above/below the field of view 
  double minDis = -1.0;
  for(int dx = 0; dx < 2; dx ++)
  {
    for(int dy = 0; dy < 2; dy ++)
    {
      for(int dz = 0; dz < 2; dz ++)
      {
        double x = locIdx._width + d[dx] - senserLocRealIdx(0);
        double y = locIdx._height + d[dy] - senserLocRealIdx(1);
        double z = locIdx._depth + d[dz] - senserLocRealIdx(2);
        double theta = VectorAngleDeg(Eigen::Vector3d(x, y, z), sensorUpDir);
        if(minDis == -1.0) minDis = sqrt(pow(x * _worldCubeSize,2) + pow(y * _worldCubeSize, 2) + pow(z * _worldCubeSize, 2));
        else minDis = std::min(minDis, sqrt(pow(x * _worldCubeSize,2) + pow(y * _worldCubeSize, 2) + pow(z * _worldCubeSize, 2)));
        theta = 90 - theta;
        if(theta >= _lidarMaxUpDegree) upNum ++;
        else if(theta <= -_lidarMaxDownDegree) downNum ++;
      }
    }
  }
  // upNum == 8 || downNum == 8 || 
  if(minDis > _lidarValidDistance) return 0;
  return 1;
}

template <typename PointT>
inline void DynamicFeatureMap<PointT>::computeActiveAera(const PointT &sensorGlo, const Eigen::Vector3d &sensorUpDir) {
  _cubeValidInd.clear();
  Point3d sensorGloIdx;
  Glo2GloIdx(sensorGloIdx, sensorGlo);
  Eigen::Vector3d senseLocoRealIdx =
          Eigen::Vector3d(sensorGlo.x / _worldCubeSize - sensorGloIdx._width, sensorGlo.y / _worldCubeSize - sensorGloIdx._height, 
                            sensorGlo.z / _worldCubeSize - sensorGloIdx._depth);
  int window_size = std::ceil(_lidarValidDistance / _worldCubeSize);
  for(int i = -window_size; i <= window_size; i ++)
  {
    for(int j = -window_size; j <= window_size; j ++)
    {
      for(int k = -window_size; k <= window_size; k ++)
      {
        if(!_cornerGloIdxPCDName.count(Point3d(sensorGloIdx[0] + i, sensorGloIdx[1] + j, sensorGloIdx[2] + k))
          && !_surfGloIdxPCDName.count(Point3d(sensorGloIdx[0] + i, sensorGloIdx[1] + j, sensorGloIdx[2] + k)))
          continue;
        if((!i && !j && !k) || InVerticalFov(Point3d(i, j, k), senseLocoRealIdx, sensorUpDir))
        {
          _cubeValidInd.push_back(locPosIdx2IndexValue(Point3d(i + _cube._width / 2, j + _cube._height / 2, k + _cube._depth / 2)));
          
        }
      }
    }
  }
}

template <typename PointT>
inline bool DynamicFeatureMap<PointT>::scanMatchScan(const PointCloudConstPtr &CornerCloud,
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
  //printf("size: %d %d\n", CornerNum, SurfNum);

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
      Point3d gloIdx, locIdx, locPosIdx;
      // printf("point sel: %.3f %.3f %.3f\n", pointSel.x, pointSel.y, pointSel.z);
      Glo2GloIdx(gloIdx, pointSel);
      // printf("Point glo Idx: (%d %d %d)\n", gloIdx[0], gloIdx[1], gloIdx[2]);
      GloIdx2LocIdx(locIdx, gloIdx, _sensorGloId);
      // printf("Loc Idx: (%d %d %d)\n", locIdx[0], locIdx[1], locIdx[2]);
      locIdx2LocPosIdx(locPosIdx, locIdx);
      // printf("Loc Positive Idx: (%d %d %d)\n", locPosIdx[0], locPosIdx[1], locPosIdx[2]);
      int idx = locPosIdx2IndexValue(locPosIdx);
      // printf("Point glo Idx: (%d %d %d), sensor glo Idx: (%d %d %d)\n", gloIdx[0], gloIdx[1], gloIdx[2], _sensorGloId[0], _sensorGloId[1], _sensorGloId[2]);
      if(_oldCornerCube[idx]->points.size() < 5) continue;
      _kdtreeCorner[idx].nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
      if (pointSearchSqDis.size() >= 5 && pointSearchSqDis[4] < 5.0) {
        const Eigen::Vector3f &point = pointSel.getVector3fMap();
        Eigen::Vector3f lineA, lineB;
        if (findLine(*_oldCornerCube[idx], pointSearchInd, lineA, lineB)) {
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
      Point3d gloIdx, locIdx, locPosIdx;
      Glo2GloIdx(gloIdx, pointSel);
      GloIdx2LocIdx(locIdx, gloIdx, _sensorGloId);
      locIdx2LocPosIdx(locPosIdx, locIdx);
      int idx = locPosIdx2IndexValue(locPosIdx);
      if(_oldSurfCube[idx]->points.size() < 5) continue;
      _kdtreeSurf[idx].nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
      if (pointSearchSqDis.size() >= 5 && pointSearchSqDis[4] < 5.0) {
        Eigen::Vector4f planeCoef;
        if (findPlane(*_oldSurfCube[idx], pointSearchInd, 0.2, planeCoef)) {
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
inline bool DynamicFeatureMap<PointT>::scanMatchScan(const PointCloudConstPtr &CornerCloud,
                              const PointCloudConstPtr &SurfCloud,
                              Eigen::Isometry3f &relative_pose){
  Twist transform;
  convertTransform(relative_pose, transform);
  bool success = scanMatchScan(CornerCloud, SurfCloud, transform);
  convertTransform(transform, relative_pose);
  return success;
}
}

#endif //__DYNAMIC_FEATURE_MAP_H__
