#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <common/Twist.h>
#include <common/math_utils.h>

#ifndef SCAN_MATCH_H__
#define SCAN_MATCH_H__
namespace lidar_slam {

class ScanMatch {
public:
  typedef pcl::PointXYZI PointI;
  typedef pcl::PointCloud<PointI> CloudI;
  typedef pcl::PointCloud<PointI>::ConstPtr CloudIConPtr;

  typedef pcl::PointXYZINormal PointIN;
  typedef pcl::PointCloud<PointIN> CloudIN;

  inline void setPercentThreshold(double percent) {
    _match_percentage_threshold = percent;
  }

  inline void setScoreThreshold(double score) { _score_threshold = score; }

  inline void setFineScore(bool enable) { _fineScore = enable; }

  inline void setConvergeThreshold(float deltaTAbort, float deltaRAbort) {
    _deltaTAbort = deltaTAbort;
    _deltaRAbort = deltaRAbort;
  }

  inline void setUseCore(bool useScore) { _useScore = useScore; }

  explicit ScanMatch(const size_t maxIterations = 10);
  ~ScanMatch();
  bool scanMatchLocal(const CloudIConPtr &referenceCornerCloud,
                      const CloudIConPtr &referenceSurfCloud,
                      const CloudIConPtr &CornerCloud,
                      const CloudIConPtr &SurfCloud,
                      Eigen::Isometry3f &relative_pose);
  bool scanMatchLocal(const CloudIConPtr &referenceCornerCloud,
                      const CloudIConPtr &referenceSurfCloud,
                      const CloudIConPtr &CornerCloud,
                      const CloudIConPtr &SurfCloud, Twist &transform);
  bool scanMatchScan(const CloudIConPtr &referenceCornerCloud,
                     const CloudIConPtr &referenceSurfCloud,
                     const CloudIConPtr &CornerCloud,
                     const CloudIConPtr &SurfCloud,
                     Eigen::Isometry3f &relative_pose);
  bool scanMatchScan(const CloudIConPtr &referenceCornerCloud,
                     const CloudIConPtr &referenceSurfCloud,
                     const CloudIConPtr &CornerCloud,
                     const CloudIConPtr &SurfCloud, Twist &transform);

  double getScore(const CloudI &coeffCloud);

  inline double getAverageScore() {
    return (_match_count > 0) ? _total_score / _match_count : 0;
  }

private:
  size_t _maxIterations; ///< maximum number of iterations
  float _deltaTAbort;    ///< optimization abort threshold for deltaT
  float _deltaRAbort;    ///< optimization abort threshold for deltaR

  pcl::VoxelGrid<PointI>
      _downSizeFilterCorner; ///< voxel filter for down sizing corner clouds
  pcl::VoxelGrid<PointI>
      _downSizeFilterSurf; ///< voxel filter for down sizing surface clouds

  CloudI::Ptr _referenceCornerCloudDS;
  CloudI::Ptr _referenceSurfCloudDS;
  CloudI::Ptr _CornerCloudDS;
  CloudI::Ptr _SurfCloudDS;

  // for socore
  bool _useScore;
  bool _fineScore;
  double _score_threshold;
  double _match_percentage_threshold;
  double _total_score;
  long _match_count;
  long _fail_match_count;
};
}
#endif // SCAN_MATCH_H__