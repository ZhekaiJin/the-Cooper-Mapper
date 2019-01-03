#ifndef LOOP_DETECTOR_H__
#define LOOP_DETECTOR_H__

#include "common/nanoflann_pcl.h"
#include "common/transform_utils.h"

#include "keyframe.h"
#include "scan_match/ScanMatch.h"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/registration.h>

namespace pose_graph {

struct Loop {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<Loop>;

  Loop(const KeyFrame::Ptr &key1, const KeyFrame::Ptr &key2,
       const Eigen::Isometry3f &relpose)
      : key1(key1), key2(key2), relative_pose(relpose) {}

  void dump(const std::string &directory) {
    if (!boost::filesystem::is_directory(directory)) {
      boost::filesystem::create_directory(directory);
    }
    pcl::io::savePCDFileBinary(
        directory + "/" + std::to_string(key1->frame_id) + "-" +
            std::to_string(key2->frame_id) + "surf1.pcd",
        *key1->surfCloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr surfCloud2(
        new pcl::PointCloud<pcl::PointXYZI>());
    lidar_slam::transformPointCloud(*key2->surfCloud, *surfCloud2,
                                    relative_pose);
    pcl::io::savePCDFileBinary(
        directory + "/" + std::to_string(key1->frame_id) + "-" +
            std::to_string(key2->frame_id) + "surf2.pcd",
        *surfCloud2);
  }

public:
  KeyFrame::Ptr key1;
  KeyFrame::Ptr key2;
  Eigen::Isometry3f relative_pose;
};

class LoopDetector {
public:
  typedef pcl::PointXYZI PointI;
  typedef pcl::PointCloud<PointI> CloudI;

  LoopDetector()
      : estimated_distance_thresh(25.0), accum_distance_thresh(30.0),
        last_loop_interval_thresh(3.0), fitness_score_thresh(0.5),
        loop_count(0), trajectory_cloud(new CloudI()),
        registration(new pcl::IterativeClosestPoint<PointI, PointI>())
  {
    last_loop_accum_distance = 0.0;
  }

  bool detect_nearest(const std::vector<KeyFrame::Ptr> &keyframes,
                      const std::deque<KeyFrame::Ptr> &new_keyframes,
                      std::vector<Loop::Ptr> &detected_loops) {
    updateTrajectory(keyframes);
    bool loop_found = false;
    for (const auto &new_keyframe : new_keyframes) {
      std::vector<KeyFrame::Ptr> candidates;
      if (find_nearest_candidates(keyframes, new_keyframe, candidates)) {
        auto loop = matching_nearest(candidates, new_keyframe);

        if (loop) {
          //loop->dump("/home/hr/lidar_slam/loop");
          loop_found = true;
          detected_loops.push_back(loop);
          loop_count++;
        }
      }
    }

    return loop_found;
  }

  double get_distance_thresh() const { return estimated_distance_thresh; }
  double get_loop_count() const { return loop_count; }

private:
  void updateTrajectory(const std::vector<KeyFrame::Ptr> &keyframes) {
    trajectory_cloud->clear();
    for (int i = 0; i < keyframes.size(); i++) {
      Eigen::Isometry3f estimate = keyframes[i]->node->estimate().cast<float>();
      PointI pos;
      pos.getVector3fMap() = estimate.translation();
      pos.y = 0;
      pos.intensity = i;
      trajectory_cloud->push_back(pos);
    }

    trajectory_tree.setInputCloud(trajectory_cloud);
  }

  bool find_nearest_candidates(const std::vector<KeyFrame::Ptr> &keyframes,
                               const KeyFrame::Ptr &new_keyframe,
                               std::vector<KeyFrame::Ptr> &candidates) const {

    // too close to the last registered loop edge
    if (new_keyframe->accum_distance - last_loop_accum_distance <
        last_loop_interval_thresh) {
      return false;
    }

    bool candidate_found = false;

    Eigen::Isometry3f estimate = new_keyframe->node->estimate().cast<float>();
    PointI pos;
    pos.getVector3fMap() = estimate.translation();
    pos.y = 0;
    // std::cout << "query pos:" << pos << std::endl;

    std::vector<int> r_indices(256, 0);
    std::vector<float> r_sqr_distances(256, 0);
    int r_found =
        // trajectory_tree.nearestKSearch(pos, 128, r_indices, r_sqr_distances);
        trajectory_tree.radiusSearch(pos, 5.0, r_indices, r_sqr_distances);
    if (r_found <= 0)
      return false;

    int candidate_count = 0;
    double candidate_dist = 0;
    for (int i = 0; (i < r_sqr_distances.size()) && (candidate_count < 6);
         i++) {

      if (r_sqr_distances[i] >= estimated_distance_thresh) {
        break;
      }
      int keyframe_index =
          std::round(trajectory_cloud->points[r_indices[i]].intensity);
      // std::cout << "keyframe_index:" << keyframe_index << std::endl;

      // traveled distance between keyframes is too small
      if (new_keyframe->accum_distance -
              keyframes[keyframe_index]->accum_distance <
          accum_distance_thresh) {
        continue;
      }
      // std::cout << "pos:" << pos << std::endl;
      if (!candidate_count) {
        candidate_dist = keyframes[keyframe_index]->accum_distance;
      } else {
        if (fabs(candidate_dist - keyframes[keyframe_index]->accum_distance) >
            5.0) {
          continue;
        }
      }
      candidate_found = true;
      candidate_count++;
      candidates.push_back(keyframes[keyframe_index]);
    }
    return candidate_found;
  }

  Loop::Ptr
  matching_nearest(const std::vector<KeyFrame::Ptr> &candidate_keyframes,
                   const KeyFrame::Ptr &new_keyframe) {
    if (candidate_keyframes.empty()) {
      return nullptr;
    }

    CloudI::Ptr cornerLocal(new CloudI());
    CloudI::Ptr surfLocal(new CloudI());
    CloudI::Ptr cornerTemp(new CloudI());
    CloudI::Ptr surfTemp(new CloudI());
    Eigen::Isometry3d candidate_tf = candidate_keyframes[0]->node->estimate();
    *cornerLocal += *candidate_keyframes[0]->cornerCloud;
    *surfLocal += *candidate_keyframes[0]->surfCloud;

    for (int i = 1; i < candidate_keyframes.size(); i++) {
      Eigen::Isometry3f relative_tf =
          (candidate_tf.inverse() * candidate_keyframes[i]->node->estimate())
              .cast<float>();
      cornerTemp->clear();
      surfTemp->clear();
      transformPointCloud(*candidate_keyframes[i]->cornerCloud, *cornerTemp,
                          relative_tf);
      *cornerLocal += *cornerTemp;
      transformPointCloud(*candidate_keyframes[i]->surfCloud, *surfTemp,
                          relative_tf);
      *surfLocal += *surfTemp;
    }

    Eigen::Isometry3f new_relative_candi =
        (candidate_tf.inverse() * new_keyframe->node->estimate()).cast<float>();
    Eigen::Matrix4f corse_guess = new_relative_candi.matrix();

    bool corseMatched = corseMatching(surfLocal, new_keyframe->surfCloud, corse_guess);
    if(!corseMatched){
      return nullptr;
    }

    Eigen::Isometry3f guess = new_relative_candi;
    Eigen::Isometry3f guess2(corse_guess);
    bool hasConverged = scan_match.scanMatchLocal(
        cornerLocal, surfLocal, new_keyframe->cornerCloud,
        new_keyframe->surfCloud, guess2);

    if (hasConverged) {
      std::cout << "--- loop detection found2 ---" << std::endl;
      Eigen::Isometry3f delta = guess.inverse() * guess2;
      double dx = delta.translation().norm();
      Eigen::AngleAxisf axis(delta.rotation());
      double da = axis.angle();
      std::cout << "guess2:" << guess2.matrix() << std::endl;

      // std::cout << "deltax:" << dx << ",deltaa:" << da << std::endl;
    }

    if (!hasConverged) {
      return nullptr;
    }
    last_loop_accum_distance = new_keyframe->accum_distance;
    return std::make_shared<Loop>(candidate_keyframes[0], new_keyframe, guess2);
  }

  bool corseMatching(const CloudI::Ptr &referCloud, const CloudI::Ptr &cloud,
                     Eigen::Matrix4f &guess) {
    if (referCloud->empty()) {
      return false;
    }

    registration->setInputTarget(referCloud);
    pcl::PointCloud<PointI>::Ptr aligned(new pcl::PointCloud<PointI>());

    registration->setInputSource(cloud);
    registration->align(*aligned, guess);

    double score = registration->getFitnessScore();
    //std::cout << "corseMatching score:" << score << std::endl;
    //std::cout << "guess1:" << guess << std::endl;
    guess = registration->getFinalTransformation();
    //std::cout << "guess2:" << guess << std::endl;

    if (!registration->hasConverged()) {
      return false;
    }
    return true;
  }

private:
  // for candidates detect >accum_distance_thresh && <estimated_distance_thresh
  // && >last_loop_interval_thresh && >fitness_score_thresh
  double estimated_distance_thresh; // estimated distance between keyframes
                                    // consisting a loop must be less than this
                                    // distance
  double accum_distance_thresh;     // traveled distance between keyframes
                                    // consisting a loop must be less than this
                                    // distance

  // for distance between loops
  double last_loop_accum_distance;  // last traveled distance when detect loop
  double last_loop_interval_thresh; // a new loop edge must far from the last
                                    // one at least this distance

  double fitness_score_thresh; // threshold for scan matching

  lidar_slam::ScanMatch scan_match;

  pcl::Registration<PointI, PointI>::Ptr registration;
  //  pcl::GeneralizedIterativeClosestPoint<PointI, PointI>
  //  pcl::NormalDistributionsTransform<PointI, PointI>

  long loop_count;
  // nanoflann::KdTreeFLANN<PointI> trajectory_tree;
  pcl::KdTreeFLANN<PointI> trajectory_tree;

  CloudI::Ptr trajectory_cloud;
};
}

#endif // LOOP_DETECTOR_H__
