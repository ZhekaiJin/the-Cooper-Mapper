#ifndef KEYFRAME_UPDATER_HPP
#define KEYFRAME_UPDATER_HPP

#include <Eigen/Dense>

namespace pose_graph {


class KeyframeUpdater {
public:

  KeyframeUpdater()
      : is_first(true), prev_keypose(Eigen::Isometry3d::Identity()),
        keyframe_delta_trans(0.25), keyframe_delta_angle(0.05) ,frame_count(0){
    accum_distance = 0.0;
  }
  ~KeyframeUpdater(){
    std::cout<<"frame_count:"<<frame_count<<std::endl;
  }

  bool update(const Eigen::Isometry3d &pose) {
    if (is_first) {
      is_first = false;
      prev_keypose = pose;
      return true;
    }

    // calculate the delta transformation from the previous keyframe
    Eigen::Isometry3d delta = prev_keypose.inverse() * pose;
    double dx = delta.translation().norm();
    Eigen::AngleAxisd axis(delta.rotation());
    double da = axis.angle();

    // too close to the previous frame
    if (dx < keyframe_delta_trans && da < keyframe_delta_angle) {
      return false;
    }
    //std::cout<<"dx:"<<dx<<",da:"<<da<<std::endl;
    accum_distance += dx;
    prev_keypose = pose;
    frame_count++;
    return true;
  }


  double get_accum_distance() const { return accum_distance; }
  long get_unique_id() { return ++frame_id; }
  long get_frame_count() { return frame_count; }

private:
  // parameters
  double keyframe_delta_trans; //
  double keyframe_delta_angle; //

  bool is_first;
  long frame_id;
  double accum_distance;
  long frame_count;
  Eigen::Isometry3d prev_keypose;
};
}

#endif // KEYFRAME_UPDATOR_HPP
