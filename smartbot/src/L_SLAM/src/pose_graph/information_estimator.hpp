#ifndef INFORMATION_ESTIMATOR_H__
#define INFORMATION_ESTIMATOR_H__

#include <pcl/point_types.h>
#include <ros/ros.h>

namespace pose_graph {

class InformationEstimator {
public:
  InformationEstimator() {}

  ~InformationEstimator() {}

  Eigen::MatrixXd getConstInformation(double const_stddev_x = 0.5,
                                      double const_stddev_q = 0.1) const {
    Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
    inf.topLeftCorner(3, 3).array() /= const_stddev_x;
    inf.bottomRightCorner(3, 3).array() /= const_stddev_q;
    return inf;
  }

  Eigen::MatrixXd getInformation(double fitness_score,
                                 const Eigen::Isometry3d &relpose) const {

    double weigth1 = fitness_score / 10000.0;
    double dx = relpose.translation().norm();
    Eigen::AngleAxisd axis(relpose.rotation());
    double da = axis.angle();

    //@ todo:
    double w_x = 1.0;
    double w_q = 1.0;

    Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
    inf.topLeftCorner(3, 3).array() *= w_x;
    inf.bottomRightCorner(3, 3).array() *= w_q;
    return inf;
  }

  double weight(double gain, double thresh, double min, double max,
                double x) const {
    double y = (1.0 - std::exp(-gain * x)) / (1.0 - std::exp(-gain * thresh));
    return min + (max - min) * y;
  }

private:
  double var_gain_a;
  double min_stddev_x;
  double max_stddev_x;
  double min_stddev_q;
  double max_stddev_q;
  double fitness_score_thresh;
};
}

#endif // INFORMATION_ESTIMATOR_H__
