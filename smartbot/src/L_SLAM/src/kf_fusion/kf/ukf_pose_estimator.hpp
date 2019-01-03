#ifndef UKF_POSE_ESTIMATOR_HPP
#define UKF_POSE_ESTIMATOR_HPP

#include <memory>
#include <ros/ros.h>

#include "pose_system.hpp"
#include "unscented_kalman_filter.hpp"

namespace lidar_slam {
namespace kf {

/**
 * @brief scan matching-based pose estimator
 */
class UKFPoseEstimator {
public:
  typedef Eigen::Matrix<float, Eigen::Dynamic, 1> VectorXt;
  typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> MatrixXt;

  /**
   * @brief constructor
   * @param stamp               timestamp
   * @param pos                 initial position
   * @param quat                initial orientation
   * @param cool_time_duration  during "cool time", prediction is not performed
   */
  UKFPoseEstimator(const ros::Time& stamp, const Eigen::Vector3f& pos, const Eigen::Quaternionf& quat, double cool_time_duration = 1.0)
    : init_stamp(stamp),
      cool_time_duration(cool_time_duration)
  {
    reset(pos, quat);
  }

  void reset(const Eigen::Vector3f& pos, const Eigen::Quaternionf& quat, Eigen::Vector3f vel = Eigen::Vector3f(0,0,0)){
    process_noise = Eigen::MatrixXf::Identity(16, 16);
    process_noise.middleRows(0, 3) *= 10.0;
    process_noise.middleRows(3, 3) *= 10.0;
    process_noise.middleRows(6, 4) *= 5.0;
    process_noise.middleRows(10, 3) *= 1e-6;
    process_noise.middleRows(13, 3) *= 1e-6;

    Eigen::MatrixXf measurement_noise = Eigen::MatrixXf::Identity(10, 10);
    measurement_noise.middleRows(0, 3) *= 0.01;
    measurement_noise.middleRows(3, 3) *= 0.1;
    measurement_noise.middleRows(6, 4) *= 0.001;

    Eigen::VectorXf mean(16);
    mean.middleRows(0, 3) = pos;
    mean.middleRows(3, 3) = vel;
    mean.middleRows(6, 4) = Eigen::Vector4f(quat.w(), quat.x(), quat.y(), quat.z());
    mean.middleRows(10, 3).setZero();
    mean.middleRows(13, 3).setZero();

    Eigen::MatrixXf cov = Eigen::MatrixXf::Identity(16, 16) * 0.01;

    PoseSystem system;
    ukf.reset(new UnscentedKalmanFilterX<float, PoseSystem>(system, 16, 6, 10, process_noise, measurement_noise, mean, cov));

  }
  /**
   * @brief predict
   * @param stamp    timestamp
   * @param acc      acceleration
   * @param gyro     angular velocity
   */
  void predict(const ros::Time& stamp, const Eigen::Vector3f& acc, const Eigen::Vector3f& gyro) {
    if((stamp - init_stamp).toSec() < cool_time_duration || prev_stamp.is_zero() || prev_stamp == stamp) {
      prev_stamp = stamp;
      return;
    }

    double dt = (stamp - prev_stamp).toSec();
    prev_stamp = stamp;

    ukf->setProcessNoiseCov(process_noise * dt);
    ukf->system.dt = dt;

    Eigen::VectorXf control(6);
    control.head<3>() = acc;
    control.tail<3>() = gyro;

    ukf->predict(control);
  }

  /**
   * @brief correct
   */
  void correct(const Eigen::Isometry3f& trans, Eigen::Vector3f& velocity) {
    Eigen::Vector3f p = trans.translation();
    Eigen::Quaternionf q(trans.rotation());
    Eigen::VectorXf observation(10);
    observation.middleRows(0, 3) = p;
    observation.middleRows(3, 3) = velocity;
    observation.middleRows(6, 4) = Eigen::Vector4f(q.w(), q.x(), q.y(), q.z());
    ukf->correct(observation);
  }

  /* getters */
  Eigen::Vector3f pos() const {
    return Eigen::Vector3f(ukf->mean[0], ukf->mean[1], ukf->mean[2]);
  }

  Eigen::Vector3f vel() const {
    return Eigen::Vector3f(ukf->mean[3], ukf->mean[4], ukf->mean[5]);
  }

  Eigen::Quaternionf quat() const {
    return Eigen::Quaternionf(ukf->mean[6], ukf->mean[7], ukf->mean[8], ukf->mean[9]).normalized();
  }

  Eigen::Matrix4f matrix() const {
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    m.block<3, 3>(0, 0) = quat().toRotationMatrix();
    m.block<3, 1>(0, 3) = pos();
    return m;
  }
  const VectorXt& getMean() const { return ukf->getMean(); }
  const MatrixXt& getCov() const { return ukf->getCov(); }
  const MatrixXt& getSigmaPoints() const { return ukf->getSigmaPoints(); }

private:
  ros::Time init_stamp;         // when the estimator was initialized
  ros::Time prev_stamp;         // when the estimator was updated last time
  double cool_time_duration;    //

  Eigen::MatrixXf process_noise;
  std::unique_ptr<UnscentedKalmanFilterX<float, PoseSystem>> ukf;

};
} // namespace kf
} // namespace lidar_slam

#endif // UKF_POSE_ESTIMATOR_HPP
