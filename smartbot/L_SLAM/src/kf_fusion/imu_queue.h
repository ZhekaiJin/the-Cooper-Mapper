
#ifndef LIDAR_IMU_QUEUE_H
#define LIDAR_IMU_QUEUE_H

#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include "common/math_utils.h"
#include "fusion/transPointCLoud.h"
#include "fusion/utmProjection.h"
#include <exception>
#include <iostream>
#include <mutex>
#include <signal.h>
#include <sstream>
#include <stdlib.h>
#include <vector>

#include "kf/ukf_pose_estimator.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "loadExtrinsic.hpp"

using namespace lidar_slam;

class IMUQueue {
public:
  typedef Eigen::Matrix<float, Eigen::Dynamic, 1> VectorXt;
  typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> MatrixXt;

  IMUQueue() : que_size(1000) { initialize = false; }

  void setup(ros::NodeHandle &node, ros::NodeHandle &privateNode) {

    std::string extrinsic_file;
    if (privateNode.getParam("extrinsic_file", extrinsic_file)){
      ROS_INFO_STREAM("Set extrinsic file:"<<extrinsic_file);
    }else{
      ROS_ERROR("NO extrinsic file setup.");
    }
    if(loadExtrinsic(extrinsic_file, Tli)){
        ROS_INFO_STREAM("imu2lidar calibration Tli:\n" << Tli.matrix() << std::endl);
    }else{
        ROS_ERROR_STREAM("Error in "<<extrinsic_file);
    }

    subIMU = node.subscribe<sensor_msgs::Imu>("/imu/data_raw", 5,
                                              &IMUQueue::imuHandler, this);
    ukf_pose_estimator.reset(
        new kf::UKFPoseEstimator(ros::Time::now(), Eigen::Vector3f(0, 0, 0),
                                 Eigen::Quaternionf(1.0, 0.0, 0.0, 0.0), 0.5));
  }

  void imuHandler(const sensor_msgs::Imu::ConstPtr &msg) { addMsg(*msg); }

  void addMsg(sensor_msgs::Imu msg) {
    que_mutex.lock();
    imu_que.push_back(msg);
    if (imu_que.size() > que_size)
      imu_que.erase(imu_que.begin(), imu_que.begin() + 1);
    que_mutex.unlock();
  }

  bool predict(ros::Time &stamp, Eigen::Isometry3f &trans) {
    que_mutex.lock();

    // std::cout << "size:" << imu_que.size() << std::endl;
    int size = imu_que.size();
    if (size < 1) {
      que_mutex.unlock();
      return false;
    }

    auto seek = std::lower_bound(
        imu_que.begin(), imu_que.end(), stamp,
        [&](const sensor_msgs::Imu &msg, const ros::Time &stamp) {
          return msg.header.stamp < stamp;
        });

    if (seek == imu_que.end()) {
      ROS_INFO_STREAM("imu msg very slower than the lidar time"<< std::fixed
      <<"lidar time:"<< stamp << " imu newest time: " << (seek - 1)->header.stamp);
      que_mutex.unlock();

      return false;
    }

    if (seek == imu_que.begin()) {
      ROS_INFO_STREAM("imu msg very faster than the lidar time"<< std::fixed
      <<"lidar time:"<< stamp << " imu oldest time: " << seek->header.stamp);
      que_mutex.unlock();

      return false;
    }

    int count = 0;
    for (auto imu_iter = imu_que.begin(); imu_iter != seek; imu_iter++) {
      if (stamp < (imu_iter)->header.stamp) {
        break;
      }
      const auto &acc = (imu_iter)->linear_acceleration;
      const auto &gyro = (imu_iter)->angular_velocity;
      double gyro_sign = 1.0;
      count++;
      ukf_pose_estimator->predict(
          (imu_iter)->header.stamp, Eigen::Vector3f(acc.x, acc.y, acc.z),
          gyro_sign * Eigen::Vector3f(gyro.x, gyro.y, gyro.z));
    }
    imu_que.erase(imu_que.begin(), seek);
    que_mutex.unlock();
    trans.matrix() =
        ukf_pose_estimator->matrix() * Tli.inverse().matrix().cast<float>();
    /*
    std::cout << "count:" << count << "\n pos:" << ukf_pose_estimator->pos()
              << "\n vel:" << ukf_pose_estimator->vel()
              << "\n quat:" << ukf_pose_estimator->quat().coeffs() << std::endl;
    */
  }

  bool correct(const Eigen::Isometry3f &correct_pose, Eigen::Isometry3f &trans,
               Eigen::Vector3f &velocity) {
    Eigen::Isometry3f imu_pose = correct_pose * Tli.cast<float>();
    ukf_pose_estimator->correct(imu_pose, velocity);
    trans.matrix() =
        ukf_pose_estimator->matrix() * Tli.inverse().matrix().cast<float>();


    velocity = ukf_pose_estimator->vel();/*
    Eigen::Quaternionf q(imu_pose.rotation());
    std::cout << "correct:"
              << "\n pos:" << ukf_pose_estimator->pos()
              << "\n vel:" << ukf_pose_estimator->vel()
              << "\n quat:" << ukf_pose_estimator->quat().coeffs() << std::endl;
    */
  }

  bool reset(const Eigen::Vector3f &pos, const Eigen::Quaternionf &quat) {
    ukf_pose_estimator->reset(pos, quat);
  }

  const VectorXt &getMean() const { return ukf_pose_estimator->getMean(); }
  const MatrixXt &getCov() const { return ukf_pose_estimator->getCov(); }

private:
  Eigen::Isometry3d Tli;
  std::deque<sensor_msgs::Imu> imu_que;
  int que_size;
  std::mutex que_mutex;

  ros::Subscriber subIMU;
  std::unique_ptr<kf::UKFPoseEstimator> ukf_pose_estimator;

  tf::StampedTransform odomTF;
  tf::StampedTransform initTF;

  bool initialize;
};

#endif // LIDAR_IMU_QUEUE_H
