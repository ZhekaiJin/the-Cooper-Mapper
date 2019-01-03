
#ifndef TRANSFORM_MAINTENANCE
#define TRANSFORM_MAINTENANCE

#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include "common/math_utils.h"
#include "common/ros_utils.h"

#include "fusion/transPointCLoud.h"
#include "fusion/utmProjection.h"
#include "fusion/kf/ukf_pose_estimator.hpp"
#include "fusion/loadExtrinsic.hpp"

#include <exception>
#include <iostream>
#include <mutex>
#include <signal.h>
#include <sstream>
#include <stdlib.h>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>


namespace lidar_slam{



class TransformMaintenance {
public:
  typedef Eigen::Matrix<float, Eigen::Dynamic, 1> VectorXt;
  typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> MatrixXt;

  TransformMaintenance() : que_size(200) {
      initialize = false;
      lastCorrect = Eigen::Isometry3d::Identity();
   }

  bool setup(ros::NodeHandle &node, ros::NodeHandle &privateNode) {

    std::string extrinsic_file;
    if (privateNode.getParam("extrinsic_file", extrinsic_file)){
      ROS_INFO_STREAM("Set extrinsic file:"<<extrinsic_file);
    }else{
      ROS_ERROR("NO extrinsic file setup.");
      return false;
    }
    if(loadExtrinsic(extrinsic_file, Tli)){
        ROS_INFO_STREAM("imu2lidar calibration Tli:\n" << Tli.matrix() << std::endl);
    }else{
        ROS_ERROR_STREAM("Error in "<<extrinsic_file);
        return false;
    }

    Qli = Eigen::Quaternionf(Tli.rotation().cast<float>());

    subIMU = node.subscribe<sensor_msgs::Imu>("/imu/data_raw", 5,
                                              &TransformMaintenance::imuHandler, this);

    _pubLaserPredect = node.advertise<nav_msgs::Odometry>("/lidar_to_map", 5);

      _subOdomAftMapped = node.subscribe<nav_msgs::Odometry>(
      "/lidar_to_map2", 5, &TransformMaintenance::odomAftMappedHandler,
      this);

    ukf_pose_estimator.reset(
        new kf::UKFPoseEstimator(ros::Time::now(), Eigen::Vector3f(0, 0, 0),
                                 Eigen::Quaternionf(1.0, 0.0, 0.0, 0.0), 0.5));

    return true;
  }

  bool findNearestIMU(ros::Time &stamp, int& index) {
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
    index = seek - imu_que.begin();
    que_mutex.unlock();
    return true;
  }

  bool findNearest(const ros::Time &stamp, Eigen::Isometry3d &trans) {
    que_mutex.lock();

    //std::cout << "size:" << odom_que.size() << std::endl;
    int size = odom_que.size();
    if (size < 1){
      que_mutex.unlock();
      return false;
    }

    auto seek = std::lower_bound(
        odom_que.begin(), odom_que.end(), stamp,
        [&](const nav_msgs::Odometry &msg, const ros::Time &stamp) {
          return msg.header.stamp < stamp;
        });

    if (seek == odom_que.end()) {
      std::cout << "fpd msg slow than the time:" << std::endl;
      std::cout << std::fixed << stamp << "\n"
                << (seek-1)->header.stamp << std::endl;
     que_mutex.unlock();
      return false;
    }

    if (seek == odom_que.begin()) {
      std::cout << "fpd msg very faster than the time" << std::endl;
      std::cout << std::fixed << stamp << "\n"
                << seek->header.stamp << std::endl;
      que_mutex.unlock();
      return false;
    }
    double t0 = stamp.toSec();
    double t1 = (seek - 1)->header.stamp.toSec();
    double t2 = (seek)->header.stamp.toSec();

    // interplate
    // std::cout << "query  stamp:" << std::fixed << t0 << std::endl;
    // std::cout << "lower  stamp:" << std::fixed << t1 << std::endl;
    // std::cout << "larger stamp:" << std::fixed << t2 << std::endl;

    Eigen::Vector3d v1, v2;
    v1(0) = (seek - 1)->pose.pose.position.x;
    v1(1) = (seek - 1)->pose.pose.position.y;
    v1(2) = (seek - 1)->pose.pose.position.z;
    v2(0) = (seek)->pose.pose.position.x;
    v2(1) = (seek)->pose.pose.position.y;
    v2(2) = (seek)->pose.pose.position.z;

    Eigen::Quaterniond q1, q2;
    q1.x() = (seek - 1)->pose.pose.orientation.x;
    q1.y() = (seek - 1)->pose.pose.orientation.y;
    q1.z() = (seek - 1)->pose.pose.orientation.z;
    q1.w() = (seek - 1)->pose.pose.orientation.w;

    q2.x() = (seek)->pose.pose.orientation.x;
    q2.y() = (seek)->pose.pose.orientation.y;
    q2.z() = (seek)->pose.pose.orientation.z;
    q2.w() = (seek)->pose.pose.orientation.w;
    que_mutex.unlock();

    using std::abs;
    using std::sin;
    using std::acos;

    // std::cout << "v1:" << v1 << "\n v2:" << v2 << std::endl;
    // std::cout << "q1:" << q1.coeffs() << "\n q2:" << q2.coeffs() <<
    // std::endl;

    Eigen::Quaterniond q(q1.conjugate() * q2);
    q.normalize();

    Eigen::Quaterniond q0(Eigen::Quaterniond::Identity());
    Eigen::Quaterniond q22(q1 * q);

    // std::cout << "q22:" << q22.coeffs() << std::endl;
    // std::cout << "q:" << q.coeffs() << std::endl;
    double d = q0.dot(q);
    double abs_d = abs(d);

    double f = 1.0 / (t2 - t1);

    double t = (t0 - t1) * f;

    Eigen::Translation3d ti((1 - t) * v1 + t * v2);

    if (abs_d < 1.0 - 1.0e-8) {
      double theta = acos(abs_d);
      double sin_theta = sin(theta);
      double c1_sign = (d > 0) ? 1 : -1;
      // std::cout << "d:" << d << ",theta:" << theta << std::endl;

      double c0 = sin((1 - t) * theta) / sin_theta;
      double c1 = sin(t * theta) / sin_theta * c1_sign;
      Eigen::Quaterniond qi(c0 * q0.coeffs() + c1 * q.coeffs());
      qi.normalize();

      trans = ti * q1 * qi;
       //std::cout << "qi:" << qi.coeffs() << std::endl;
       //std::cout << "ti:" << trans.matrix() << std::endl;

      return true;
    } else {
      trans = ti * q1;
      //std::cout << "ti:" << trans.matrix() << std::endl;
      return true;
    }
  }

  bool findNewest( Eigen::Isometry3d &trans, ros::Time& stamp) {
    int size = odom_que.size();
    if (size < 1)
      return false;

    Eigen::Vector3d v1;
    auto seek = odom_que.end() -1;
    stamp = seek->header.stamp;
    v1(0) = (seek)->pose.pose.position.x;
    v1(1) = (seek)->pose.pose.position.y;
    v1(2) = (seek)->pose.pose.position.z;

    Eigen::Quaterniond q1;
    q1.x() = (seek)->pose.pose.orientation.x;
    q1.y() = (seek)->pose.pose.orientation.y;
    q1.z() = (seek)->pose.pose.orientation.z;
    q1.w() = (seek)->pose.pose.orientation.w;

    trans = Eigen::Isometry3d::Identity();
    trans.rotate(q1);
    trans.pretranslate(v1);

    //std::cout<<"newest:"<<trans.matrix()<<std::endl;
    return true;
  }

  void odomAftMappedHandler(
    const nav_msgs::Odometry::ConstPtr &odomAftMapped) {
        if(!initialize){
            initialize = true;
        }
    //nav_msgs::Odometry odom_correct, odom_predict;
    kf_mutex.lock();
    odom_correct = *odomAftMapped;
    odom_correct.header.stamp = odomAftMapped->header.stamp;
    //correct(odom_correct, odom_predict);
    kf_mutex.unlock();
    //ROS_INFO("Receive corrected pose");
    //_pubLaserPredect.publish(odom_predict);
  }

  void imuHandler(const sensor_msgs::Imu::ConstPtr &msg) {
    addMsg(*msg);
    nav_msgs::Odometry odom;
    kf_mutex.lock();
    if(predict(odom))
        _pubLaserPredect.publish(odom);
    kf_mutex.unlock();

/*
    que_mutex.lock();
    odom_que.push_back(odom);
    if (odom_que.size() > 100)
      odom_que.erase(odom_que.begin(), odom_que.begin() + 1);
    que_mutex.unlock();
*/

    }

  void addMsg(sensor_msgs::Imu msg) {
    que_mutex.lock();
    imu_que.push_back(msg);
    if (imu_que.size() > que_size)
      imu_que.erase(imu_que.begin(), imu_que.begin() + 1);
    que_mutex.unlock();
  }

 bool predict(nav_msgs::Odometry& odom) {
     if(!initialize)
        return false;

    sensor_msgs::Imu msg;
    int index = 1;
    if(!findNearestIMU(odom_correct.header.stamp, index))
        return false;
    const auto &position = odom_correct.pose.pose.position;
    const auto &orientation = odom_correct.pose.pose.orientation;
    const auto &linear = odom_correct.twist.twist.linear;

    Eigen::Vector3f pos(position.x, position.y, position.z);
    Eigen::Quaternionf quat(orientation.w, orientation.x, orientation.y, orientation.z);
    Eigen::Vector3f vel(linear.x, linear.y, linear.z);


    ros::Time newest_time;
    for(index;index<imu_que.size();index++){
        double dt = imu_que[index].header.stamp.toSec() - imu_que[index-1].header.stamp.toSec();
        imuStep(imu_que[index], dt, pos, quat, vel);
        newest_time = imu_que[index].header.stamp;
    }
    //Isometry2Odom(trans.cast<double>(), odom);
    odom.pose.pose.position.x = pos(0);
    odom.pose.pose.position.y = pos(1);
    odom.pose.pose.position.z = pos(2);
    odom.pose.pose.orientation.x = quat.x();
    odom.pose.pose.orientation.y = quat.y();
    odom.pose.pose.orientation.z = quat.z();
    odom.pose.pose.orientation.w = quat.w();

    odom.twist = odom_correct.twist;
    odom.pose.covariance[0] = odom_correct.pose.covariance[0];
    odom.pose.covariance[7] = odom_correct.pose.covariance[7];
    odom.pose.covariance[14] = odom_correct.pose.covariance[14];
    odom.twist.covariance[0] = odom_correct.twist.covariance[0];
    odom.twist.covariance[7] = odom_correct.twist.covariance[7];
    odom.twist.covariance[14] = odom_correct.twist.covariance[14];

    odom.header.stamp = newest_time;
    odom.header.frame_id = "/map";
    odom.child_frame_id = "/lidar_predict";
    return true;
  }

  bool predict(const sensor_msgs::Imu::ConstPtr &msg,
              nav_msgs::Odometry& odom) {
      const auto &acc = (msg)->linear_acceleration;
      const auto &gyro = (msg)->angular_velocity;
      double gyro_sign = 1.0;
      ukf_pose_estimator->predict(
          (msg)->header.stamp, Eigen::Vector3f(acc.x, acc.y, acc.z),
          gyro_sign * Eigen::Vector3f(gyro.x, gyro.y, gyro.z));

    Eigen::Isometry3f trans;
    trans.matrix() =
        ukf_pose_estimator->matrix() * Tli.inverse().matrix().cast<float>();

    Isometry2Odom(trans.cast<double>(), odom);
    Eigen::Matrix<float, Eigen::Dynamic, 1> VectorXt = getMean();
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> MatrixXt = getCov();
    odom.twist.twist.linear.x = VectorXt(3);
    odom.twist.twist.linear.y = VectorXt(4);
    odom.twist.twist.linear.z = VectorXt(5);
    odom.pose.covariance[0] = MatrixXt(0, 0);
    odom.pose.covariance[7] = MatrixXt(1, 1);
    odom.pose.covariance[14] = MatrixXt(2, 2);
    odom.twist.covariance[0] = MatrixXt(3, 3);
    odom.twist.covariance[7] = MatrixXt(4, 4);
    odom.twist.covariance[14] = MatrixXt(5, 5);

    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = "/map";
    odom.child_frame_id = "/lidar_predict";
    /*
    std::cout << "count:" << count << "\n pos:" << ukf_pose_estimator->pos()
              << "\n vel:" << ukf_pose_estimator->vel()
              << "\n quat:" << ukf_pose_estimator->quat().coeffs() << std::endl;
    */
  }

  bool correct(const nav_msgs::Odometry& odom_correct,
              nav_msgs::Odometry& odom_predict) {
    Eigen::Isometry3d trans_before, trans_after, trans_update;
    ros::Time stamp;

    Eigen::Isometry3d correct_pose;
    Odom2Isometry(odom_correct, correct_pose);
    std::cout<<std::fixed<<"correct_pose:"<<odom_correct.header.stamp.toSec()<<"\n"<<correct_pose.matrix()<<std::endl;
    Eigen::Isometry3f imu_pose = correct_pose.cast<float>() * Tli.cast<float>();
    std::cout<<"imu_pose before:"<<imu_pose.matrix()<<std::endl;

    Eigen::Vector3f velocity;
    velocity(0) = odom_correct.twist.twist.linear.x;
    velocity(1) = odom_correct.twist.twist.linear.y;
    velocity(2) = odom_correct.twist.twist.linear.z;

    Eigen::Vector3f imu_velocity;
    imu_velocity = Tli.inverse().rotation().cast<float>()*velocity;



    Eigen::Isometry3d correctUpdate = lastCorrect.inverse() * correct_pose;
    lastCorrect = correct_pose;
    if(correctUpdate.translation().norm()>5.0)
    {
        Eigen::Quaternionf quat(imu_pose.rotation());
        Eigen::Vector3f pos(imu_pose.translation());
        reset(pos, quat, imu_velocity);
        std::cout<<"kf reset\n";
        return false;
    }
    if(!findNearest(odom_correct.header.stamp, trans_before))
      return false;
    if(!findNewest(trans_after,stamp))
      return false;
    trans_update = trans_before.inverse()*trans_after;
    std::cout<<std::fixed<<"trans_update:"<<stamp.toSec()<<"\n"<<trans_update.matrix()<<std::endl;



    imu_pose = imu_pose * trans_update.cast<float>();
    std::cout<<"imu_pose update:"<<imu_pose.matrix()<<std::endl;

    Eigen::Isometry3d predict_pose;
    predict_pose.matrix() =
    ukf_pose_estimator->matrix().cast<double>();
    std::cout<<"kf_pose before:"<<predict_pose.matrix()<<std::endl;

    ukf_pose_estimator->correct(imu_pose, imu_velocity);

    predict_pose.matrix() =
    ukf_pose_estimator->matrix().cast<double>();
    std::cout<<"kf_pose update:"<<predict_pose.matrix()<<std::endl;

    predict_pose= predict_pose  * Tli.inverse().matrix();
    std::cout<<"predict_pose:"<<predict_pose.matrix()<<std::endl;
    Isometry2Odom(predict_pose, odom_predict);

    Eigen::Matrix<float, Eigen::Dynamic, 1> VectorXt = getMean();
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> MatrixXt = getCov();
    odom_predict.twist.twist.linear.x = VectorXt(3);
    odom_predict.twist.twist.linear.y = VectorXt(4);
    odom_predict.twist.twist.linear.z = VectorXt(5);
    odom_predict.pose.covariance[0] = MatrixXt(0, 0);
    odom_predict.pose.covariance[7] = MatrixXt(1, 1);
    odom_predict.pose.covariance[14] = MatrixXt(2, 2);
    odom_predict.twist.covariance[0] = MatrixXt(3, 3);
    odom_predict.twist.covariance[7] = MatrixXt(4, 4);
    odom_predict.twist.covariance[14] = MatrixXt(5, 5);
    odom_predict.header.stamp = stamp;
    odom_predict.header.frame_id = "/map";
    odom_predict.child_frame_id = "/lidar_predict";
    /*
    Eigen::Quaternionf q(imu_pose.rotation());
    std::cout << "correct:"
              << "\n pos:" << ukf_pose_estimator->pos()
              << "\n vel:" << ukf_pose_estimator->vel()
              << "\n quat:" << ukf_pose_estimator->quat().coeffs() << std::endl;
    */
  }

  void imuStep(const sensor_msgs::Imu imu, double dt, Eigen::Vector3f& pos, Eigen::Quaternionf& quat, Eigen::Vector3f& vel) {
    auto &acc = imu.linear_acceleration;
    auto &gyro = imu.angular_velocity;
    Eigen::Vector3f acc3(acc.x, acc.y, acc.z);
    Eigen::Vector3f gyro3(gyro.x, gyro.y, gyro.z);
    Eigen::Vector3f g(0.0f, 0.0f, -9.80665f);
    Eigen::Vector3f accg = quat * Qli * acc3;
    //Eigen::Vector3f velg = Qli.inverse() * vel;
    //Eigen::Vector3f vel += (accg - g) * dt;		// acceleration didn't contribute to accuracy due to large noise
    pos = pos + vel * dt;					//
    Eigen::Quaternionf dq(1, gyro3[0] * dt / 2, gyro3[1] * dt / 2, gyro3[2] * dt / 2);
    dq.normalize();
    quat = (quat * Qli* dq).normalized();
    quat = quat * Qli.inverse();
  }

  bool reset(const Eigen::Vector3f &pos, const Eigen::Quaternionf &quat, Eigen::Vector3f vel) {
    ukf_pose_estimator->reset(pos, quat, vel);
  }

  const VectorXt &getMean() const { return ukf_pose_estimator->getMean(); }
  const MatrixXt &getCov() const { return ukf_pose_estimator->getCov(); }

private:
  Eigen::Isometry3d Tli;
  std::deque<sensor_msgs::Imu> imu_que;
  std::deque<nav_msgs::Odometry> odom_que;

  nav_msgs::Odometry odom_correct, odom_predict;
  int que_size;
  std::mutex que_mutex;
      std::mutex kf_mutex;
  Eigen::Isometry3d lastCorrect;

  ros::Subscriber subIMU;
  std::unique_ptr<kf::UKFPoseEstimator> ukf_pose_estimator;

  bool initialize;
  Eigen::Quaternionf Qli;

  ros::Publisher _pubLaserPredect; ///< integrated laser odometry publishe

  ros::Subscriber
      _subOdomAftMapped; ///< (low frequency) mapping odometry subscriber

}; //class
}  //namespace lidar_slam
#endif // TRANSFORM_MAINTENANCE
