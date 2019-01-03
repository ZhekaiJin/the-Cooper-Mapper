#include "ros/ros.h"
#include <hdmap_msgs/gpfpd.h>
#include <nav_msgs/Odometry.h>
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

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace lidar_slam;

class OdomFPDQueue {
public:
  OdomFPDQueue() : que_size(1000) { initialize = false; }

  void setup(ros::NodeHandle &node) {

    subFPD = node.subscribe<nav_msgs::Odometry>(
        "/fpd", 2, &OdomFPDQueue::odomFPDHandler, this);


  }


  void odomFPDHandler(const nav_msgs::Odometry::ConstPtr &msg) { addMsg(*msg); }

  void addMsg(nav_msgs::Odometry msg) {
    que_mutex.lock();
    fpd_que.push_back(msg);
    if (fpd_que.size() > que_size)
      fpd_que.erase(fpd_que.begin(), fpd_que.begin() + 1);
    que_mutex.unlock();
  }

  bool findNearest(ros::Time &stamp, Eigen::Isometry3d &trans) {
    que_mutex.lock();

    // std::cout << "size:" << fpd_que.size() << std::endl;
    int size = fpd_que.size();
    if (size < 1)
      return false;

    auto seek = std::lower_bound(
        fpd_que.begin(), fpd_que.end(), stamp,
        [&](const nav_msgs::Odometry &msg, const ros::Time &stamp) {
          return msg.header.stamp < stamp;
        });

    if (seek == fpd_que.end()) {
      std::cout << "fpd msg slow than the time:" << std::endl;
      std::cout << std::fixed << stamp << "\n"
                << (seek-1)->header.stamp << std::endl;
      return false;
    }

    if (seek == fpd_que.begin()) {
      std::cout << "fpd msg very faster than the time" << std::endl;
      std::cout << std::fixed << stamp << "\n"
                << seek->header.stamp << std::endl;

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
      // std::cout << "qi:" << qi.coeffs() << std::endl;
      // std::cout << "ti:" << trans.matrix() << std::endl;

      return true;
    } else {
      trans = ti * q1;
      // std::cout << "ti:" << trans.matrix() << std::endl;
      return true;
    }
  }

private:
  std::deque<nav_msgs::Odometry> fpd_que;
  int que_size;
  std::mutex que_mutex;

  ros::Subscriber subFPD;


  tf::StampedTransform odomTF;
  tf::StampedTransform initTF;

  bool initialize;
};
