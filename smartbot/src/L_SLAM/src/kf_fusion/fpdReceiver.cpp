#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <hdmap_msgs/gpfpd.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include "common/math_utils.h"
#include "common/transform_utils.h"
#include "fusion/transPointCLoud.h"
#include "fusion/utmProjection.h"
#include <exception>
#include <iostream>
#include <signal.h>
#include <sstream>
#include <stdlib.h>
#include <vector>
//#include "loadExtrinsic.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "loadExtrinsic.hpp"

using namespace lidar_slam;

class FPDReceiver {
public:
  struct GPFPD {
    int nGPSWeek;
    double dGPSTime;
    double dHeading;
    double dPitch;
    double dRoll;
    double dLattitude;
    double dLongitude;
    double dAltitude;
    double dVEast;
    double dVNorth;
    double dVUp;
    double dBaseline;
    int nSatelitesNum1;
    int nSatelitesNum2;
    char cStatus[2];
    char cReserved[3];
  };

  FPDReceiver(ros::NodeHandle &node, std::string mode_ = "",
              std::string extrinsic_file_ = "") {
    mode = mode_;
    subGPSfpd = node.subscribe<hdmap_msgs::gpfpd>(
        "/sensor/gpfpd", 2, &FPDReceiver::gpfpdHandler, this);
    pubOml = node.advertise<nav_msgs::Odometry>("/fpd", 1);
    Oml.header.frame_id = "/map";
    Oml.child_frame_id = "/velodyne";
    TFml.frame_id_ = "/map";
    TFml.child_frame_id_ = "/velodyne";

    // just for lidar mapping
    pubOml_initMap = node.advertise<nav_msgs::Odometry>("/init_lidar2map", 1);
    Oml_initMap.header.frame_id = "/map";
    Oml_initMap.child_frame_id = "/lidar_init";
    TFml_initMap.frame_id_ = "/map";
    TFml_initMap.child_frame_id_ = "/lidar_init";

    // just for init lidar relocalization module
    pubOml_initLoc = node.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        "/initialpose2", 1);

    _initLocSrv = node.advertiseService("/initLoc", &FPDReceiver::initLoc, this);

    ros::NodeHandle privateNode("~");
    std::string extrinsic_file;

    if (extrinsic_file_ == "") {
      if (privateNode.getParam("extrinsic_file", extrinsic_file)) {
        ROS_INFO_STREAM("Set extrinsic file:" << extrinsic_file);
      } else {
        ROS_ERROR("NO extrinsic file setup.");
      }
    } else {
      extrinsic_file = extrinsic_file_;
    }

    if (loadExtrinsic(extrinsic_file, Tli)) {
      ROS_INFO_STREAM("imu2lidar calibration Tli:\n"
                      << Tli.matrix() << std::endl);
    } else {
      ROS_ERROR_STREAM("Error in " << extrinsic_file);
    }

    privateNode.param("latitude", _latitude, 39.86966052);
    privateNode.param("longitude", _longitude, 116.17656364);
    privateNode.param("altitude", _altitude, 62.41314815);
    ROS_INFO("Map origin, latitude:%.8f, longitude:%.8f, altitude:%.4f",_latitude,_longitude,_altitude);
    // map origin in world frame
    wgs2utm_proj4(_longitude, _latitude, _altitude, Vwm_offset[0],
                  Vwm_offset[1], Vwm_offset[2]);
    ROS_INFO_STREAM("Map origin utm:\n" << std::fixed << Vwm_offset);
    initialize = false;
  }

  bool initLoc(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
    geometry_msgs::PoseWithCovarianceStamped Oml_initLoc;
    Oml_initLoc.header.stamp = Oml.header.stamp;
    Oml_initLoc.header.frame_id = "map";
    Oml_initLoc.pose.pose.orientation.x = Oml.pose.pose.orientation.x;
    Oml_initLoc.pose.pose.orientation.y = Oml.pose.pose.orientation.y;
    Oml_initLoc.pose.pose.orientation.z = Oml.pose.pose.orientation.z;
    Oml_initLoc.pose.pose.orientation.w = Oml.pose.pose.orientation.w;
    Oml_initLoc.pose.pose.position.x = Oml.pose.pose.position.x;
    Oml_initLoc.pose.pose.position.y = Oml.pose.pose.position.y;
    Oml_initLoc.pose.pose.position.z = Oml.pose.pose.position.z;
    pubOml_initLoc.publish(Oml_initLoc);
    return true;
  }

  void gpfpdHandler(hdmap_msgs::gpfpd fpd) {

    Eigen::Vector3d utm_pos;
    wgs2utm_proj4(fpd.dLongitude, fpd.dLattitude, fpd.dAltitude, utm_pos[0],
                  utm_pos[1], utm_pos[2]);

    // std::cout << "utm_pos:" << utm_pos << std::endl;
    std::vector<double> pos_data;
    pos_data.push_back(utm_pos[1]);
    pos_data.push_back(utm_pos[0]);
    pos_data.push_back(utm_pos[2]);
    pos_data.push_back(fpd.dRoll);
    pos_data.push_back(fpd.dPitch);
    pos_data.push_back(fpd.dHeading);

    Eigen::Matrix4Xd Twi;
    Eigen::Matrix4Xd Twi_inv;
    transMatrixContruct(pos_data, Twi, Twi_inv);
    // std::cout << std::fixed << "Twi:" << Twi << std::endl;
    Eigen::Matrix4Xd Tmi = Twi;
    Tmi.block(0, 3, 3, 1) -= Vwm_offset;
    // std::cout << std::fixed << "Tmi:" << Tmi << std::endl;

    Eigen::Matrix4Xd Tml = Tmi * Tli.inverse().matrix();
    // std::cout << std::fixed << "Tml:" << Tml << std::endl;

    // just for init lidar relocalization module
    if (!initialize && mode == "loc") {
      geometry_msgs::PoseWithCovarianceStamped Oml_initLoc;

      Eigen::Matrix4Xd l_odom = Tml.matrix();
      Eigen::Matrix3d Rml = l_odom.block(0, 0, 3, 3);
      Eigen::Vector3d Vml = l_odom.block(0, 3, 3, 1);
      Eigen::Quaterniond Qml(Rml);

      Oml_initLoc.header.stamp = fpd.header.stamp;
      Oml_initLoc.header.frame_id = "map";
      Oml_initLoc.pose.pose.orientation.x = Qml.x();
      Oml_initLoc.pose.pose.orientation.y = Qml.y();
      Oml_initLoc.pose.pose.orientation.z = Qml.z();
      Oml_initLoc.pose.pose.orientation.w = Qml.w();
      Oml_initLoc.pose.pose.position.x = Vml(0);
      Oml_initLoc.pose.pose.position.y = Vml(1);
      Oml_initLoc.pose.pose.position.z = Vml(2);
      pubOml_initLoc.publish(Oml_initLoc);
    }

    Eigen::Matrix3d Rml = Tml.block(0, 0, 3, 3);
    Eigen::Vector3d Vml = Tml.block(0, 3, 3, 1);
    // std::cout << std::fixed << "Vml:" << Vml << std::endl;

    Eigen::Quaterniond Qml(Rml);
    // std::cout << std::fixed << "Qml:" << Qml.w() << "    " << Qml.x() << " "
    //          << Qml.y() << "    " << Qml.z() << std::endl;
    if (!initialize) {
      Vml_init = Vml;
      Qml_init = Qml;
      initialize = true;
      // std::cout << "Qml_init:" << Qml_init.coeffs() << std::endl;
    }

    Oml.header.stamp = fpd.header.stamp;
    Oml.pose.pose.orientation.x = Qml.x();
    Oml.pose.pose.orientation.y = Qml.y();
    Oml.pose.pose.orientation.z = Qml.z();
    Oml.pose.pose.orientation.w = Qml.w();
    Oml.pose.pose.position.x = Vml(0);
    Oml.pose.pose.position.y = Vml(1);
    Oml.pose.pose.position.z = Vml(2);
    Oml.twist.twist.linear.x = fpd.dVEast;
    Oml.twist.twist.linear.y = fpd.dVNorth;
    Oml.twist.twist.linear.z = fpd.dVUp;
    // printf("/fpd global position: (x, y, z) = (%.3f %.3f %.3f)\n",
    // std::round(Vml(0) / 50.0), std::round(Vml(1) / 50.0), std::round(Vml(2) /
    // 50.0));
    pubOml.publish(Oml);

    TFml.stamp_ = fpd.header.stamp;
    TFml.setRotation(tf::Quaternion(Qml.x(), Qml.y(), Qml.z(), Qml.w()));
    TFml.setOrigin(tf::Vector3(Vml[0], Vml[1], Vml[2]));
    _tfBroadcaster.sendTransform(TFml);

    if (mode == "map") {
      Oml_initMap.header.stamp = fpd.header.stamp;
      Oml_initMap.pose.pose.orientation.x = Qml_init.x();
      Oml_initMap.pose.pose.orientation.y = Qml_init.y();
      Oml_initMap.pose.pose.orientation.z = Qml_init.z();
      Oml_initMap.pose.pose.orientation.w = Qml_init.w();
      Oml_initMap.pose.pose.position.x = Vml_init(0);
      Oml_initMap.pose.pose.position.y = Vml_init(1);
      Oml_initMap.pose.pose.position.z = Vml_init(2);
      Oml_initMap.twist.twist.linear.x = fpd.dVEast;
      Oml_initMap.twist.twist.linear.y = fpd.dVNorth;
      Oml_initMap.twist.twist.linear.z = fpd.dVUp;
      pubOml_initMap.publish(Oml_initMap);
      TFml_initMap.stamp_ = fpd.header.stamp;
      TFml_initMap.setRotation(tf::Quaternion(Qml_init.x(), Qml_init.y(),
                                              Qml_init.z(), Qml_init.w()));
      TFml_initMap.setOrigin(
          tf::Vector3(Vml_init(0), Vml_init(1), Vml_init(2)));
      _tfBroadcaster.sendTransform(TFml_initMap);
    }
  }

private:
  std::string mode;

  // ros something
  ros::Subscriber subGPSfpd;
  tf::TransformBroadcaster _tfBroadcaster;
  ros::Publisher pubOml;
  nav_msgs::Odometry Oml;
  tf::StampedTransform TFml;

  ros::Publisher pubOml_initMap;
  nav_msgs::Odometry Oml_initMap;
  tf::StampedTransform TFml_initMap;

  ros::Publisher pubOml_initLoc;
  nav_msgs::Odometry Oml_initLoc;
  tf::StampedTransform TFml_init;

  ros::ServiceServer _initLocSrv;

  //
  bool initialize;
  Eigen::Vector3d Vml_init;
  Eigen::Quaterniond Qml_init;

  Eigen::Vector3d Vwm_offset;
  Eigen::Isometry3d Tli;

  double _latitude;
  double _longitude;
  double _altitude;
};

int main(int argc, char **argv) {

  ros::init(argc, argv, "gpfpd_receiver");
  ros::NodeHandle n;

  std::string mode;
  if (argc <= 1) {
    ROS_INFO("Run in default mode, option: map|loc");
  } else if (argc == 2) {
    mode = argv[1];
    ROS_INFO("Run in mode: %s", mode.c_str());
    FPDReceiver fpd_receiver(n, mode);
    ros::spin();
  } else if (argc == 3) {
    mode = argv[1];
    ROS_INFO("Run in mode: %s", mode.c_str());
    std::string extrinsic_file = argv[2];
    FPDReceiver fpd_receiver(n, mode, extrinsic_file);
    ros::spin();
  }

  return 0;
}