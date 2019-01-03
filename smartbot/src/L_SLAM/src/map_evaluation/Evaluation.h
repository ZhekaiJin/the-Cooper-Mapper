#include <ros/ros.h>

#include "nav_msgs/Odometry.h"
#include <thread>


namespace lidar_slam {

const int GpsStoreNum = 1000;
const int LidarNum = 100000;

class Evaluation {
public:
  Evaluation();
  ~Evaluation();
  bool init(ros::NodeHandle &node, ros::NodeHandle &privateNode);
  void gpsHandler(const nav_msgs::Odometry::ConstPtr &gpsMsg);
  void lidarToMapHandler(const nav_msgs::Odometry::ConstPtr &lidarToMapMsg);
  void spin();
  void process();

private:
  ros::Subscriber _subGps;
  ros::Subscriber _subLidarToMap;

  double _gpsX[GpsStoreNum];
  double _gpsY[GpsStoreNum];
  double _gpsZ[GpsStoreNum];
  double _gpsTime[GpsStoreNum];
  int _gpsNowId;
  int _gpsMsgNum;

  int _lidarMsgNum;
  double _differX[LidarNum];
  double _differY[LidarNum];
  double _differZ[LidarNum];
  double _differDis[LidarNum];
  double _differTime[LidarNum];
  
  double _averageDifferX;
  double _averageDifferY;
  double _averageDifferZ;
  double _averageDifferDis;
  double _averageDifferTime;

  double _varianceDifferX;
  double _varianceDifferY;
  double _varianceDifferZ;
  double _varianceDifferDis;

  double _maxDifferX;
  double _maxDifferY;
  double _maxDifferZ;
  double _maxDifferDis;

  double _nowTime;

  std::thread _spinThread;
  
};
}