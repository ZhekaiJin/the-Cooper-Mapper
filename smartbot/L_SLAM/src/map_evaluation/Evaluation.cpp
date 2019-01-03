#include "Evaluation.h"
#include <math.h>

namespace lidar_slam {

Evaluation::Evaluation() {}

Evaluation::~Evaluation() {}

bool Evaluation::init(ros::NodeHandle &node, 
                      ros::NodeHandle &privateNode) 
{
  _subGps =  node.subscribe<nav_msgs::Odometry>("/fpd", 5, &Evaluation::gpsHandler, this);
  _subLidarToMap = node.subscribe<nav_msgs::Odometry>("/lidar_to_map", 5, &Evaluation::lidarToMapHandler, this);
  _gpsNowId = 0;
  _gpsMsgNum = 0;
  _lidarMsgNum = 0;
  _averageDifferX = _averageDifferY = _averageDifferZ = _averageDifferDis = 0;
  _varianceDifferX = _varianceDifferY = _varianceDifferZ = _varianceDifferDis = 0;
  _maxDifferX = _maxDifferY = _maxDifferZ = _maxDifferDis = 0;
  
  _spinThread = std::thread(&Evaluation::spin, this);

  return true;
}

void Evaluation::gpsHandler(const nav_msgs::Odometry::ConstPtr &gpsMsg)
{
  _gpsX[_gpsNowId] = gpsMsg->pose.pose.position.x;
  _gpsY[_gpsNowId] = gpsMsg->pose.pose.position.y;
  _gpsZ[_gpsNowId] = gpsMsg->pose.pose.position.z;
  _gpsTime[_gpsNowId] = gpsMsg->header.stamp.toSec();

  _gpsNowId = (_gpsNowId + 1) % GpsStoreNum;
  _gpsMsgNum ++;
  if (_gpsMsgNum > GpsStoreNum) _gpsMsgNum = GpsStoreNum;
}

void Evaluation::lidarToMapHandler(const nav_msgs::Odometry::ConstPtr &lidarToMapMsg)
{
  _nowTime = lidarToMapMsg->header.stamp.toSec();
  _differTime[_lidarMsgNum] = -1;
  int tmpId, corId, i;
  for (tmpId = (_gpsNowId + GpsStoreNum - 1) % GpsStoreNum, i = 0; i < _gpsMsgNum; i ++, tmpId = (tmpId + GpsStoreNum - 1) % GpsStoreNum)
  {
    if (i == 0 || _differTime[_lidarMsgNum] > fabs(_nowTime - _gpsTime[tmpId])) _differTime[_lidarMsgNum] = fabs(_nowTime - _gpsTime[tmpId]), corId = tmpId;
    else
    { 
      break;
    }
  } 
  
  _differX[_lidarMsgNum] = fabs(lidarToMapMsg->pose.pose.position.x - _gpsX[corId]);
  _differY[_lidarMsgNum] = fabs(lidarToMapMsg->pose.pose.position.y - _gpsY[corId]);
  _differZ[_lidarMsgNum] = fabs(lidarToMapMsg->pose.pose.position.z - _gpsZ[corId]);
  _differDis[_lidarMsgNum] = sqrt(_differX[_lidarMsgNum] * _differX[_lidarMsgNum] 
                                  + _differY[_lidarMsgNum] * _differY[_lidarMsgNum] 
                                  + _differZ[_lidarMsgNum] * _differZ[_lidarMsgNum]);
  _maxDifferX = std::max(_maxDifferX, _differX[_lidarMsgNum]);
  _maxDifferY = std::max(_maxDifferY, _differY[_lidarMsgNum]);
  _maxDifferZ = std::max(_maxDifferZ, _differZ[_lidarMsgNum]);
  _maxDifferDis = std::max(_maxDifferDis, _differDis[_lidarMsgNum]);
  
  if(_differDis[_lidarMsgNum] <= 10) 
    _lidarMsgNum ++;
  else // 误差过大，应该是没初始化好导致的
    printf("Not initialized\n");

  // std::cout << "Average Different X:         " << _averageDifferX  << "\n"
  //           << "Average Different Y:         " << _averageDifferY  << "\n"
  //           << "Average Different Z:         " << _averageDifferZ  << "\n"
  //           << "Average Different Distance:  " << _averageDifferDis << "\n"
  //           << "Variance Different X:        " << _varianceDifferX << "\n"
  //           << "Variance Different Y:        " << _varianceDifferY << "\n"
  //           << "Variance Different Z:        " << _varianceDifferZ << "\n"
  //           << "Variance Different Distance: " << _varianceDifferDis << "\n";
  
  if(_lidarMsgNum > LidarNum) printf("Lidars' number outrange!");
}

void Evaluation::spin()
{
  ros::Rate rate(20);
  bool status = ros::ok();

  while(status)
  {
    ros::spinOnce();
    process();
    status = ros::ok();
    rate.sleep();
  }
}

void Evaluation::process()
{
  int num = _lidarMsgNum;
  double nowTime = _nowTime;
  if(num && num % 1000 == 0)
  {
    _averageDifferX = _averageDifferY = _averageDifferZ = _averageDifferDis = _averageDifferTime = 0;
    _varianceDifferX = _varianceDifferY = _varianceDifferZ = _varianceDifferDis = 0;
    for(int i = num - 1000; i < num; i ++)
    {
      
    }
    for(int i = 0; i < num; i ++)
    {
      _averageDifferX += _differX[i];
      _averageDifferY += _differY[i];
      _averageDifferZ += _differZ[i];
      _averageDifferDis += _differDis[i];
      _averageDifferTime += _differTime[i];
    }
    _averageDifferX /= num;
    _averageDifferY /= num;
    _averageDifferZ /= num;
    _averageDifferDis /= num;
    _averageDifferTime /= num;

    for(int i = 0; i < num;  i++)
    {
      _varianceDifferX += (_differX[i] - _averageDifferX) * (_differX[i] - _averageDifferX);
      _varianceDifferY += (_differY[i] - _averageDifferY) * (_differY[i] - _averageDifferY);
      _varianceDifferZ += (_differZ[i] - _averageDifferZ) * (_differZ[i] - _averageDifferZ);
      _varianceDifferDis += (_differDis[i] - _averageDifferDis) * (_differDis[i] - _averageDifferDis);
    }
    _varianceDifferX /= (num - 1);
    _varianceDifferY /= (num - 1);
    _varianceDifferZ /= (num - 1);
    _varianceDifferDis /= (num - 1);

    std::cout << "Lidar Time:                  " << nowTime << "\n"
              << "Average Different Time:      " << _averageDifferTime << "\n"
              << "Average Different X:         " << _averageDifferX  << "\n"
              << "Average Different Y:         " << _averageDifferY  << "\n"
              << "Average Different Z:         " << _averageDifferZ  << "\n"
              << "Average Different Distance:  " << _averageDifferDis << "\n"
              << "Variance Different X:        " << _varianceDifferX << "\n"
              << "Variance Different Y:        " << _varianceDifferY << "\n"
              << "Variance Different Z:        " << _varianceDifferZ << "\n"
              << "Variance Different Distance: " << _varianceDifferDis << "\n"
              << "Max Different X:             " << _maxDifferX << "\n"
              << "Max Different Y:             " << _maxDifferY << "\n"
              << "Max Different Z:             " << _maxDifferZ << "\n"
              << "Max Different Distance:      " << _maxDifferDis << "\n";
  }
}

}