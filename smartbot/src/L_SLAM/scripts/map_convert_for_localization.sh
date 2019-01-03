#!/bin/bash  

echo "this script is for lidar_slam map convert."

if [ $# -lt 1 ]
then echo "Please provide map file."
exit
fi

export LIDAR_DATA=$HOME/lidar_slam
LIDAR_MAP=$LIDAR_DATA/map
LIDAR_LOC=$LIDAR_DATA/loc

echo $LIDAR_MAP;
if [ ! -d $LIDAR_DATA ]; then
  mkdir  $LIDAR_DATA
fi

if [ ! -d $LIDAR_MAP ]; then
  mkdir  $LIDAR_MAP   
fi

rosrun lidar_slam featureExtracter _filesDirectory:=$LIDAR_MAP $@
sleep 2
