//
// Created by scott on 10/14/18.
//
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

//refer to sensor_msg/LaserScan
void rangereporterCallback (const sensor_msgs::LaserScan::ConstPtr & msg) {
    //just trying to connect
    float distance =  msg->ranges[msg->ranges.size() / 2];
    std::cout.precision(8);
    std::cout << "range reading:" << std::setw(10) << distance << std::endl;
}

// some math facts to fill in
// bearing = msg->angle_min + i * msg->angle_max / msg->ranges.size();
// distance =  msg->ranges[msg->ranges.size() / 2];


int main (int argc, char** argv) {
    ros::init(argc, argv, "range_reporter");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("scan", 1000, rangereporterCallback);

    ros::spin();

}
