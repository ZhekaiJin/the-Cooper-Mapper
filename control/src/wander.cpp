//
// Created by scott on 10/14/18.
//

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <algorithm>
#include <iostream>

//use a global variable here since there is no class or anything just for demo
float distance_to_go = 1.00;

void rangereporterCallback (const sensor_msgs::LaserScan::ConstPtr & msg) {
    //just trying to connect
    float min_distance =  *std::min_element(msg->ranges.begin(), msg->ranges.end());
    ::distance_to_go = min_distance;
    std::cout.precision(8);
    std::cout << "going forward with:" << std::setw(10) << min_distance << std::endl;
}



int main(int argc, char** argv) {

    ros::init(argc,argv,"wander_bot");

    ros::NodeHandle n;

    //notify the master registar
    ros::Publisher publisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1); // setting queue size to be 1 for real time controlling
    ros::Subscriber sub = n.subscribe("scan", 1000, rangereporterCallback);

    ros::Rate loop_rate(10);

    int count = 0;
    bool go_or_nogo = true;

    ros::Time timer = ros::Time::now();

    while (ros::ok()) {

        if (go_or_nogo) {
            //EITHER YOU RUN INTO SOMETHING OR ITS TIME TO CHANGE DIRECTION
            if (distance_to_go < 0.4 || ros::Time::now() > timer) {
                go_or_nogo = false;
                timer = ros::Time::now() + ros::Duration(1.0);
                ROS_INFO("sending msg_go");
            }
        } else { //turning
            if (ros::Time::now() > timer) {
                go_or_nogo = true;
                timer = ros::Time::now() + ros::Duration(1.0);
                ROS_INFO("spining");
            }
        }

        geometry_msgs::Twist msg;
        if (go_or_nogo) {
            msg.linear.x = 1;
        } else {
            msg.angular.z = 1;
        }
        publisher.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        count++;

    }
}
