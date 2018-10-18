//
// Created by scott on 10/14/18.
// Simple and Dumb Controller Node, For Environment testing and tutorial purpose 
//

#include "../include/smartbot/random_controller.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

#include <sstream>

int main(int argc, char** argv) {

    ros::init(argc,argv,"action_publisher");

    ros::NodeHandle n;

    //notify the master registar
    ros::Publisher publisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1); // setting queue size to be 1 for real time controlling

    ros::Rate loop_rate(10); //ok we will run this at 10 HZ and this will only be in effect with the call "sleep"

    geometry_msgs::Twist msg_go;
    msg_go.linear.x = 0.5;
    geometry_msgs::Twist msg_nogo;

    int count = 0;

    bool go_or_nogo = false;

    ros::Time timer = ros::Time::now();

    while (ros::ok()) {
        
        if (go_or_nogo) {
            publisher.publish(msg_go);
            ROS_INFO("sending msg_go");
        } else {
            publisher.publish(msg_nogo);
            ROS_INFO("sending msg_nogo");
        }

        if (timer < ros::Time::now()) {
            go_or_nogo = !go_or_nogo;
            timer = ros::Time::now() + ros::Duration(3.0);
        }

        ros::spinOnce();

        loop_rate.sleep();
        count++;

    }
}


