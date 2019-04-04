//
// Created by scott on 10/18/18.
//
// Instruction:
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include <termios.h>
#include <sys/select.h>
#include <iostream>

struct termios old_attr, new_attr;

int main(int argc, char** argv) {

    ros::init(argc,argv,"keyboard_driver");

    ros::NodeHandle n;

    //notify the master registar
    ros::Publisher publisher = n.advertise<geometry_msgs::Twist>("twist", 1); // setting queue size to be 1 for real time controlling

    ros::Rate loop_rate(10); //ok we will run this at 10 HZ and this will only be in effect with the call "sleep"

    if( tcgetattr (STDIN_FILENO, &old_attr) < 0) {
        std::cerr <<  "Failed to get serial configuration" << std::endl;
        exit(-1);
    }
    new_attr = old_attr;
    new_attr.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
    new_attr.c_cflag &= ~(CSIZE | PARENB);

    new_attr.c_cc[VMIN] = 1;
    new_attr.c_cc[VTIME] = 0;

    if(cfsetispeed(&new_attr, B9600) < 0 || cfsetospeed(&new_attr, B9600) < 0) {
        std::cerr <<  "Failed to set serial comm speed" << std::endl;
        exit(-1);
    }

    if(tcsetattr(STDIN_FILENO, TCSANOW, &new_attr) < 0) {
        std::cerr <<  "Failed to set serial configuration" << std::endl;
        exit(-1);
    }

    std::cout << "publishing key strokes now" << std::endl;
    int ch;
    while (ros::ok()) {
		ROS_INFO("waiting for character\n");
        ch = getchar();
        geometry_msgs::Twist msg;
	float lin_speed = 0.15;
	float rot_speed = 1.2;
	float rot_lin_speed = 0;
        if (ch == 'q'){
			ROS_INFO("Shutting Down...\n");
			ros::shutdown();
        } else if (ch == 'w'){
			msg.linear.x = lin_speed;
			msg.linear.y = 0;//unit? //axis?
			msg.linear.z = 0;
			msg.angular.x = 0;
			msg.angular.y = 0;//unit? //axis?
			msg.angular.z = 0;
			
		} else if (ch == 'a'){
			msg.linear.x = rot_lin_speed;
			msg.linear.y = 0;//unit? //axis?
			msg.linear.z = 0;
			msg.angular.x = 0;
			msg.angular.y = 0;//unit? //axis?
			msg.angular.z = -rot_speed;
			
		} else if (ch == 'd'){
			msg.linear.x = rot_lin_speed;
			msg.linear.y = 0;//unit? //axis?
			msg.linear.z = 0;
			msg.angular.x = 0;
			msg.angular.y = 0;//unit? //axis?
			msg.angular.z = rot_speed;
			
		} else if (ch == 's'){
			msg.linear.x = -lin_speed;
			msg.linear.y = 0;//unit? //axis?
			msg.linear.z = 0;
			msg.angular.x = 0;
			msg.angular.y = 0;//unit? //axis?
			msg.angular.z = 0;
			
		} else {
			msg.linear.x = 0;
			msg.linear.y = 0;//unit? //axis?
			msg.linear.z = 0;
			msg.angular.x = 0;
			msg.angular.y = 0;//unit? //axis?
			msg.angular.z = 0;
		}
        publisher.publish(msg);
        ros::spinOnce(); //let it call any callback
        loop_rate.sleep();

    }
    tcsetattr(STDIN_FILENO, TCSANOW, &old_attr);
}


