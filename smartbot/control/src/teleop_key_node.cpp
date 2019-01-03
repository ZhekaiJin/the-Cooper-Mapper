//
// Created by scott on 10/18/18.
//
// Instruction:
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <termios.h>
#include <sys/select.h>
#include <iostream>

struct termios old_attr, new_attr;

int main(int argc, char** argv) {

    ros::init(argc,argv,"keyboard_driver");

    ros::NodeHandle n;

    //notify the master registar
    ros::Publisher publisher = n.advertise<std_msgs::String>("keys", 1); // setting queue size to be 1 for real time controlling

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
        std_msgs::String msg;
        std::string s(1, ch);
        msg.data = s.c_str();
        publisher.publish(msg);
        ros::spinOnce(); //let it call any callback
        loop_rate.sleep();

    }
    tcsetattr(STDIN_FILENO, TCSANOW, &old_attr);
}


