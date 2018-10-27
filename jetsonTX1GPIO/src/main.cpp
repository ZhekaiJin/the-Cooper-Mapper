
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include "encoder.h"
#include "jetsonGPIO.c"
int main(int argc, char** argv) {
	ros::init(argc,argv,"gpio_encoder");
	if (argc != 4 ) return 1;// 1: l or r 2: pinA 3: pinB
	ros::NodeHandle n;
	//TODO: remap string to jetsonTX1GPIONumber
	jetsonTX1GPIONumber A = pin13;
	jetsonTX1GPIONumber B = pin33;
	Encoder enc_handle(A,B);
	enc_handle.pub = n.advertise<std_msgs::Int16>(strcat(argv[1] ,"wheel"), 1);
	ros::Rate loop_rate(300);
	while (ros::ok()) {
		enc_handle.monitorEncoder();
		std_msgs::Int16 msg;
		msg.data = enc_handle.count;
		enc_handle.pub.publish(msg);
		ros::spinOnce();
	}
	return 0;
}
