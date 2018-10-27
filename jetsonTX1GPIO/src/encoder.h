#include <iostream>
#include <thread>
#include <unistd.h>
#include "jetsonGPIO.h"
#include <std_msgs/Int16.h>
#include <ros/ros.h>
#include <string>
#define PI 3.1415



class Encoder
{
public:
    Encoder(jetsonTX1GPIONumber inA, jetsonTX1GPIONumber inB);
    jetsonTX1GPIONumber inA;
    jetsonTX1GPIONumber inB;
    int count; //A ahead of B is defined as the positive direction
    const int resolution = 64; // the resolution of the encoder, used for calculating the distance
    const float wheelDia = 0.1; // the diamater of the wheel, used for calculating the distance
    float distance;

    void monitorEncoder();
    void publish(const std_msgs::Int16::ConstPtr& msg);
    ros::Publisher pub;
};


