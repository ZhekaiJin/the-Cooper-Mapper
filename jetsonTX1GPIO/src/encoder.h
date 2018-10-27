#include <iostream>
#include <thread>
#include <unistd.h>
#include "jetsonGPIO.h"

#define PI 3.1415



class Encoder
{
public:
    Encoder(jetsonGPIONumber inA, jetsonGPIONumber inB);

    jetsonGPIONumber inA;
    jetsonGPIONumber inB;
    int count; //A ahead of B is defined as the positive direction
    const int resolution = 64; // the resolution of the encoder, used for calculating the distance
    const float wheelDia = 0.1; // the diamater of the wheel, used for calculating the distance
    float distance;

    void monitorEncoder();
};


