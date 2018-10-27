#include "encoder.h"

void monitorEncoder(encoder *encoder)
{
    unsigned int *Avalue;
    unsigned int *Bvalue;
    gpioExport(inA);
    gpioExport(inB);
    gpioSetDirection(inA, inputPin);
    gpioSetDirection(inA, inputPin);
    while(true)
    {
        gpioGetValue(inA, Avalue);
        gpioGetValue(inB, Bvalue);
        if(*Avalue == high && *Bvalue == low) //detect the moment when A is high and B is low
        {
            do
            {
                gpioGetValue(inA, Avalue);
                gpioGetValue(inB, Bvalue);
            }
            while(*Avalue != *Bvalue); //wait until A or B changes

            if(*Avalue == 1) //if A keeps on 1, that means B turns into 1, and thus A is ahead of B
            {
                encoder->count++;
                encoder->distance += encoder->wheelDia*PI/encoder->resolution;
            }
            else //if A turns into 0, then B is ahead of A
            {
                encoder->count--;
                encoder->distance -= encoder->wheelDia*PI/encoder->resolution;
            }
        }
    }
}