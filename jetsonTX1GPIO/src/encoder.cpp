#include "encoder.h"

Encoder::Encoder(jetsonTX1GPIONumber inA, jetsonTX1GPIONumber inB){
	//xmap["pin32"] = pin32; 
	//xmap["pin13"] = pin13;
	//xmap ["pin7"] = pin7;
	this->inA = inA;
    	this->inB = inB;
	gpioExport(inA);
	gpioExport(inB);
}

//TODO:use SetEdge method instead
void Encoder::monitorEncoder()
{
    unsigned int *Avalue;
    unsigned int *Bvalue;
    gpioSetDirection(inA, inputPin);
    gpioSetDirection(inB, inputPin);
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
            this->count++;
            this->distance += this->wheelDia*PI/this->resolution;
        }
        else //if A turns into 0, then B is ahead of A
        {
		this->count--;
                this->distance -= this->wheelDia*PI/this->resolution;
        }
    }
}

