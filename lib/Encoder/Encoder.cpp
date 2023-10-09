#include "Encoder.h"


Encoder::Encoder() 
{
    
}

Encoder::~Encoder() 
{
    
}


void Encoder::setup(uint8_t chA, uint8_t chB, float degrees) 
{
    pinA = chA;
    pinB = chB;
    _angle = degrees;
    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);
    counts = 0;
    prev_Time = micros();
    prev_counts = counts;
    counter = 0;
    
}

void Encoder::handler() 
{   
    counter += 1;
    counts += 1;
    flag = false;
    current_time = micros();
    /* if (counter == 8)
    {
        speed = ((_angle*8) / (current_time - prev_Time)) * 1000000;
        prev_Time = current_time;
        counter = 0;
    } */

    current_counts = counts;
    num_counts = current_counts - prev_counts;

    if (current_time - prev_Time > 10000)
    {
        
        if (num_counts < 0)
        {
            speed = 0;
        } else {
            speed = (num_counts*1000)-2000;
        }

        num_counts = 0;
        prev_Time = current_time;
        prev_counts = current_counts;
    }
    
    
}






