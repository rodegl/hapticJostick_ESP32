#ifndef _FILTER_H
#define _FILTER_H

#include <Arduino.h>

class Filter
{
    public:
        Filter();
        ~Filter();
        void setup(float a[], float b[], uint8_t size_a, uint8_t size_b);
        float filtering (float x0);

    private:
        float y[6];
        float x[6];
        uint8_t _size_a;
        uint8_t _size_b;
        float *_a;
        float *_b;
        

};

#endif // _FILTER_H