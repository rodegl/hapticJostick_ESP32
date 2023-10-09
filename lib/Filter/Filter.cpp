#include "Filter.h"

Filter::Filter()
{
}

Filter::~Filter()
{
}

void Filter::setup(float a[], float b[], uint8_t size_a, uint8_t size_b)
{
    _size_a = size_a;
    _size_b = size_b;
    _a = a;
    _b = b;
    for (uint8_t i = 0; i < size_b; i++)
    {
        x[i] = 0;
    }
    for (uint8_t i = 0; i < size_a; i++)
    {
        y[i] = 0;
    }
}

float Filter::filtering(float x0)
{

    for (uint8_t i = _size_b; i >= 1; i--)
    {
        x[i] = x[i - 1];
    }

    for (uint8_t i = _size_a; i >= 1; i--)
    {
        y[i] = y[i - 1];
    }

    x[0] = x0;

    y[0] = 0;


    for (uint8_t i = 0; i < _size_b; i++)
    {
        y[0] += _b[i] * x[i];
    }

    for (uint8_t i = 1; i < _size_a; i++)
    {
        y[0] -= _a[i] * y[i];
    }


    return y[0];
}
