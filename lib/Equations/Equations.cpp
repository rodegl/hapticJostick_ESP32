#include <Equations.h>

float polynomial(float x, float coeff[], unsigned char num_coeff)
{
    float y = coeff[0] + coeff[1] * x;
    unsigned char order = num_coeff - 1;
    if (order > 2)
    {
        for (unsigned char i = 2; i <= order; i++)
        {
            x *= x;
            if (coeff[i] != 0.0)
                y += coeff[i] * x;
        }
    }
    return y;
}