#ifndef _ANALOGSENSOR_H
#define _ANALOGSENSOR_H

#include <Arduino.h>

enum measurament
{
    DIGITAL_LEVEL,
    SENSOR_MEASURAMENT
};

class AnalogSensor
{
    public:
        AnalogSensor();
        ~AnalogSensor();
        void setup(const uint8_t pin, float(*equation)(float,float[],uint8_t), float coeff[], uint8_t num_coeff);
        float read();
        float getMeasurament(measurament type = DIGITAL_LEVEL);

    private:
        uint8_t _pin;
        float (*_equation)(float, float[], uint8_t);
        float *_coeff;
        uint8_t _num_coeff;

    protected:   
        uint16_t _digital_level;
        float _sensor_measurament;
};

#endif // _ANALOGSENSOR_H
