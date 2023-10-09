#include "AnalogSensor.h"


AnalogSensor::AnalogSensor() 
{
    
}

AnalogSensor::~AnalogSensor() 
{
    
}

void AnalogSensor::setup(const uint8_t pin, float(*equation)(float,float[],uint8_t), float coeff[], uint8_t num_coeff) 
{
    _pin = pin;
    _equation = equation;
    _coeff = coeff;
    _num_coeff = num_coeff;
    pinMode(_pin, INPUT);
}

float AnalogSensor::read() 
{
    _digital_level = analogRead(_pin);
    _sensor_measurament = _equation ((float)analogRead(_pin), _coeff, _num_coeff);
    return _sensor_measurament;
}

float AnalogSensor::getMeasurament(measurament type) 
{
    switch (type)
    {
    case DIGITAL_LEVEL:
        return _digital_level;
        break;

    case SENSOR_MEASURAMENT:
        return _sensor_measurament;
        break;
    
    default:
        return _digital_level;
        break;
    }
}




