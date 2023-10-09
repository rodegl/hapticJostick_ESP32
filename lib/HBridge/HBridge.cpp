#include "HBrigde.h"



HBridge::HBridge() 
{
    
}

HBridge::~HBridge() 
{
    
}

void HBridge::setup(uint8_t IN1, uint8_t IN2, uint8_t PWM, uint8_t EN, uint8_t CH) 
{
    _IN1 = IN1;
    _IN2 = IN2;
    _EN = EN;
    pwm.setup(PWM, CH, 20000, 10);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(EN, OUTPUT);
}

void HBridge::setSpeed(float speed) 
{
    digitalWrite(_EN, HIGH);

    if (speed > 0)
    {
        setDirection(FORWARD);
    } else {
        setDirection(REVERSE);
    }
    pwm.setDuty(abs(speed));
}

void HBridge::setDirection(bool direction) 
{
    if (direction == FORWARD)
    {
        digitalWrite(_IN1, HIGH);
        digitalWrite(_IN2, LOW);
    } else {
        digitalWrite(_IN1, LOW);
        digitalWrite(_IN2, HIGH);
    }
}