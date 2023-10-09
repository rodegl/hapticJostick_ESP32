#ifndef __HBRIGDE_H__
#define __HBRIGDE_H__

#include <Arduino.h>
#include <ESP32_PWM.h>

#define FORWARD true
#define REVERSE false

class HBridge{

    public:
        HBridge();
        ~HBridge();
        void setup(uint8_t IN1, uint8_t IN2, uint8_t PWM, uint8_t EN, uint8_t CH);
        void setSpeed(float speed);
        void setDirection(bool direction);
        

    private:
        PWM pwm;
        uint8_t _IN1;
        uint8_t _IN2;
        uint8_t _EN;
};




#endif // __HBRIGDE_H__