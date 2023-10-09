#ifndef _ESP32_PWM_H
#define _ESP32_PWM_H

#include <Arduino.h>

class PWM
{
public:
    PWM();
    ~PWM();
    void setup(uint8_t pin, uint8_t channel, double frequency = 1000, uint8_t bits_resolution = 10, bool on_state = HIGH, float on_voltage = 3.3f);
    void attachPin(uint8_t pin, bool on_state = HIGH);
    void detachPin(uint8_t pin);
    void setDuty(float duty_cycle);
    void setDigitalLevel(uint32_t level);
    void setDAC(float voltage);
    void setFrequency(float frequency);
    void setNote(note_t note, uint8_t octave);

protected:
    uint8_t _pin;
    uint8_t _channel;
    uint32_t _digital_range;
    float _on_voltage;
    float _duty_cycle;
    float _frequency;
};

#endif // _ESP32_PWM_H