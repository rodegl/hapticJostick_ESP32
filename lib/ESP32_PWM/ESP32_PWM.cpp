#include "ESP32_PWM.h"

PWM::PWM()
{
}

PWM::~PWM()
{
    detachPin(_pin);
}

void PWM::setup(uint8_t pin, uint8_t channel, double frequency, uint8_t bits_resolution, bool on_state, float on_voltage)
{
    _pin = pin;
    _channel = channel;
    _frequency = frequency;
    _digital_range = pow(2, bits_resolution) - 1;
    _on_voltage = on_voltage;
    ledcSetup(_channel, _frequency, bits_resolution);
    attachPin(_pin, on_state);
    setDuty(0);
}

void PWM::attachPin(uint8_t pin, bool on_state)
{
    pinMode(pin, OUTPUT);
    ledcAttachPin(pin, _channel);
    if (on_state == LOW)
        GPIO.func_out_sel_cfg[pin].inv_sel = 1;
}

void PWM::detachPin(uint8_t pin)
{
    ledcDetachPin(pin);
    GPIO.func_out_sel_cfg[pin].inv_sel = 0;
}

void PWM::setDuty(float duty_cycle)
{
    if (duty_cycle > 100)
        duty_cycle = 100;
    else if (duty_cycle < 0)
        duty_cycle = 0;
    setDigitalLevel((_digital_range * duty_cycle) / 100);
}

void PWM::setDigitalLevel(uint32_t level)
{
    if (level > _digital_range)
        level = _digital_range;
    _duty_cycle = (level * 100.0f) / _digital_range;
    ledcWrite(_channel, level);
}

void PWM::setDAC(float voltage)
{
    float duty_cycle = 100.0f * voltage / _on_voltage;
    setDuty(duty_cycle);
}

void PWM::setFrequency(float frequency)
{
    _frequency = frequency;
    ledcWriteTone(_channel, _frequency);
}

void PWM::setNote(note_t note, uint8_t octave)
{
    ledcWriteNote(_channel, note, octave);
}