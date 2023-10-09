#ifndef __ENCODER_H__
#define __ENCODER_H__

#include <Arduino.h>

class Encoder
{
    public:
        Encoder();
        ~Encoder();
        void setup(uint8_t chA, uint8_t chB, float degrees);
        void handler();
        void get();
        bool flag;
        uint8_t pinA;
        uint8_t pinB;
        float speed;
        float position;
        bool stateA;
        bool stateB;
        long counts;
        ulong prev_Time, current_time;
        long prev_counts, num_counts, current_counts;
        uint8_t counter;   
        

    private:
        int _mode;
        bool _stateA;
        bool _stateB;
        float _angle;
        



};


#endif // __ENCODER_H__