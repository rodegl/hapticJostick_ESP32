#ifndef __PID_H__
#define __PID_H__

#include <Arduino.h>
#include <ESP32_PWM.h>

class PID
{
    public: 
        PID();
        ~PID();
        void setup(float gainP, float I, float gainD, float SetPoint);
        void setupSP(float gaink1, float gaink2);
        float getOutput(float ActualValue, float T_time);
        float superTwisting(float ActualValue, float T_time, float gaink1, float gaink2);
        float superTwistingLV(float ActualValue, float T_time, float derivada);
        float read();
        float gainP;
        float gainI;
        float gainD;
        float SetPoint;
        float error;
        float sumIntegral;
        float derivateE;

        //SUPER TWISTING
        //ganancias del algoritmo de Super Twisting
        float k1;
        float k2;
        float delta1; //posicion estimada
        float delta2; //velocidad estimada
        float delta1_prev;
        float delta2_prev;
        float errorST; //error estimado por Super Twisting
        float sign; //signo del error estimado

    private: 
        bool _firstRun; 
        float _T_time;
        float _lastActual;
        float _output;
        float _Doutput;
        float _Ioutput;
        float _Poutput;

    protected: 
};

#endif // __PID_H__ 