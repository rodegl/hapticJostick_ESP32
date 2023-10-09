#include "PID.h"

PID::PID()
{
}

PID::~PID()
{
}

void PID::setup(float P, float I, float D, float SP)
{
    gainP = P;
    gainD = D;
    gainI = I;
    SetPoint = SP;
    error = 0;
    sumIntegral = 0;
    _lastActual = 0;
}

void PID::setupSP(float gaink1, float gaink2)
{
    k1 = gaink1;
    k2 = gaink2;
    delta1 = 0;
    delta2 = 0;
    delta1_prev = 0;
    delta2_prev = 0;
}

float PID::getOutput(float ActualValue, float T_time)
{

    _T_time = T_time;

    error = SetPoint - ActualValue;

    // P term

    _Poutput = gainP * error;

    // I term
    sumIntegral += error * _T_time;
    _Ioutput = gainI * sumIntegral;

    // D term

    _Doutput = gainD * ((error - _lastActual) / _T_time);

    _lastActual = error;

    // output

    _output = _Poutput + _Ioutput + _Doutput;


    return _output;
}

float PID::superTwisting(float ActualValue, float T_time, float gaink1, float gaink2)
{
    _T_time = T_time;

    error = SetPoint - ActualValue;

    k1 = -gaink1;
    k2 = -gaink2;

    errorST = delta1 - SetPoint;

    // Signo del error estimado
    if (errorST < 0)
    {
        sign = -1;
    }
    else if (errorST > 0)
    {
        sign = 1;
    }
    else
    {
        sign = 0;
    }

    // Estimacion de posición
    delta1 = -delta1_prev + _T_time * (delta2_prev + k1 * (sqrt(abs(errorST)) * sign));
    delta2 = -delta2_prev + _T_time * (k2*sign);
    
    
    // P term

    _Poutput = gainP * error;

    // I term
    sumIntegral += error * _T_time;
    _Ioutput = gainI * sumIntegral;

    // D term

    _Doutput = gainD * delta2;
    //_Doutput = gainD * ((error - _lastActual) / _T_time);

    _lastActual = error;
    delta1_prev = delta1;
    delta2_prev = delta2;

    // output

    _output = _Poutput + _Ioutput + _Doutput;

    return _output;
}


float PID::superTwistingLV(float ActualValue, float T_time, float derivada)
{
    _T_time = T_time;
    error = SetPoint - ActualValue;
    delta2 = derivada;

    // P term
    _Poutput = gainP * error;

    // I term
    sumIntegral += error * _T_time;
    _Ioutput = gainI * sumIntegral;

    // D term
    _Doutput = gainD * delta2;
    //_Doutput = gainD * ((error - _lastActual) / _T_time);

    _lastActual = error;

    // output
    _output = _Poutput + _Ioutput + _Doutput;

    return _output;
}
