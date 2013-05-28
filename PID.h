#ifndef PID_H
#define PID_H

#include"Arduino.h"

class PID
{
    public:
        PID();

        void setPID(float p, float i, float d);
        float Calculate(float rcCommand,float gyroData);
        void resetITerm();

    public:
        long prevTime;
        float prevRef;
        float prevInput;
        float pTerm;
        float iTerm;
        float dTerm;
        float Kp, Ki, Kd;
        float error;
        int16_t errorGyroI;
        int16_t delta,delta1,delta2,deltaSum;
        int16_t lastGyro;
};

#endif // PID_H
