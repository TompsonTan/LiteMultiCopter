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
        float pTerm;
        float iTerm;
        float dTerm;
        float Kp, Ki, Kd;
        float error;
        float output;
};

#endif // PID_H
