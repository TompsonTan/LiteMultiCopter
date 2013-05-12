#ifndef PID_H
#define PID_H

#include"Arduino.h"

class PID
{
    public:
        PID(float p, float i, float d);

        float Calculate(float Ref,float Input);
        void resetITerm();

    public:
        long prevTime;
        float prevRef;
        float prevInput;
        float pTerm;
        float iTerm;
        float dTerm;
        float Kp, Ki, Kd;
};

#endif // PID_H
