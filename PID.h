#ifndef PID_H
#define PID_H

#include"Arduino.h"

class PID
{
    public:
        PID();
        PID(float new_p, float new_i, float new_d);

        float update(float incoming_val,float goal_value);
        void Zero();
        float p,i,d;

    private:
        float error;
        float prev_val;
};

#endif // PID_H
