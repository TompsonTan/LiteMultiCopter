#ifndef PID_H
#define PID_H

#include"Arduino.h"

class PID
{
    public:
        PID();
        PID(int16_t new_p, int16_t new_i, int16_t new_d);
        ~PID();
        int16_t update(int16_t incoming_val,int16_t goal_value);
        void zero();
        int16_t p,i,d;

    private:
        int64_t error;
        int16_t prev_val;
};

#endif // PID_H
