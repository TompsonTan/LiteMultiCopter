#include "PID.h"

#define TOTAL_SCALING_FACTOR 1500.00 //(1<<10) = 1,024.
#define INTEGRAL_SCALING_FACTOR 8

PID::~PID()
{
    p = 0;
    i = 0;
    d = 0;
}

PID::PID(int16_t new_p, int16_t new_i, int16_t new_d)
{
    p = new_p;
    i = new_i;
    d= new_d;
}

int16_t PID::update(int16_t incoming_val,int16_t goal_value)
{
    int16_t delta =goal_value - incoming_val;

    //积分计算
    error += delta;

    //微分计算
    int16_t this_d = incoming_val - prev_val;
    prev_val = incoming_val;

    return (float) (delta*(float)p)+(error*i)/INTEGRAL_SCALING_FACTOR+(this_d*d)/ (float)TOTAL_SCALING_FACTOR;
}

void PID::zero()
{
    error = 0;
}
