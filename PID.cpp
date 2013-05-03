#include "PID.h"

#define TOTAL_SCALING_FACTOR 1500.00 //(1<<10) = 1,024.
#define INTEGRAL_SCALING_FACTOR 8

PID::PID()
{
    p = 0.2;
    i = 0.0;
    d = 0.0;

    Zero();
}

PID::PID(float new_p, float new_i, float new_d)
{
    p = new_p;
    i = new_i;
    d= new_d;
}

float PID::update(float incoming_val,float goal_value)
{
    float delta =goal_value - incoming_val;

    //积分计算
    error += delta;

    //微分计算
    float this_d = incoming_val - prev_val;
    prev_val = incoming_val;

    return (float) (delta*(float)p)+(error*i)/INTEGRAL_SCALING_FACTOR+(this_d*d)/ (float)TOTAL_SCALING_FACTOR;
}

void PID::Zero()
{
    error = 0;
    prev_val = 0;
}
