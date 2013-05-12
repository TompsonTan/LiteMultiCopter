#include "PID.h"

PID::PID(float p, float i, float d)
{
    Kp = p;
    Ki = i;
    Kd = d;
    prevTime = millis();

    iTerm = 0;
}

float PID::Calculate(float Ref,float Input)
{
    //计算采样时间，并把ms转换为s
    long dt = millis() - prevTime;
    float dt_float = dt*0.001;

    float error = Ref - Input;
    pTerm = Kp*error;
    dTerm = - Kd*(Input - prevInput)/dt_float;
    iTerm += Ki*error*dt;

    float output = pTerm + iTerm + dTerm;

    //更新状态
    prevTime = millis();
    prevRef = Ref;
    prevInput = Input;

    return output;
}

void PID::resetITerm()
{
    iTerm = 0;
    prevTime = millis();
}
