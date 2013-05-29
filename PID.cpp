#include "PID.h"


PID::PID()
{
    pTerm = iTerm = dTerm = 0;
    error = 0;
}

void PID::setPID(float p, float i, float d)
{
    Kp = p;
    Ki = i;
    Kd = d;
}
float PID::Calculate(float Input, float Ref)
{
    //计算采样时间，并把ms转换为s
    long dt = millis() - prevTime;

    //if(dt>2)
    {
        float dt_float = dt*0.001;

        float error = Input-Ref;
        pTerm = Kp*error;
        dTerm = - Kd*(Ref - prevRef)/dt_float;
        iTerm += Ki*error*dt_float;

        if(iTerm>20)
            iTerm = 0;
        if(abs(error)<10)
            iTerm = 0;

        output = pTerm + iTerm + dTerm;

        //更新状态
        prevTime = millis();
        prevRef = Ref;
    }
    return output;
}

void PID::resetITerm()
{
    iTerm = 0;
    //prevTime = millis();
}
