#include "PID.h"


PID::PID()
{
    //prevTime = millis();

    pTerm = iTerm = dTerm = 0;
    errorGyroI = 0;
    error = 0;
    delta = deltaSum = 0;
    lastGyro = 0;
}

void PID::setPID(float p, float i, float d)
{
    Kp = p;
    Ki = i;
    Kd = d;
}
float PID::Calculate(float Input, float Ref)
{
//    error = rcCommand*10*8/Kp - gyroData;
//
//    //P项
//    pTerm = error*Kp/8/10;
//
//    //I项
//    //陀螺仪积分并限幅在+/-16000
//    errorGyroI = constrain(errorGyroI+error,-16000,+16000);
//    //如果角速度大于某个值，清空积分值，640/8192*2000 = 156 deg/sec
//    if(abs(gyroData)>640)
//        errorGyroI = 0;
//    iTerm = (errorGyroI/125*Ki)>>8;
//
//    //D项
//    delta = gyroData - lastGyro;
//    lastGyro = gyroData;
//    deltaSum = delta + delta1 + delta2;
//    delta2 = delta1;
//    delta1 = delta;
//    dTerm = (deltaSum*Kd)>>5;
//
//    //最终输出
//    return pTerm + iTerm - dTerm;

    //计算采样时间，并把ms转换为s
    long dt = millis() - prevTime;
    float dt_float = dt*0.001;

    float error = Input-Ref;
    pTerm = Kp*error;
    dTerm = - Kd*(Ref - prevRef)/dt_float;
    iTerm += Ki*error*dt;

    if(abs(dTerm)>500)
        dTerm = 0;
    if(dt=0)
        dTerm = 0;
    float output = pTerm + iTerm + dTerm;

    //更新状态
    prevTime = millis();
    prevRef = Ref;
    //prevInput = Input;

    return output;
}

void PID::resetITerm()
{
    iTerm = 0;
    errorGyroI = 0;
    //prevTime = millis();
}
