#ifndef MOTOR_H
#define MOTOR_H

#include"Receiver.h"

class Motor
{
    public:
        Motor();
        virtual ~Motor();
        void CalculateOutput(Receiver MyReceiver);
        unsigned char MotorLimitValue(int v);

    public:
        int thr,ail,ele,rud;//遥控四个通道的输入:油门、副翼、升降舵、方向舵。
        int Front,Back,Left,Right;//四个电机的输出
};

#endif // MOTOR_H
