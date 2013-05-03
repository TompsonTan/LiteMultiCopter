#ifndef MOTOR_H
#define MOTOR_H

#include"Receiver.h"
#include"MPU6050.h"
#include"PID.h"

class Motor
{
    public:
        Motor();
        virtual ~Motor();
        void CalculateOutput(MPU6050  MySensor,Receiver MyReceiver);
        unsigned char MotorLimitValue(int v);
        unsigned char MotorLimitOffset(int v);
        void Lock();
        void OutPut();

    public:
        PID Pitch_PID,Roll_PID,Yaw_PID;
        int Throttle,Pitch_Offset,Roll_Offset,Yaw_Offset;//油门、副翼、升降舵、方向舵。(操纵时的附加到电机的偏量)
        int Front,Back,Left,Right;//四个电机的输出
};

#endif // MOTOR_H
