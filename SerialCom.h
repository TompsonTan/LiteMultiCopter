#ifndef SERIALCOM_H_INCLUDED
#define SERIALCOM_H_INCLUDED

#include"Arduino.h"
#include"MPU6050.h"
#include"Receiver.h"

//该类用于飞控板和电脑端监控/调试软件的通信
class SerialCom
{
public:
    SerialCom();

    void Init();//串口初始化
    void SensorDataToPC(MPU6050 Sensor);//将MPU6050的数据发送到串口
    void ReceiverDataToPC(Receiver MyReceiver);//将接收机的信号数据数据发送到串口
};

#endif // SERIALCOM_H_INCLUDED
