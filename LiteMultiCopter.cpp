#include"Arduino.h"
#include <Wire.h>

#include"MPU6050.h"
#include"SerialCom.h"
#include"Receiver.h"

//加速度/陀螺仪传感器
MPU6050 MySensor;

//串口通信
SerialCom MyCom;

//接收机信号读取
Receiver MyReceiver;

//中断服务函数，用来检测接收机信号
/*当发现接收机通道电平变化时，会中断当前的“进程”，
执行中断服务函数，读取接收机的信号，完成任务后返回*/
SIGNAL(PCINT2_vect)
{
    MyReceiver.MegaPcIntISR();
}

void setup()
{
    //初始化I2C总线
    Wire.begin();

    //初始化串口通信
    MyCom.Init();

    //初始化MPU6050
    MySensor.Init();

    //设置接收机信号端口
    MyReceiver.Init();
}

void loop()
{
    //读取陀螺仪数据
    //MySensor.ReadData();
    //将陀螺仪数据发送到串口
    //MyCom.SensorDataToPC(MySensor);

    //读取接收机信号
    MyReceiver.ReadData();
    //将接收机信号发送到串口
    MyCom.ReceiverDataToPC(MyReceiver);

    delay(250);
}
