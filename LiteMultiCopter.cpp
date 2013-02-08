#include"Arduino.h"
#include <Wire.h>

#include"MPU6050.h"
#include"SerialCom.h"

//加速度/陀螺仪传感器
MPU6050 MySensor;

//串口通信
SerialCom MyCom;

void setup()
{
    //初始化I2C总线
    Wire.begin();

    //初始化串口通信
    MyCom.Init();

    //初始化MPU6050
    MySensor.Init();
}

void loop()
{
    //读取陀螺仪数据
    MySensor.ReadData();
    //将陀螺仪数据发送到串口
    MyCom.SensorDataToPC(MySensor);

    delay(100);
}
