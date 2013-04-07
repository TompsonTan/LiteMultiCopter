#include"Arduino.h"
#include <Wire.h>

#include"MPU6050.h"
#include"SerialCom.h"
#include"Receiver.h"
#include"Motor.h"

//根据Uno板的PWM接口设定的电机控制引脚
#define Motor_Front 3
#define Motor_Back 5
#define Motor_Left 6
#define Motor_Right 9


MPU6050  LMC_Sensor;//加速度/陀螺仪传感器
SerialCom LMC_Com;//串口通信
Receiver    LMC_Receiver;//接收机信号读取

//中断服务函数，用来检测接收机信号
/*当发现接收机通道电平变化时，会中断当前的“进程”，
执行中断服务函数，读取接收机的信号，完成任务后返回*/
SIGNAL(PCINT2_vect)
{
    LMC_Receiver.MegaPcIntISR();
}

void setup()
{
    //初始化I2C总线
    Wire.begin();

    //初始化串口通信
    LMC_Com.Init();

    //初始化MPU6050
    LMC_Sensor.Init();

    //设置接收机信号端口
    LMC_Receiver.Init();
}

void loop()
{
    //读取/更新陀螺仪数据
    LMC_Sensor.ReadData();

    //读取/更新接收机信号
    LMC_Receiver.ReadData();

    //将陀螺仪和接收机信号发送到串口
    LMC_Com.DataToPC(LMC_Sensor,LMC_Receiver);

    /******下面就是PID算法和根据PID的结果来控制电机的函数了******/

    //电机控制未与串口进行通信，有待改进
   void Motor::SingleMotorTest();
   {
       int val;
       for(val=43;val<=255;val=val+1)
       {
           analogWrite(Motor_Front,val)；
       }
       for(val=255;val>=43;val=val-1)
       {
           analogWrite(Motor_Front,val)；
       }
    }

}

    delay(250);
}
