#include"Arduino.h"
#include <Wire.h>

#include"def.h"
#include"Receiver.h"
#include"Motor.h"

#include <I2Cdev.h>
#include <MPU60X0.h>
#include <SPI.h>
#include <EEPROM.h>

#include "FreeIMU.h"


/*    四轴（前、左）和传感器MPU（X、Y、Z）的关系
                     /\   X 前
                      |
                      |
左                   |
Y<-------------+   Z轴 垂直于屏幕向外为正

规定：俯仰角-上仰为正，滚转角-右滚为正，偏航角-左偏为正
遥控器信号、传感器数据与规定不一致的时候应修改正负号
*/


float ypr[3]; // 偏航、俯仰、滚转角度
float val[9];
float yawRate;//偏航角速度


FreeIMU    myIMU = FreeIMU();//IMU惯性测量单元
Receiver    LMC_Receiver;//接收机信号读取
Motor       LMC_Motor;//电机控制

//中断服务函数，用来检测接收机信号
/*当发现接收机通道电平变化时，会中断当前的“进程”，
执行中断服务函数，读取接收机的信号，完成任务后返回*/
SIGNAL(PCINT2_vect)
{
    LMC_Receiver.MegaPcIntISR();
}

//四轴锁定/解锁函数
int InLock = 1;//初始为锁定状态
int ArmCnt = 0;	//锁定/解锁计数
#define ARMING_TIME 150
#define STICKGATE 300//锁定/解锁阀值

#if defined(Mega2560)
    int LockLED = A5;//若常亮，则为锁定；闪烁，则为已解锁
#elif defined(Promini)
    int LockLED = 13;
#endif // defined
void ArmingRoutine()
{
    //消抖动,计数达到阀值的时候认为执行锁定或解锁
    if(abs(LMC_Receiver.RxRud)>STICKGATE)
        ArmCnt++;
    else
        ArmCnt = 0;

    if(ArmCnt>ARMING_TIME)
    {
        if(InLock)
        {
            if(LMC_Receiver.RxRud>STICKGATE)//方向舵向右，解锁
            {
                InLock = 0;
                //解锁后重置MPU
                myIMU.init();
            }
        }
        else
        {
            if(LMC_Receiver.RxRud<-STICKGATE)//方向舵向左，锁定
                InLock = 1;
        }
    }
}

void setup()
{
    //设置起始波特率
    Serial.begin(9600);

    //初始化I2C总线
    Wire.begin();

    //设置接收机信号端口
    LMC_Receiver.Init();

#if defined(Mega2560)
    pinMode(4, OUTPUT);
    pinMode(3, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
#elif defined(Promini)
    pinMode(3, OUTPUT);
    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);
    pinMode(11, OUTPUT);
#endif // defined

    delay(200);
    LMC_Receiver.ReadData();
    LMC_Receiver.Calibrate();

    pinMode(LockLED,OUTPUT);
    digitalWrite(LockLED,LOW);

    LMC_Motor.CalibrateESCs();

    myIMU.init();
    delay(5);
}

void loop()
{
    //读取/更新接收机信号
    LMC_Receiver.ReadData();

    if(LMC_Receiver.RxThr<1200)//若油门低于10%，进入锁定/解锁函数
        ArmingRoutine();

    //读取接收机信号，结果需转换后控制电机
    LMC_Motor.CalculateOutput(yawRate,ypr[1],ypr[2],LMC_Receiver);

    //若已锁定，禁止电机输出
    if(InLock)
    {
        LMC_Motor.Lock();
        digitalWrite(LockLED,LOW);
    }
    else
    {
        digitalWrite(LockLED,HIGH);
        //输出电机控制信号
        LMC_Motor.OutPut();
    }

    //获取姿态角和角速度
    myIMU.getYawPitchRoll(ypr);
    myIMU.getValues(val);
    yawRate = val[5];

//    Serial.print('H');//开头标记
//    Serial.print(LMC_Receiver.RxEle);
//    Serial.print('*');//数据分隔符
//    Serial.print(LMC_Sensor.ReadGyroY());
//    Serial.print('*');
//    Serial.println(LMC_Receiver.ChannelData[3]);
//    Serial.print('*');
//    Serial.print(InLock);
//    Serial.print('*');
//
//    Serial.print('H');//开头标记
//    Serial.print(ypr[0]);
//    Serial.print('*');
//    Serial.print(ypr[1]);
//    Serial.print('*');
//    Serial.print(ypr[2]);
//    Serial.print('*');
//    Serial.print(val[5]);
//    Serial.print('*');
//    Serial.print('\n');//结束标记
}
