#include"Arduino.h"
#include <Wire.h>

#include"def.h"
#include"MPU6050.h"
#include"IMU.h"
#include"SerialCom.h"
#include"Receiver.h"
#include"Motor.h"

#define USE_SERIAL 1


MPU6050  LMC_Sensor;//加速度/陀螺仪传感器
IMU          LMC_IMU(&LMC_Sensor);//姿态解算
SerialCom LMC_Com;//串口通信
Receiver    LMC_Receiver;//接收机信号读取
Motor       LMC_Motor;//电机控制

//中断服务函数，用来检测接收机信号
/*当发现接收机通道电平变化时，会中断当前的“进程”，
执行中断服务函数，读取接收机的信号，完成任务后返回*/
SIGNAL(PCINT2_vect)
{
    LMC_Receiver.MegaPcIntISR();
}

//陀螺仪基准建立
#define GYROBASECNT 200
int GyroBaseCnt=0;			//Gyro base build count 陀螺仪中点建立计数器

//四轴锁定/解锁函数
int InLock = 1;//初始为锁定状态
int ArmCnt = 0;	//锁定/解锁计数
//#define ARMING_TIME 350
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
                GyroBaseCnt=GYROBASECNT;
                LMC_IMU.initialze();
            }
        }
        else
        {
            if(LMC_Receiver.RxRud<-STICKGATE)//方向舵向左，锁定
                InLock = 1;
        }
    }
}

bool ESC_isCalibrated = false;
void CalibrateThrottle()
{
    if(!ESC_isCalibrated)
    {
        if(LMC_Receiver.RxThr>85)
        {
            InLock = 0;
            ESC_isCalibrated = true;
        }
    }
}

void setup()
{
    //初始化I2C总线
    Wire.begin();

//#if(USE_SERIAL)
    //初始化串口通信
    LMC_Com.Init();
    //设置起始波特率
    Serial.begin(9600);
//#endif

    //初始化MPU6050
    LMC_Sensor.Init();

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

    delay(2000);
    LMC_Receiver.ReadData();
    LMC_Receiver.Calibrate();

    pinMode(LockLED,OUTPUT);
    digitalWrite(LockLED,LOW);

    LMC_Motor.CalibrateESCs();
}

void loop()
{
    //读取/更新陀螺仪数据
    LMC_Sensor.ReadData();

    //读取/更新接收机信号
    LMC_Receiver.ReadData();


    //将陀螺仪和接收机信号发送到串口
    //LMC_Com.DataToPC(LMC_Sensor,LMC_Receiver);


    /******下面就是PID算法和根据PID的结果来控制电机的函数了******/

    if(LMC_Receiver.RxThr<1200)//若油门低于10%，进入锁定/解锁函数
        ArmingRoutine();

    //读取接收机信号，结果需转换后控制电机
    LMC_Motor.CalculateOutput(LMC_Sensor,LMC_Receiver);

    //若已锁定或未建立陀螺仪基准，禁止电机输出
    if(InLock)
    {
        LMC_Motor.Lock();
        digitalWrite(LockLED,LOW);
    }
    else
    {
        //IMU姿态解算
        LMC_IMU.integrateRawGyro();

        digitalWrite(LockLED,HIGH);
        //输出电机控制信号
        LMC_Motor.OutPut();
    }

//    Serial.print('H');//开头标记
//    Serial.print(LMC_Receiver.RxEle);
//    Serial.print('*');//数据分隔符
//    Serial.print(LMC_Sensor.ReadGyroY());
//    Serial.print('*');
//    Serial.println(LMC_Receiver.ChannelData[3]);
//    Serial.print('*');
//    Serial.print(InLock);
//    Serial.print('*');

    Serial.print('H');//开头标记
    Serial.print(LMC_IMU.Euler.x);
    Serial.print('*');
    Serial.print(LMC_IMU.Euler.y);
    Serial.print('*');
    Serial.print(LMC_IMU.Euler.z);
    Serial.print('*');
    Serial.print('\n');//结束标记

//    Serial.print('H');//开头标记
//    Serial.print(LMC_Sensor.ReadGyroX());
//    Serial.print('*');
//    Serial.print(LMC_Sensor.ReadGyroY());
//    Serial.print('*');
//    Serial.print(LMC_Sensor.ReadGyroZ());
//    Serial.print('*');
//    Serial.print('\n');//结束标记
}
