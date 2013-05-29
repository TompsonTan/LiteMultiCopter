#include"Arduino.h"
#include <Wire.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include "Streaming.h"
#include"def.h"
#include"Receiver.h"
#include"Motor.h"

#define USE_SERIAL 1


MPU6050  mpu;//加速度/陀螺仪传感器
Receiver    LMC_Receiver;//接收机信号读取
Motor       LMC_Motor;//电机控制

// MPU 控制/状态-变量
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

//姿态/运动-变量
Quaternion q;           // [w, x, y, z]         四元数
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   偏航/俯仰/滚转

//MPU中断检测
volatile bool mpuInterrupt = false;     //标记MPU中断引脚是否为高电平
void dmpDataReady()
{
    mpuInterrupt = true;
}


//接收机信号，中断服务函数
/*当发现接收机通道电平变化时，会中断当前的“进程”，
执行中断服务函数，读取接收机的信号，完成任务后返回*/
SIGNAL(PCINT2_vect)
{
    LMC_Receiver.MegaPcIntISR();
}

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

int currentSerialNum = 1;
int nextSerialNum;
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

void initMPU()
{
    //初始化MPU6050
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    //核实MPU已连接
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    //等待串口准备好
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    //while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    //确认正常工作(devStatus为0)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // 打开Arduino 中断检测（外部中断0，即数字2引脚）
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        //否则为出现错误!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void getEulerAngle()
{
    //重置中断标记并获取INT_STATUS字节
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    //检查是否溢出 (除非我们的代码效率十分低，否则几乎不会溢出)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    }
    //检查DMP数据是否准备好中断读取
    else if (mpuIntStatus & 0x02)
    {
        //等待正确的可用数据长度（不用等很久）
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
    }
}

void setup()
{
    //初始化I2C总线
    Wire.begin();

    //设置起始波特率
    Serial.begin(38400);
    // 注意: 8MHz 或更低速度的MCU, 例如3.3v的Teensy或Ardunio
    // Pro Mini , 无法处理太高的波特率，要将其设置在38400或以下。

    //while (!Serial); // wait for Leonardo enumeration, others continue immediately

    //设置接收机信号端口
    LMC_Receiver.Init();

    //初始化MPU6050
    initMPU();

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
    //等待MPU的DMP准备好，否则不要进行任何操作
    if (!dmpReady) return;

    //DMP之外的其他操作在这里定义
    while (!mpuInterrupt && fifoCount < packetSize)
    {
        // 如果你偏向于在其他地方定义这些操作，你可以通过查看mpuInterrupt是否为真，
        // 若为真，则使用break来退出while循环以执行读取MPU数据的操作。

        //读取/更新接收机信号
        //LMC_Receiver.ReadData();
        //读取接收机信号，结果需转换后控制电机
        LMC_Motor.CalculateOutput(0,ypr[1],ypr[2],LMC_Receiver);

        /******下面就是PID算法和根据PID的结果来控制电机的函数了******/

        if(LMC_Receiver.RxThr<1200)//若油门低于10%，进入锁定/解锁函数
            ArmingRoutine();

        //读取接收机信号，结果需转换后控制电机
        //LMC_Motor.CalculateOutput(0,ypr[1],ypr[2],LMC_Receiver);

        //若已锁定或未建立陀螺仪基准，禁止电机输出
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

        //以角度单位显示偏航、俯仰、滚转角度
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        if(currentSerialNum == 1)
        {
//            Serial.print('H');
//            Serial.print(ypr[0] * 180/M_PI);
//            Serial.print('*');
//            Serial.print(ypr[1] * 180/M_PI);
//            Serial.print('*');
//            Serial.print(ypr[2] * 180/M_PI);
//            Serial.print('*');
//            Serial.print('\n');
            Serial << 'H'
            << ypr[2] * 180/M_PI << '*'
            <<LMC_Receiver.RxAil/12<< '*'
            <<LMC_Motor.Roll_Offset << '*'
            <<LMC_Motor.Roll_PID.dTerm<< '*'
            <<'\n';
            //nextSerialNum = 2;
            //currentSerialNum = 2;
        }
//        if(currentSerialNum ==2)
//        {
//            Serial.print('M');
//            Serial.print(LMC_Receiver.RxAil/12);
//            Serial.print('*');
//            Serial.print(LMC_Motor.Roll_Offset);
//            Serial.print('*');
//            Serial.print(LMC_Motor.Roll_PID.pTerm);
//            Serial.print('*');
//            Serial.print(LMC_Motor.Roll_PID.dTerm);
//            Serial.print('*');
//            Serial.print('\n');
//            currentSerialNum = 1;
//        }
    }

    //获取偏航角速度、俯仰角、滚转角
    getEulerAngle();

    //读取/更新接收机信号
    LMC_Receiver.ReadData();
}
