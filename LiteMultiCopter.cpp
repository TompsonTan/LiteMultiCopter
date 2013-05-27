#include"Arduino.h"
#include <Wire.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include"def.h"
#include"Receiver.h"
#include"Motor.h"

#define USE_SERIAL 1


MPU6050  mpu;//加速度/陀螺仪传感器
Receiver    LMC_Receiver;//接收机信号读取
Motor       LMC_Motor;//电机控制

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}


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

    //设置起始波特率
    Serial.begin(38400);

    //初始化MPU6050

    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
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
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

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
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    //读取/更新接收机信号
    LMC_Receiver.ReadData();


    //将陀螺仪和接收机信号发送到串口
    //LMC_Com.DataToPC(LMC_Sensor,LMC_Receiver);

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & 0x02)
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetEuler(euler, &q);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        //Serial.print("euler\t");
        Serial.print('H');
        Serial.print(ypr[0] * 180/M_PI);
        Serial.print('*');
        Serial.print(ypr[1] * 180/M_PI);
        Serial.print('*');
        Serial.print(ypr[2] * 180/M_PI);
        Serial.print('*');
        Serial.print('\n');
    }

    /******下面就是PID算法和根据PID的结果来控制电机的函数了******/

    if(LMC_Receiver.RxThr<1200)//若油门低于10%，进入锁定/解锁函数
        ArmingRoutine();

    //读取接收机信号，结果需转换后控制电机
    LMC_Motor.CalculateOutput(ypr,LMC_Receiver);

    //若已锁定或未建立陀螺仪基准，禁止电机输出
    if(InLock)
    {
        LMC_Motor.Lock();
        digitalWrite(LockLED,LOW);
    }
    else
    {
        //IMU姿态解算
        //LMC_IMU.integrateRawGyro();

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
}
