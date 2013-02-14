#ifndef RECEIVER_H_INCLUDED
#define RECEIVER_H_INCLUDED

#include"Arduino.h"

// Receiver variables
#define TIMEOUT 25000
#define MINCOMMAND 1000
#define MIDCOMMAND 1500
#define MAXCOMMAND 2000
#define MINDELTA 200
#define MINCHECK (MINCOMMAND + 100)
#define MAXCHECK (MAXCOMMAND - 100)
#define MINTHROTTLE (MINCOMMAND + 100)
#define LEVELOFF 100

//PWM相关
#define RISING_EDGE 1
#define FALLING_EDGE 0
#define MINONWIDTH 950
#define MAXONWIDTH 2075
#define MINOFFWIDTH 12000
#define MAXOFFWIDTH 24000

//通道定义
#define ChannelNumber 4
#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2
#define THROTTLE 3

    int lastReceiverChannel = 0;

    float receiverXmitFactor = 0.0;
    int receiverData[ChannelNumber] = {0,0,0,0};
    int receiverZero[3] = {0,0,0};
    int receiverCommand[ChannelNumber] = {0,0,0,0};
    float receiverSlope[ChannelNumber] = {0.0,0.0,0.0,0.0};
    float receiverOffset[ChannelNumber] = {0.0,0.0,0.0,0.0};

  // arduino pins 67, 65, 64, 66, 63, 62
  static byte receiverPin[4] = {0,1,2,3}; // bit number of PORTK used for XAXIS, YAXIS, ZAXIS, THROTTLE

//通道的数据
typedef struct {
  byte edge;
  unsigned long riseTime;
  unsigned long fallTime;
  unsigned int lastGoodWidth;
} tPinTimingData;
volatile static tPinTimingData pinData[9];

volatile static uint8_t PCintLast[3];

//这里要用到ISR中断服务函数来读取接收机的数据
class Receiver
{
public:
    Receiver();
    void Init();
    void ReadData();
    void MegaPcIntISR();
    int getRawChannelValue(byte channel);
};


#endif // RECEIVER_H_INCLUDED
