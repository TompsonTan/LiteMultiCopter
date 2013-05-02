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


//通道的数据
typedef struct {
  byte edge;
  unsigned long riseTime;
  unsigned long fallTime;
  unsigned int lastGoodWidth;
} tPinTimingData;


//这里要用到ISR中断服务函数来读取接收机的数据
class Receiver
{
public:
    Receiver();
    void Init();
    void ReadData();
    void MegaPcIntISR();
    int getRawChannelValue(byte channel);

    int ChannelData[4];

    float RxThr;//油门,缩放：0~100.0
    float RxAil,RxEle,RxRud;//副翼、升降舵、方向舵。(与1500相减后的偏差)，再缩放到-100~+100

    tPinTimingData pinData[9];

    uint8_t PCintLast[3];
};


#endif // RECEIVER_H_INCLUDED
