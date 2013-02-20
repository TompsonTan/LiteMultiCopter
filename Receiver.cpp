#include"Receiver.h"


Receiver::Receiver()
{

}

void Receiver::Init()
{
    DDRK = 0; //设置端口K为输入
    PORTK = 0;//端口K全部置零

    //引脚电平变化屏蔽寄存器 2 (PCMSK2)的低四位全部置为1，即：
    //使到PCINT16、17、18、19中断使能，接收机的四个通道接到ANALOG IN的8、9、10、11引脚
    PCMSK2 |=(1<<4)-1;

    //将“引脚电平变化中断控制寄存器”的第三位（PCIE2）置为1，即：
    //使到PCINT16-23的引脚上的任何电平变化都会引起中断
    //注意：四通道，但开启了6个“引脚电平变化中断”
    PCICR |= 0x1 << 2;

    ChannelData[XAXIS] = 1500;
    ChannelData[YAXIS] = 1500;
    ChannelData[ZAXIS] = 1500;
    ChannelData[THROTTLE] = 1000;
}


/*avr在硬件中断过程中,只是对程序计数器2个字节压入堆栈,
状态寄存器不由硬件处理,要由用户软件来完成。
就是说要由软件来压入堆栈*/


void Receiver::ReadData()
{
    for(byte channel = XAXIS; channel < 4; channel++)
    {
        // 接收机信号校正/微调
        ChannelData[channel] = getRawChannelValue(channel);
    }
}

void Receiver::MegaPcIntISR()
{
    uint8_t bit;
    uint8_t curr;
    uint8_t mask;
    uint8_t pin;
    uint32_t currentTime;
    uint32_t time;

    curr = PINK;
    mask = curr ^ PCintLast[0];
    PCintLast[0] = curr;

    //mask 标记了哪些引脚的的电平发生了变化，
    //如果没有变化，则直接返回。
    if ((mask &= PCMSK2) == 0)
    {
        return;
    }

    //获取自系统启动后的时间，这里得到的是微秒us（microseconds），而不是毫秒（millisecond）
    currentTime = micros();

    for (uint8_t i=0; i < 8; i++)
    {
        bit = 0x01 << i;
        if (bit & mask)
        {
            pin = i;
            // for each pin changed, record time of change
            if (bit & PCintLast[0])
            {
                time = currentTime - pinData[pin].fallTime;
                pinData[pin].riseTime = currentTime;
                if ((time >= MINOFFWIDTH) && (time <= MAXOFFWIDTH))
                    pinData[pin].edge = RISING_EDGE;
                else
                    pinData[pin].edge = FALLING_EDGE; //检测到无效的上升沿
            }
            else
            {
                time = currentTime - pinData[pin].riseTime;
                pinData[pin].fallTime = currentTime;
                if ((time >= MINONWIDTH) && (time <= MAXONWIDTH) && (pinData[pin].edge == RISING_EDGE))
                {
                    pinData[pin].lastGoodWidth = time;//占空时间
                    pinData[pin].edge = FALLING_EDGE;
                }
            }
        }
    }
}

int Receiver::getRawChannelValue(byte channel)
{
    byte pin = channel;
    uint8_t oldSREG = SREG;
    cli();
    // Get receiver value read by pin change interrupt handler
    uint16_t receiverRawValue = pinData[pin].lastGoodWidth;
    SREG = oldSREG;

    return receiverRawValue;
}
