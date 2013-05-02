#include "Motor.h"

Motor::Motor()
{
    thr = ail = ele =rud = 0;
    Front = Back = Left = Right = 0;
}

Motor::~Motor()
{
    //dtor
}

//根据传感器数据和遥控器信号计算四个电机的输出
void Motor::CalculateOutput(MPU6050  MySensor,Receiver MyReceiver)
{
    ail  = (MyReceiver.ChannelData[0]-1500)-MySensor.ReadGyroX()/25000.0;
    ele = (MyReceiver.ChannelData[1]-1500)/32.0;
    thr = MyReceiver.ChannelData[2]/8.0;
    rud = (MyReceiver.ChannelData[3]-1500)/32.0;

    // 十字模式
	//       Front
	//   Left + Right
	//       Back
    Front = MotorLimitValue(thr - ele + rud);
    Right = MotorLimitValue(thr - ail - rud);
    Left = MotorLimitValue(thr + ail - rud);
    Back = MotorLimitValue(thr + ele + rud);
}

//信号限幅
unsigned char Motor::MotorLimitValue(int v)
{
 	if(v>255)
        return 255;
	if(v<0)
        return 0;
	return v;
}
