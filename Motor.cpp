#include "Motor.h"

Motor::Motor()
{
    Thr = Ail = Ele = Rud = 0;
    Front = Back = Left = Right = 0;
}

Motor::~Motor()
{
    //dtor
}

//根据传感器数据和遥控器信号计算四个电机的输出
void Motor::CalculateOutput(MPU6050  MySensor,Receiver MyReceiver)
{
    Ail  = MyReceiver.RxAil-MySensor.ReadGyroX()/25000.0;
    Ele = MyReceiver.RxEle;
    Thr = MyReceiver.RxThr;
    Rud = MyReceiver.RxRud;

    // 十字模式
	//       Front
	//   Left + Right
	//       Back
    Front = MotorLimitValue(Thr - Ele + Rud);
    Right = MotorLimitValue(Thr - Ail - Rud);
    Left = MotorLimitValue(Thr + Ail - Rud);
    Back = MotorLimitValue(Thr + Ele + Rud);
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

void Motor::ZeroOutput()
{
    Front = Back = Left = Right =0;
}
