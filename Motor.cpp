#include "Motor.h"

#define MaxValue 250.0
#define MinValue 150.0

#define MaxOffset 20.0

Motor::Motor()
{
    Pitch_Offset = Roll_Offset = Yaw_Offset = Throttle = 0;
    Front = Back = Left = Right = 0;
}

Motor::~Motor()
{
    //dtor
}

//根据传感器数据和遥控器信号计算四个电机的输出
void Motor::CalculateOutput(MPU6050  MySensor,Receiver MyReceiver)
{
    Throttle = MyReceiver.RxThr;
    Pitch_Offset =Pitch_PID.update(MySensor.ReadGyroY(),20*MyReceiver.RxEle/100.0);
    Roll_Offset = Roll_PID.update(MySensor.ReadGyroX(),20*MyReceiver.RxAil/100.0);
    Yaw_Offset = Yaw_PID.update(MySensor.ReadGyroZ(),20*MyReceiver.RxRud/100.0);

    Roll_Offset = MotorLimitOffset(Roll_Offset);
    Pitch_Offset = MotorLimitOffset(Pitch_Offset);
    Yaw_Offset = MotorLimitOffset(Yaw_Offset);

    // 十字模式
	//       Front
	//   Left + Right
	//       Back
    Front = MotorLimitValue(Throttle- Pitch_Offset + Yaw_Offset);
    Right = MotorLimitValue(Throttle - Roll_Offset - Yaw_Offset);
    Left = MotorLimitValue(Throttle + Roll_Offset - Yaw_Offset);
    Back = MotorLimitValue(Throttle + Pitch_Offset + Yaw_Offset);
}

//信号限幅
unsigned char Motor::MotorLimitValue(int v)
{
    v = v/100.0*(MaxValue-MinValue)+MinValue;
 	if(v>MaxValue)
        return MaxValue;
	if(v<MinValue)
        return MinValue;
	return v;
}

unsigned char Motor::MotorLimitOffset(int v)
{
 	if(v>MaxOffset)
        return MaxOffset;
	if(v<-MaxOffset)
        return -MaxOffset;
	return v;
}

void Motor::Lock()
{
    Front = Back = Left = Right =0;
    Pitch_PID.Zero();
    Roll_PID.Zero();
    Yaw_PID.Zero();
}

void Motor::OutPut()
{
    //输出电机控制信号
    //analogWrite(2,Front);
    //analogWrite(3,Back);
    analogWrite(5,Right);
    analogWrite(6,Left);
}
