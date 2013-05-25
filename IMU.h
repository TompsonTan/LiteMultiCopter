#ifndef IMU_H_INCLUDED
#define IMU_H_INCLUDED

#include"Arduino.h"
#include"MPU6050.h"
#include"lmcMath/Math.h"

class IMU
{
public:
    IMU(MPU6050 *MySenser);

    void initialze();
    void integrateRawGyro();
public:
    MPU6050 *Senser;
    Quaternion Quater;//方位四元数
    Vector Euler;//姿态角
    float rawPitch,rawRoll;
    long prevTime;
};


#endif // IMU_H_INCLUDED
