#include"IMU.h"

IMU::IMU(MPU6050 *MySenser)
{
    Senser = MySenser;
}

void IMU::initialze()
{
    prevTime = 0;

    // 由欧拉角初始化（全局到机体）四元数和变换矩阵
    Quater.init(Euler);
}
void IMU::integrateRawGyro()
{
    float dt = millis() - prevTime;
    prevTime = millis();
    float P = Senser->ReadGyroZ()/57.3;
    float Q = Senser->ReadGyroY()/57.3;
    float R = Senser->ReadGyroX()/57.3;
    Quater.step(dt/1000,Vector(P,Q,R));
    Quater.updateEuler();
    Euler.x = Quater.euler.x;
    Euler.y = Quater.euler.y;
    Euler.z = Quater.euler.z;
}
