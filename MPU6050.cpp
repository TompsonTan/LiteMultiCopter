#include"Arduino.h"
#include <Wire.h>
#include "MPU6050.h"
#include "MPU6050_RegistersMap.h"

//MPU-6050 默认I2C地址为0x68.
#define MPU6050_I2C_ADDRESS 0x68

MPU6050::MPU6050()
{

}

void MPU6050::Init()
{
    //MPU6050启动时默认：
    //陀螺仪量程：+/- 250deg
    //加速度计量程：+/- 2g
    //使用内部8MHz时钟源
    //传感器处于睡眠模式（sleep mode）

    int error;
    uint8_t c;
    error = Read (MPU6050_WHO_AM_I, &c, 1);
    Serial.print(F("WHO_AM_I : "));
    Serial.print(c,HEX);
    Serial.print(F(", error = "));
    Serial.println(error,DEC);

    //根据数据手册，当传感器上电时，处于休眠模式，sleep位应该应该被清除
    error = Read (MPU6050_PWR_MGMT_2, &c, 1);
    Serial.print(F("PWR_MGMT_2 : "));
    Serial.print(c,HEX);
    Serial.print(F(", error = "));
    Serial.println(error,DEC);

    //清除'sleep' 位，启动传感器
    Write(MPU6050_PWR_MGMT_1, 0, 1);
}

//将从传感器读到的原始数据高低八位置换
void SWAP(uint8_t x,uint8_t y)
{
    uint8_t swap;
    swap = x;
    x = y;
    y = swap;
}

// 读取原始数值
// 一次性读取包括加速度、角速度和温度在内的14个字节数据
// 在MPU-6050的默认设置下是没有打开滤波的，因此数值不是很稳定
int MPU6050::ReadData()
{
    int error = Read(MPU6050_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro, sizeof(accel_t_gyro));

    SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
    SWAP (accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
    SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
    SWAP (accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
    SWAP (accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
    SWAP (accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
    SWAP (accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);

    return error;
}
float MPU6050::ReadAccX()
{
    return accel_t_gyro.value.x_accel/16384.0*9.8;
}

float MPU6050::ReadAccY()
{
    return accel_t_gyro.value.y_accel/16384.0*9.8;
}

float MPU6050::ReadAccZ()
{
    return accel_t_gyro.value.z_accel/16384.0*9.8;
}

float MPU6050::ReadGyroX()
{
    return accel_t_gyro.value.x_gyro/131.0;
}

float MPU6050::ReadGyroY()
{
    return accel_t_gyro.value.y_gyro/131.0;
}

float MPU6050::ReadGyroZ()
{
    return accel_t_gyro.value.z_gyro/131.0;
}

float MPU6050::ReadTemperature()
{
    return ((double) accel_t_gyro.value.temperature + 12412.0) / 340.0;
}

//MPU6050读取函数
//用于从I2C设备读取多个字节的数据
//使用函数Wire.endTransMission()来拉起或释放总线
int MPU6050::Read(int start, uint8_t *buffer, int size)
{
    int i, n, error;

    Wire.beginTransmission(MPU6050_I2C_ADDRESS);
    n = Wire.write(start);
    if (n != 1)
        return (-10);

    n = Wire.endTransmission(false);    // 拉起I2C总线
    if (n != 0)
        return (n);

    //第三个true: 数据读取之后释放I2C总线
    Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
    i = 0;
    while(Wire.available() && i<size)
    {
        buffer[i++]=Wire.read();
    }
    if ( i != size)
        return (-11);

    return 0;
}

// MPU6050写入函数
//用于向I2C设备发送多个字节的数据
//参数:
//start : 起始地址，用来定义寄存器
//pData : 指向要写入数据的指针
//size: 要写入的字节数
int MPU6050::Write(int start, const uint8_t *pData, int size)
{
    int n, error;

    Wire.beginTransmission(MPU6050_I2C_ADDRESS);
    n = Wire.write(start);        //写入起始地址
    if (n != 1)
        return (-20);

    n = Wire.write(pData, size);  //写入数据字节
    if (n != size)
        return (-21);

    error = Wire.endTransmission(true); //释放I2C总线
    if (error != 0)
        return (error);

    return (0);
}
