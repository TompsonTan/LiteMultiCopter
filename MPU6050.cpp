#include"Arduino.h"
#include <Wire.h>
#include "MPU6050.h"
#include "MPU6050_RegistersMap.h"

//MPU-6050 默认I2C地址为0x68.
#define MPU6050_I2C_ADDRESS 0x68

MPU6050::MPU6050()
{
    lastGyroX = lastGyroY = lastGyroZ = 0;
}

void MPU6050::Init()
{
    //MPU6050启动时默认：
    //陀螺仪量程：+/- 250deg
    //加速度计量程：+/- 2g
    //使用内部8MHz时钟源
    //传感器处于睡眠模式（sleep mode）

    //清除'sleep' 位，启动传感器
    uint8_t data = 0;
    Write(MPU6050_PWR_MGMT_1, &data, 1);
}

// 读取原始数值
// 一次性读取包括加速度、角速度和温度在内的14个字节数据
// 在MPU-6050的默认设置下是没有打开滤波的，因此数值不是很稳定
void MPU6050::ReadData()
{
    Read(MPU6050_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro, sizeof(accel_t_gyro));

    //将从传感器读到的原始数据高低八位置换
    uint8_t swap;
    #define SWAP(x,y) swap = x; x = y; y = swap
    SWAP(accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
    SWAP(accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
    SWAP(accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
    SWAP(accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
    SWAP(accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
    SWAP(accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
    SWAP(accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);
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
    float GyroX = accel_t_gyro.value.x_gyro/131.0-1.25;
    //和上一个值综合一下，防止波形的突变，起滤波作用
    GyroX = (3*GyroX+lastGyroX)/4;
    lastGyroX = GyroX;
    return GyroX;
}

float MPU6050::ReadGyroY()
{
    float GyroY = accel_t_gyro.value.y_gyro/131.0-3.7;
    GyroY = (3*GyroY+lastGyroY)/4;
    lastGyroY = GyroY;
    return GyroY;
}

float MPU6050::ReadGyroZ()
{
    float GyroZ = accel_t_gyro.value.z_gyro/131.0+0.1;
    GyroZ = (3*GyroZ+lastGyroZ)/4;
    lastGyroZ = GyroZ;
    return GyroZ;
}

float MPU6050::ReadTemperature()
{
    return ((double) accel_t_gyro.value.temperature + 12412.0) / 340.0;
}

//MPU6050读取函数
//用于从I2C设备读取多个字节的数据
//使用函数Wire.endTransMission()来拉起或释放总线
void MPU6050::Read(int start, uint8_t *buffer, int size)
{
    //根据从机地址发起通信
    Wire.beginTransmission(MPU6050_I2C_ADDRESS);

    Wire.write(start);

    //用false作为参数，当数据发送完之后发送重启信号，并不释放I2C总线
    Wire.endTransmission(false);    // 拉起

    //向I2C设备请求读取size大小的数据，当数据完成读取之后释放I2C总线
    Wire.requestFrom(MPU6050_I2C_ADDRESS, size,true);

    int i = 0;
    while(Wire.available() && i<size)
    {
        buffer[i++]=Wire.read();//读取数据
    }
}

// MPU6050写入函数
//用于向I2C设备发送多个字节的数据
//参数:
//start : 起始地址，用来定义寄存器
//pData : 指向要写入数据的指针
//size: 要写入的字节数
void MPU6050::Write(int start, const uint8_t *pData, int size)
{
    Wire.beginTransmission(MPU6050_I2C_ADDRESS);
    Wire.write(start);        //发送起始地址
    Wire.write(pData, size);  //发送数据字节
    Wire.endTransmission(true); //释放I2C总线
}
