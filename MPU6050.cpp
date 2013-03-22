#include"Arduino.h"
#include <Wire.h>
#include "MPU6050.h"
#include "MPU6050_RegistersMap.h"

// Default I2C address for the MPU-6050 is 0x68.
// But only if the AD0 pin is low.
// Some sensor boards have AD0 high, and the
// I2C address thus becomes 0x69.
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

    // According to the datasheet, the 'sleep' bit
    // should read a '1'. But I read a '0'.
    // That bit has to be cleared, since the sensor
    // is in sleep mode at power-up. Even if the
    // bit reads '0'.
    error = Read (MPU6050_PWR_MGMT_2, &c, 1);
    Serial.print(F("PWR_MGMT_2 : "));
    Serial.print(c,HEX);
    Serial.print(F(", error = "));
    Serial.println(error,DEC);


    // Clear the 'sleep' bit to start the sensor.
    WriteRegister(MPU6050_PWR_MGMT_1, 0);
}

int MPU6050::ReadData()
{
    // 读取原始数值
    // 一次性读取包括加速度、角速度和温度在内的14个字节数据
    // 在MPU-6050的默认设置下是没有打开滤波的，因此数值不是很稳定
    int error;
    error = Read(MPU6050_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro, sizeof(accel_t_gyro));

    uint8_t swap;
#define SWAP(x,y) swap = x; x = y; y = swap
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

// MPU6050_read
//
// This is a common function to read multiple bytes
// from an I2C device.
//
// It uses the boolean parameter for Wire.endTransMission()
// to be able to hold or release the I2C-bus.
// This is implemented in Arduino 1.0.1.
//
// Only this function is used to read.
// There is no function for a single byte.
//
int MPU6050::Read(int start, uint8_t *buffer, int size)
{
    int i, n, error;

    Wire.beginTransmission(MPU6050_I2C_ADDRESS);
    n = Wire.write(start);
    if (n != 1)
        return (-10);

    n = Wire.endTransmission(false);    // hold the I2C-bus
    if (n != 0)
        return (n);

    // Third parameter is true: relase I2C-bus after data is read.
    Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
    i = 0;
    while(Wire.available() && i<size)
    {
        buffer[i++]=Wire.read();
    }
    if ( i != size)
        return (-11);

    return (0);  // return : no error
}

// MPU6050_write
//
// This is a common function to write multiple bytes to an I2C device.
//
// If only a single register is written,
// use the function MPU_6050_write_reg().
//
// Parameters:
//   start : Start address, use a define for the register
//   pData : A pointer to the data to write.
//   size  : The number of bytes to write.
//
// If only a single register is written, a pointer
// to the data has to be used, and the size is
// a single byte:
//   int data = 0;        // the data to write
//   MPU6050_write (MPU6050_PWR_MGMT_1, &c, 1);
int MPU6050::Write(int start, const uint8_t *pData, int size)
{
    int n, error;

    Wire.beginTransmission(MPU6050_I2C_ADDRESS);
    n = Wire.write(start);        // write the start address
    if (n != 1)
        return (-20);

    n = Wire.write(pData, size);  // write data bytes
    if (n != size)
        return (-21);

    error = Wire.endTransmission(true); // release the I2C-bus
    if (error != 0)
        return (error);

    return (0);         // return : no error
}

// MPU6050_write_reg
//
// An extra function to write a single register.
// It is just a wrapper around the MPU_6050_write()
// function, and it is only a convenient function
// to make it easier to write a single register.
int MPU6050::WriteRegister(int reg, uint8_t data)
{
    int error;
    error = Write(reg, &data, 1);
    return (error);
}
