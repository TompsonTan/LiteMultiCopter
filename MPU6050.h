#ifndef MPU6050_H_INCLUDED
#define MPU6050_H_INCLUDED

#include <Wire.h>

//声明一个联合体存放寄存器地址和测量数据。
//但测出的数据和AVR的数据方式不是一致的。
//AVR（Arduino的芯片）将字节低位（Low Byte）储存在地址低位（lower address）。
//而MPU6050则相反，因此要将读出的数据高地位对换。

//不要随便更改下面的AccGyo，因为读取数据是一次性读取的，各个参数的内存地址要“紧密相连”
union AccGyo
{
    struct
    {
        uint8_t x_accel_h;
        uint8_t x_accel_l;
        uint8_t y_accel_h;
        uint8_t y_accel_l;
        uint8_t z_accel_h;
        uint8_t z_accel_l;
        uint8_t t_h;
        uint8_t t_l;
        uint8_t x_gyro_h;
        uint8_t x_gyro_l;
        uint8_t y_gyro_h;
        uint8_t y_gyro_l;
        uint8_t z_gyro_h;
        uint8_t z_gyro_l;
    } reg;
    struct
    {
        int x_accel;
        int y_accel;
        int z_accel;
        int temperature;
        int x_gyro;
        int y_gyro;
        int z_gyro;
    } value;
};

//该类用于检测MPU6050传感器的数据
class MPU6050
{
public:
    MPU6050();
    void Init();

    void ReadData();

    //读取加速度
    float ReadAccX();
    float ReadAccY();
    float ReadAccZ();

    //读取角速度
    float ReadGyroX();
    float ReadGyroY();
    float ReadGyroZ();

    //读取温度
    float ReadTemperature();

private:
    AccGyo accel_t_gyro;
    void Read(int start, uint8_t *buffer, int size);
    void Write(int start, const uint8_t *pData, int size);
};

#endif // MPU6050_H_INCLUDED
