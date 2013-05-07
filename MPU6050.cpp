#include"Arduino.h"
#include <Wire.h>
#include "MPU6050.h"
#include "MPU6050_RegistersMap.h"

//MPU-6050 Ĭ��I2C��ַΪ0x68.
#define MPU6050_I2C_ADDRESS 0x68

MPU6050::MPU6050()
{

}

void MPU6050::Init()
{
    //MPU6050����ʱĬ�ϣ�
    //���������̣�+/- 250deg
    //���ٶȼ����̣�+/- 2g
    //ʹ���ڲ�8MHzʱ��Դ
    //����������˯��ģʽ��sleep mode��

    //���'sleep' λ������������
    uint8_t data = 0;
    Write(MPU6050_PWR_MGMT_1, &data, 1);
}

// ��ȡԭʼ��ֵ
// һ���Զ�ȡ�������ٶȡ����ٶȺ��¶����ڵ�14���ֽ�����
// ��MPU-6050��Ĭ����������û�д��˲��ģ������ֵ���Ǻ��ȶ�
void MPU6050::ReadData()
{
    Read(MPU6050_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro, sizeof(accel_t_gyro));

    //���Ӵ�����������ԭʼ���ݸߵͰ�λ�û�
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

float MPU6050::ReadPitchAngle()
{
    float xtoz = ReadAccX()*(1/ReadAccZ());
    float pitch = atan(xtoz)/3.14*180;
    return pitch;
}

float MPU6050::ReadRollAngle()
{
    float ytoz = ReadAccY()*(1/ReadAccZ());
    float roll = atan(ytoz)/3.14*180;
    return roll;
}

float MPU6050::ReadGyroX()
{
    return accel_t_gyro.value.x_gyro/131.0-1.4;
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

//MPU6050��ȡ����
//���ڴ�I2C�豸��ȡ����ֽڵ�����
//ʹ�ú���Wire.endTransMission()��������ͷ�����
void MPU6050::Read(int start, uint8_t *buffer, int size)
{
    //���ݴӻ���ַ����ͨ��
    Wire.beginTransmission(MPU6050_I2C_ADDRESS);

    Wire.write(start);

    //��false��Ϊ�����������ݷ�����֮���������źţ������ͷ�I2C����
    Wire.endTransmission(false);    // ����

    //��I2C�豸�����ȡsize��С�����ݣ���������ɶ�ȡ֮���ͷ�I2C����
    Wire.requestFrom(MPU6050_I2C_ADDRESS, size,true);

    int i = 0;
    while(Wire.available() && i<size)
    {
        buffer[i++]=Wire.read();//��ȡ����
    }
}

// MPU6050д�뺯��
//������I2C�豸���Ͷ���ֽڵ�����
//����:
//start : ��ʼ��ַ����������Ĵ���
//pData : ָ��Ҫд�����ݵ�ָ��
//size: Ҫд����ֽ���
void MPU6050::Write(int start, const uint8_t *pData, int size)
{
    Wire.beginTransmission(MPU6050_I2C_ADDRESS);
    Wire.write(start);        //������ʼ��ַ
    Wire.write(pData, size);  //���������ֽ�
    Wire.endTransmission(true); //�ͷ�I2C����
}
