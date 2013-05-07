#include"Arduino.h"
#include <Wire.h>

#include"MPU6050.h"
#include"SerialCom.h"
#include"Receiver.h"
#include"Motor.h"

#define USE_SERIAL 1


MPU6050  LMC_Sensor;//���ٶ�/�����Ǵ�����
SerialCom LMC_Com;//����ͨ��
Receiver    LMC_Receiver;//���ջ��źŶ�ȡ
Motor       LMC_Motor;//�������

//�жϷ����������������ջ��ź�
/*�����ֽ��ջ�ͨ����ƽ�仯ʱ�����жϵ�ǰ�ġ����̡���
ִ���жϷ���������ȡ���ջ����źţ��������󷵻�*/
SIGNAL(PCINT2_vect)
{
    LMC_Receiver.MegaPcIntISR();
}

//�����ǻ�׼����
#define GYROBASECNT 200
int GyroBaseCnt=0;			//Gyro base build count �������е㽨��������

//��������/��������
bool InLock = true;//��ʼΪ����״̬
int ArmCnt = 0;	//����/��������
#define ARMING_TIME 250
#define STICKGATE 90//����/������ֵ
int LockLED = A5;//����������Ϊ��������˸����Ϊ�ѽ���
void ArmingRoutine()
{
    //������,�����ﵽ��ֵ��ʱ����Ϊִ�����������
    if(abs(LMC_Receiver.RxRud)>85)
        ArmCnt++;
    else
        ArmCnt = 0;

    if(ArmCnt>ARMING_TIME)
    {
        if(InLock)
        {
            if(LMC_Receiver.RxRud>STICKGATE)//��������ң�����
            {
                InLock = false;
                GyroBaseCnt=GYROBASECNT;
            }
        }
        else
        {
            if(LMC_Receiver.RxRud<-STICKGATE)//�������������
                InLock = true;
        }
    }

}

bool ESC_isCalibrated = false;
void CalibrateThrottle()
{
    if(!ESC_isCalibrated)
    {
        if(LMC_Receiver.RxThr>85)
        {
            InLock = false;
            ESC_isCalibrated = true;
        }
    }
}

void setup()
{
    //��ʼ��I2C����
    Wire.begin();

//#if(USE_SERIAL)
    //��ʼ������ͨ��
    LMC_Com.Init();
    //������ʼ������
    Serial.begin(9600);
//#endif

    //��ʼ��MPU6050
    LMC_Sensor.Init();

    //���ý��ջ��źŶ˿�
    LMC_Receiver.Init();

    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);

    pinMode(LockLED,OUTPUT);
    digitalWrite(LockLED,LOW);

    LMC_Motor.CalibrateESCs();
}

void loop()
{
    //��ȡ/��������������
    LMC_Sensor.ReadData();

    //��ȡ/���½��ջ��ź�
    LMC_Receiver.ReadData();

    //�������Ǻͽ��ջ��źŷ��͵�����
    //LMC_Com.DataToPC(LMC_Sensor,LMC_Receiver);


    /******�������PID�㷨�͸���PID�Ľ�������Ƶ���ĺ�����******/

    if(LMC_Receiver.RxThr<1200)//�����ŵ���10%����������/��������
        ArmingRoutine();

    //��ȡ���ջ��źţ������ת������Ƶ��
    LMC_Motor.CalculateOutput(LMC_Sensor,LMC_Receiver);

    //����������δ���������ǻ�׼����ֹ������
    if(InLock)
    {
        LMC_Motor.Lock();
        digitalWrite(LockLED,LOW);
    }
    else
    {
        digitalWrite(LockLED,HIGH);
        //�����������ź�
        LMC_Motor.OutPut();
    }

   Serial.println(LMC_Motor.Roll_Offset);
//    Serial.println(30*LMC_Receiver.RxAil/100.0);
    //Serial.println(LMC_Sensor.ReadAccX());
//Serial.println(LMC_Sensor.ReadAccZ());
  // Serial.println(LMC_Sensor.ReadPitchAngle());

    //delay(500);
}
