#include"SerialCom.h"

SerialCom::SerialCom()
{
    //msg = 1;
}

void SerialCom::Init()
{
    //初始化串口通信，波特率
    Serial.begin(9600);
}

void SerialCom::DataToPC(MPU6050 MySensor,Receiver MyReceiver)
{
    SensorDataToPC(MySensor);
//    ChannelDataToPC(MyReceiver);
//
//    String comdata = "";
//    while(Serial.available()>0)
//    {
//        msg = Serial.read();
//        comdata +=char(Serial.read());
//        Serial.print(msg);
//        Serial.println(msg,DEC);
//    }
//    if (comdata.length() >0)
//    {
//        Serial.print(comdata);
//        Serial.println(F(""));
//
//        comdata.trim();
//        msg = comdata.toInt();
//        Serial.print(msg);
//        String hhh = " ";
//        Serial.println(hhh);
//    }
//    Serial.println("enter 4 letter stream indentifier");
}

void SerialCom::SensorDataToPC(MPU6050 Sensor)
{
    Serial.println(F(""));
    Serial.println(F("MPU-6050"));

    Serial.print(F("accel x,y,z: "));
    Serial.print(Sensor.ReadAccX(), 3);
    Serial.print(F(", "));
    Serial.print(Sensor.ReadAccY(), 3);
    Serial.print(F(", "));
    Serial.print(Sensor.ReadAccZ(), 3);
    Serial.println(F(""));

    Serial.print(F("temperature: "));
    double dT = Sensor.ReadTemperature();
    Serial.print(dT, 3);
    Serial.print(F(" degrees Celsius"));
    Serial.println(F(""));

    Serial.print(F("gyro x,y,z : "));
    Serial.print(Sensor.ReadGyroX(), 3);
    Serial.print(F(", "));
    Serial.print(Sensor.ReadGyroY(), 3);
    Serial.print(F(", "));
    Serial.print(Sensor.ReadGyroZ(), 3);
    Serial.print(F(", "));
    Serial.println(F(""));
}

void SerialCom::ChannelDataToPC(Receiver MyReceiver)
{
//    Serial.print(F("axis 0: "));
//    Serial.print(MyReceiver.receiverCommand[0]);
//    Serial.println(F(""));
//
//    Serial.print(F("axis 1: "));
//    Serial.print(MyReceiver.receiverCommand[1]);
//    Serial.println(F(""));
//
//    Serial.print(F("axis 2: "));
//    Serial.print(MyReceiver.receiverCommand[2]);
//    Serial.println(F(""));

//       Serial.print(F("Throttle: "));
//    Serial.print(MyReceiver.ChannelData[3]);
//    Serial.println(F(""));
}
