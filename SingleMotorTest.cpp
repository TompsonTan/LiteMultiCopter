/****************
 单个电机测试实验
 ****************/

//9号管脚为输出管脚
#define analogPin 9

//添加变量
int val;

void setup()
 {
     //设置起始波特率
    Serial.begin(9600);
 }

void loop()
{

   for(val=43;val<=255;val=val+1)
   {
     analogWrite(analogPin,val)；
   }

   Serial.println(val);
   //输出最大值
   for(val=255;val>=43;val=val-1)
   {
     analogWrite(analogPin,val);
   }

  Serial.println(val);
  //输出最小值
}
