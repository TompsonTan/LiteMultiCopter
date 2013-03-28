/****************
 单个电机测试实验
 ****************/


#define analogPin 9        //将9号管脚定义为输出管脚

int val;                   //添加变量

void setup()
 {
   Serial.begin(9600);     //起始波特率
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
