#define analogPin 9        #��9�ŹܽŶ���Ϊ����ܽ�

int val;                   #��ӱ���

void setup()
 {
   Serial.begin(9600);     #��ʼ������
 }

void loop()
{
  
   for(val=43;val<=255;val=val+1)
   {
     analogWrite(analogPin,val)�� 
   }
   
   Serial.println(val);           
   #������ֵ
   for(val=255;val>=43;val=val-1)
   {
     analogWrite(analogPin,val);
   }  
   
  Serial.println(val);          
  #�����Сֵ
}