
/****参数配置文件****/

//QUADP四轴、十字形
#define QUADP

//电机最小油门(怠速)
#define MINTHROTTLE 1150

// 电机最大油门(最多可以设到2000)
#define MAXTHROTTLE 1850

//电调行程最低点
#define MINCOMMAND  1000

// I2C 信号频率
#define I2C_SPEED 100000L

//使用内部I2C，如果是MEGA板，也就是20,21引脚（对应SDA,SCL）
#define INTERNAL_I2C_PULLUPS

//传感器
#define MPU6050       //combo + ACC

//传感器的坐标系设置
#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  Y; accADC[PITCH]  = -X; accADC[YAW]  = Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = -Y; gyroADC[PITCH] =  X; gyroADC[YAW] = Z;}


//下面三行和串口通信有关
#define SERIAL_COM_SPEED 115200//串口传输速度
#define INTERLEAVING_DELAY 3000
#define NEUTRALIZE_DELAY 100000

//安全模式
//#define FAILSAFE
#define FAILSAVE_DELAY     10
#define FAILSAVE_OFF_DELAY 200
#define FAILSAVE_THROTTLE  (MINTHROTTLE + 200)

//俯仰、滚转、偏航的摇杆死区
//#define DEADBAND 6

//悬停油门的中性区域
//#define ALT_HOLD_THROTTLE_NEUTRAL_ZONE 20


//电机、舵机和其它设置项
//启用这个解锁后电机不转，而不是怠速
#define MOTOR_STOP
//有些遥控接收装置，其信号的中性点不在1500，这里可以把它们改为1500。
#define MIDRC 1500

//电调校正，校正方法见http://code.google.com/p/multiwii/wiki/ESCsCalibration
#define ESC_CALIB_LOW  MINCOMMAND
#define ESC_CALIB_HIGH 2000
//#define ESC_CALIB_CANNOT_FLY  // uncomment to activate

// 回归测试
//#define COPTERTEST 1
