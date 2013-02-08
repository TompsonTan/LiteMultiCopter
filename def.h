//检测设置
#if COPTERTEST == 1
  #define QUADP
  #define WMP
#elif defined(COPTERTEST)
  #error "*** this test is not yet defined"
#endif

//Arduino板的类型设置
#define MEGA

/**************************  all the Mega types  ***********************************/
#if defined(MEGA)
  #define LEDPIN_PINMODE             pinMode (13, OUTPUT);pinMode (30, OUTPUT);
  #define LEDPIN_TOGGLE              PINB  |= (1<<7); PINC  |= (1<<7);
  #define LEDPIN_ON                  PORTB |= (1<<7); PORTC |= (1<<7);
  #define LEDPIN_OFF                 PORTB &= ~(1<<7);PORTC &= ~(1<<7);

  #define POWERPIN_PINMODE           pinMode (37, OUTPUT);
  #define POWERPIN_ON                PORTC |= 1<<0;
  #define POWERPIN_OFF               PORTC &= ~(1<<0);

  #define I2C_PULLUPS_ENABLE         PORTD |= 1<<0; PORTD |= 1<<1;       // PIN 20&21 (SDA&SCL)
  #define I2C_PULLUPS_DISABLE        PORTD &= ~(1<<0); PORTD &= ~(1<<1);

  //RX PIN assignment inside the port //for PORTK
  #define THROTTLEPIN            0  //PIN 62 =  PIN A8   油门通道输入
  #define ROLLPIN                    1  //PIN 63 =  PIN A9    滚转通道输入
  #define PITCHPIN                  2  //PIN 64 =  PIN A10  俯仰通道输入
  #define YAWPIN                    3  //PIN 65 =  PIN A11  偏航通道输入

  #define PSENSORPIN                 A2    // Analog PIN 2
  #define PCINT_PIN_COUNT            8
  #define PCINT_RX_BITS              (1<<2),(1<<4),(1<<5),(1<<6),(1<<7),(1<<0),(1<<1),(1<<3)
  #define PCINT_RX_PORT              PORTK
  #define PCINT_RX_MASK              PCMSK2
  #define PCIR_PORT_BIT              (1<<2)
  #define RX_PC_INTERRUPT            PCINT2_vect    //中断向量
  #define RX_PCINT_PIN_PORT          PINK

  #define ISR_UART                   ISR(USART0_UDRE_vect)
#endif

//传感器类型
#if defined(MPU6050)
  #define GYRO 1
#endif

//多轴飞行器类型
#if defined(QUADP)
  #define MULTITYPE 2
#endif

//电机数目
#define NUMBER_MOTOR     4
