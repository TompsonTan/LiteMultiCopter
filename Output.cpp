
//电机引脚定义
#if defined(MEGA)
  uint8_t PWM_PIN[4] = {3,5,6,2};      //for a quad+: rear,right,left,front
#endif


/**************************************************************************************/
/************  Writes the Motors values to the PWM compare register  ******************/
/**************************************************************************************/
void writeMotors()
{
  #if defined(MEGA)
        OCR3C = motor[0]<<3; //  pin 3
        OCR3A = motor[1]<<3; //  pin 5
        OCR4A = motor[2]<<3; //  pin 6
        OCR3B = motor[3]<<3; //  pin 2
  #endif
}

/**************************************************************************************/
/************          Writes the mincommand to all Motors           ******************/
/**************************************************************************************/
void writeAllMotors(int16_t mc)
 {
// Sends commands to all motors
for (uint8_t i =0;i<NUMBER_MOTOR;i++)
{
    motor[i]=mc;
}
  writeMotors();
}

/**************************************************************************************/
/************        Initialize the PWM Timers and Registers         ******************/
/**************************************************************************************/
void initOutput()
{

//设置输出引脚
  for(uint8_t i=0;i<NUMBER_MOTOR;i++)
{
    pinMode(PWM_PIN[i],OUTPUT);
}

/****************  Specific PWM Timers & Registers for the MEGA's    ******************/
  #if defined(MEGA)
      // init 16bit timer 3
      TCCR3A |= (1<<WGM31); // phase correct mode
      TCCR3A &= ~(1<<WGM30);
      TCCR3B |= (1<<WGM33);
      TCCR3B &= ~(1<<CS31); // no prescaler
      ICR3   |= 0x3FFF; // TOP to 16383;

      TCCR3A |= _BV(COM3C1); // connect pin 3 to timer 3 channel C
      TCCR3A |= _BV(COM3A1); // connect pin 5 to timer 3 channel A

      // init 16bit timer 4
      TCCR4A |= (1<<WGM41); // phase correct mode
      TCCR4A &= ~(1<<WGM40);
      TCCR4B |= (1<<WGM43);
      TCCR4B &= ~(1<<CS41); // no prescaler
      ICR4   |= 0x3FFF; // TOP to 16383;

      TCCR4A |= _BV(COM4A1); // connect pin 6 to timer 4 channel A
      TCCR3A |= _BV(COM3B1); // connect pin 2 to timer 3 channel B
  #endif


 /********  special version of MultiWii to calibrate all attached ESCs ************/
  #if defined(ESC_CALIB_CANNOT_FLY)
    writeAllMotors(ESC_CALIB_HIGH);
    delay(3000);
    writeAllMotors(ESC_CALIB_LOW);
    delay(500);
    while (1) {
      delay(5000);
      blinkLED(2,20, 2);
    }
    exit; // statement never reached
  #endif

  writeAllMotors(MINCOMMAND);
  delay(300);
}


/**************************************************************************************/
/********** Mixes the Computed stabilize values to the Motors & Servos  ***************/
/**************************************************************************************/
void mixTable() {
  int16_t maxMotor;
  uint8_t i;

  #define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL]*X + axisPID[PITCH]*Y + YAW_DIRECTION * axisPID[YAW]*Z

  #if NUMBER_MOTOR > 3
    //prevent "yaw jump" during yaw correction
    axisPID[YAW] = constrain(axisPID[YAW],-100-abs(rcCommand[YAW]),+100+abs(rcCommand[YAW]));
  #endif
  /****************                   main Mix Table                ******************/
  #ifdef QUADP
    motor[0] = PIDMIX( 0,+1,-1); //REAR
    motor[1] = PIDMIX(-1, 0,+1); //RIGHT
    motor[2] = PIDMIX(+1, 0,+1); //LEFT
    motor[3] = PIDMIX( 0,-1,-1); //FRONT
  #endif
  #ifdef QUADX
    motor[0] = PIDMIX(-1,+1,-1); //REAR_R
    motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
    motor[2] = PIDMIX(+1,+1,+1); //REAR_L
    motor[3] = PIDMIX(+1,-1,-1); //FRONT_L
  #endif
}
