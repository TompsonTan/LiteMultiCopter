/*
	本程序给予MWC飞控2.1，为自由软件，使用GPL V3授权协议，你可以协议下修改和发布代码。
*/

#include <avr/io.h>

#include "config.h"
#include "def.h"

#include <avr/pgmspace.h>

/*********** RC alias *****************/
#define ROLL       0
#define PITCH      1
#define YAW        2
#define THROTTLE   3
#define AUX1       4
#define AUX2       5
#define AUX3       6
#define AUX4       7

#define PIDALT     3
#define PIDPOS     4
#define PIDPOSR    5
#define PIDNAVR    6
#define PIDLEVEL   7
#define PIDMAG     8
#define PIDVEL     9 // not used currently

#define BOXACC       0
#define BOXBARO      1
#define BOXMAG       2
#define BOXCAMSTAB   3
#define BOXCAMTRIG   4
#define BOXARM       5
#define BOXGPSHOME   6
#define BOXGPSHOLD   7
#define BOXPASSTHRU  8
#define BOXHEADFREE  9
#define BOXBEEPERON  10
#define BOXLEDMAX    11 // we want maximum illumination
#define BOXLLIGHTS   12 // enable landing lights at any altitude
#define BOXHEADADJ   13 // acquire heading for HEADFREE mode

#define PIDITEMS 10
#define CHECKBOXITEMS 14

const char boxnames[] PROGMEM = // names for dynamic generation of config GUI
  "ACC;"
  "BARO;"
  "MAG;"
  "CAMSTAB;"
  "CAMTRIG;"
  "ARM;"
  "GPS HOME;"
  "GPS HOLD;"
  "PASSTHRU;"
  "HEADFREE;"
  "BEEPER;"
  "LEDMAX;"
  "LLIGHTS;"
  "HEADADJ;"
;

const char pidnames[] PROGMEM =
  "ROLL;"
  "PITCH;"
  "YAW;"
  "ALT;"
  "Pos;"
  "PosR;"
  "NavR;"
  "LEVEL;"
  "MAG;"
  "VEL;"
;

static uint32_t currentTime = 0;
static uint16_t previousTime = 0;
static uint16_t cycleTime = 0;     // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
static uint16_t calibratingA = 0;  // the calibration is done in the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
static uint16_t calibratingG;
static uint16_t acc_1G;             // this is the 1G measured acceleration
static int16_t  acc_25deg;
static int16_t  headFreeModeHold;
static int16_t  gyroADC[3],accADC[3],accSmooth[3],magADC[3];
static int16_t  heading,magHold;
static uint8_t  vbat;               // battery voltage in 0.1V steps
static uint8_t  rcOptions[CHECKBOXITEMS];
static int32_t  BaroAlt;
static int32_t  EstAlt;             // in cm
static int16_t  BaroPID = 0;
static int32_t  AltHold;
static int16_t  errorAltitudeI = 0;

#if defined(ARMEDTIMEWARNING)
  static uint32_t  ArmedTimeWarningMicroSeconds = 0;
#endif

static int16_t  debug[4];
static int16_t  sonarAlt; //to think about the unit

struct flags_struct {
  uint8_t OK_TO_ARM :1 ;
  uint8_t ARMED :1 ;
  uint8_t I2C_INIT_DONE :1 ; // For i2c gps we have to now when i2c init is done, so we can update parameters to the i2cgps from eeprom (at startup it is done in setup())
  uint8_t ACC_CALIBRATED :1 ;
  uint8_t NUNCHUKDATA :1 ;
  uint8_t ACC_MODE :1 ;
  uint8_t MAG_MODE :1 ;
  uint8_t BARO_MODE :1 ;
  uint8_t GPS_HOME_MODE :1 ;
  uint8_t GPS_HOLD_MODE :1 ;
  uint8_t HEADFREE_MODE :1 ;
  uint8_t PASSTHRU_MODE :1 ;
  uint8_t GPS_FIX :1 ;
  uint8_t GPS_FIX_HOME :1 ;
  uint8_t SMALL_ANGLES_25 :1 ;
  uint8_t CALIBRATE_MAG :1 ;
} f;


static int16_t  i2c_errors_count = 0;
static int16_t  annex650_overrun_count = 0;


// ******************
// rc functions
// ******************
#define MINCHECK 1100
#define MAXCHECK 1900

static int16_t failsafeEvents = 0;
volatile int16_t failsafeCnt = 0;

static int16_t rcData[8];          // interval [1000;2000]
static int16_t rcCommand[4];       // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW
static int16_t lookupPitchRollRC[6];// lookup table for expo & RC rate PITCH+ROLL
static int16_t lookupThrottleRC[11];// lookup table for expo & mid THROTTLE
volatile uint8_t rcFrameComplete; // for serial rc receiver Spektrum


// **************
// gyro+acc IMU
// **************
static int16_t gyroData[3] = {0,0,0};
static int16_t gyroZero[3] = {0,0,0};
static int16_t angle[2]    = {0,0};  // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800

// *************************
// 电机控制相关
// *************************
static int16_t axisPID[3];
static int16_t motor[NUMBER_MOTOR];

// ************************
// EEPROM Layout definition
// ************************
static uint8_t dynP8[3], dynD8[3];
static struct {
  uint8_t checkNewConf;
  uint8_t P8[PIDITEMS], I8[PIDITEMS], D8[PIDITEMS];
  uint8_t rcRate8;
  uint8_t rcExpo8;
  uint8_t rollPitchRate;
  uint8_t yawRate;
  uint8_t dynThrPID;
  uint8_t thrMid8;
  uint8_t thrExpo8;
  int16_t accZero[3];
  int16_t magZero[3];
  int16_t angleTrim[2];
  uint16_t activate[CHECKBOXITEMS];
  uint8_t powerTrigger1;
} conf;

void blinkLED(uint8_t num, uint8_t wait,uint8_t repeat) {
  uint8_t i,r;
  for (r=0;r<repeat;r++) {
    for(i=0;i<num;i++) {
      #if defined(LED_FLASHER)
        switch_led_flasher(1);
      #endif
      LEDPIN_TOGGLE; // switch LEDPIN state
      BUZZERPIN_ON;
      delay(wait);
      BUZZERPIN_OFF;
      #if defined(LED_FLASHER)
        switch_led_flasher(0);
      #endif
    }
    delay(60);
  }
}

void annexCode() { // this code is excetuted at each loop and won't interfere with control loop if it lasts less than 650 microseconds
  static uint32_t calibratedAccTime;
  uint16_t tmp,tmp2;
  uint8_t axis,prop1,prop2;

  #define BREAKPOINT 1500
  // PITCH & ROLL only dynamic PID adjustemnt,  depending on throttle value
  if   (rcData[THROTTLE]<BREAKPOINT) {
    prop2 = 100;
  } else {
    if (rcData[THROTTLE]<2000) {
      prop2 = 100 - (uint16_t)conf.dynThrPID*(rcData[THROTTLE]-BREAKPOINT)/(2000-BREAKPOINT);
    } else {
      prop2 = 100 - conf.dynThrPID;
    }
  }

  for(axis=0;axis<3;axis++) {
    tmp = min(abs(rcData[axis]-MIDRC),500);
    #if defined(DEADBAND)
      if (tmp>DEADBAND) { tmp -= DEADBAND; }
      else { tmp=0; }
    #endif
    if(axis!=2) { //ROLL & PITCH
      tmp2 = tmp/100;
      rcCommand[axis] = lookupPitchRollRC[tmp2] + (tmp-tmp2*100) * (lookupPitchRollRC[tmp2+1]-lookupPitchRollRC[tmp2]) / 100;
      prop1 = 100-(uint16_t)conf.rollPitchRate*tmp/500;
      prop1 = (uint16_t)prop1*prop2/100;
    } else {      // YAW
      rcCommand[axis] = tmp;
      prop1 = 100-(uint16_t)conf.yawRate*tmp/500;
    }
    dynP8[axis] = (uint16_t)conf.P8[axis]*prop1/100;
    dynD8[axis] = (uint16_t)conf.D8[axis]*prop1/100;
    if (rcData[axis]<MIDRC) rcCommand[axis] = -rcCommand[axis];
  }
  tmp = constrain(rcData[THROTTLE],MINCHECK,2000);
  tmp = (uint32_t)(tmp-MINCHECK)*1000/(2000-MINCHECK); // [MINCHECK;2000] -> [0;1000]
  tmp2 = tmp/100;
  rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp-tmp2*100) * (lookupThrottleRC[tmp2+1]-lookupThrottleRC[tmp2]) / 100; // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]

  if(f.HEADFREE_MODE) { //to optimize
    float radDiff = (heading - headFreeModeHold) * 0.0174533f; // where PI/180 ~= 0.0174533
    float cosDiff = cos(radDiff);
    float sinDiff = sin(radDiff);
    int16_t rcCommand_PITCH = rcCommand[PITCH]*cosDiff + rcCommand[ROLL]*sinDiff;
    rcCommand[ROLL] =  rcCommand[ROLL]*cosDiff - rcCommand[PITCH]*sinDiff;
    rcCommand[PITCH] = rcCommand_PITCH;
  }


  if ( (calibratingA>0 && ACC ) || (calibratingG>0) ) { // Calibration phasis
    LEDPIN_TOGGLE;
  } else {
    if (f.ACC_CALIBRATED) {LEDPIN_OFF;}
    if (f.ARMED) {LEDPIN_ON;}
  }

  #if defined(LED_FLASHER)
    auto_switch_led_flasher();
  #endif

  if ( currentTime > calibratedAccTime ) {
    if (! f.SMALL_ANGLES_25) {
      // the multi uses ACC and is not calibrated or is too much inclinated
      f.ACC_CALIBRATED = 0;
      LEDPIN_TOGGLE;
      calibratedAccTime = currentTime + 500000;
    } else {
      f.ACC_CALIBRATED = 1;
    }
  }

  #if defined(LOG_VALUES) && (LOG_VALUES == 2)
    if (cycleTime > cycleTimeMax) cycleTimeMax = cycleTime; // remember highscore
    if (cycleTime < cycleTimeMin) cycleTimeMin = cycleTime; // remember lowscore
  #endif
}

void setup() {
  LEDPIN_PINMODE;
  POWERPIN_PINMODE;
  BUZZERPIN_PINMODE;
  STABLEPIN_PINMODE;
  POWERPIN_OFF;
  initOutput();
  readEEPROM();
  checkFirstTime();
  configureReceiver();

  initSensors();

  previousTime = micros();
  calibratingG = 400;
  #if defined(ARMEDTIMEWARNING)
    ArmedTimeWarningMicroSeconds = (ARMEDTIMEWARNING *1000000);
  #endif
  /************************************/
  ADCSRA |= _BV(ADPS2) ; ADCSRA &= ~_BV(ADPS1); ADCSRA &= ~_BV(ADPS0); // this speeds up analogRead without loosing too much resolution: http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1208715493/11
  #if defined(LED_FLASHER)
    init_led_flasher();
    led_flasher_set_sequence(LED_FLASHER_SEQUENCE);
  #endif
  f.SMALL_ANGLES_25=1; // important for gyro only conf
}

// ******** Main Loop 主循环*********
void loop () {
  static uint8_t rcDelayCommand; // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
  uint8_t axis,i;
  int16_t error,errorAngle;
  int16_t delta,deltaSum;
  int16_t PTerm,ITerm,DTerm;
  static int16_t lastGyro[3] = {0,0,0};
  static int16_t delta1[3],delta2[3];
  static int16_t errorGyroI[3] = {0,0,0};
  static int16_t errorAngleI[2] = {0,0};
  static uint32_t rcTime  = 0;
  static int16_t initialThrottleHold;

  #define RC_FREQ 50

  if (currentTime > rcTime ) { // 50Hz
    rcTime = currentTime + 20000;
    computeRC();
    // Failsafe routine - added by MIS
    #if defined(FAILSAFE)
      if ( failsafeCnt > (5*FAILSAVE_DELAY) && f.ARMED) {                  // Stabilize, and set Throttle to specified level
        for(i=0; i<3; i++) rcData[i] = MIDRC;                               // after specified guard time after RC signal is lost (in 0.1sec)
        rcData[THROTTLE] = FAILSAVE_THROTTLE;
        if (failsafeCnt > 5*(FAILSAVE_DELAY+FAILSAVE_OFF_DELAY)) {          // Turn OFF motors after specified Time (in 0.1sec)
          f.ARMED = 0;   // This will prevent the copter to automatically rearm if failsafe shuts it down and prevents
          f.OK_TO_ARM = 0; // to restart accidentely by just reconnect to the tx - you will have to switch off first to rearm
        }
        failsafeEvents++;
      }
      if ( failsafeCnt > (5*FAILSAVE_DELAY) && !f.ARMED) {  //Turn of "Ok To arm to prevent the motors from spinning after repowering the RX with low throttle and aux to arm
          f.ARMED = 0;   // This will prevent the copter to automatically rearm if failsafe shuts it down and prevents
          f.OK_TO_ARM = 0; // to restart accidentely by just reconnect to the tx - you will have to switch off first to rearm
      }
      failsafeCnt++;
    #endif
    // end of failsave routine - next change is made with RcOptions setting

    if (rcData[THROTTLE] < MINCHECK) {
      errorGyroI[ROLL] = 0; errorGyroI[PITCH] = 0; errorGyroI[YAW] = 0;
      errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
      rcDelayCommand++;
      if (rcData[YAW] < MINCHECK && rcData[PITCH] < MINCHECK && !f.ARMED) {
        if (rcDelayCommand == 20) {
          calibratingG=400;
        }
      } else if (rcData[YAW] > MAXCHECK && rcData[PITCH] > MAXCHECK && !f.ARMED) {
        if (rcDelayCommand == 20) {
          previousTime = micros();
        }
      }
      else if (conf.activate[BOXARM] > 0) {
        if ( rcOptions[BOXARM] && f.OK_TO_ARM
        #if defined(FAILSAFE)
          && failsafeCnt <= 1
        #endif
        ) {
	  f.ARMED = 1;
	  headFreeModeHold = heading;
        } else if (f.ARMED) f.ARMED = 0;
        rcDelayCommand = 0;
      #ifdef ALLOW_ARM_DISARM_VIA_TX_YAW
      } else if ( (rcData[YAW] < MINCHECK )  && f.ARMED) {
        if (rcDelayCommand == 20) f.ARMED = 0; // rcDelayCommand = 20 => 20x20ms = 0.4s = time to wait for a specific RC command to be acknowledged
      } else if ( (rcData[YAW] > MAXCHECK ) && rcData[PITCH] < MAXCHECK && !f.ARMED && calibratingG == 0 && f.ACC_CALIBRATED) {
        if (rcDelayCommand == 20) {
	  f.ARMED = 1;
	  headFreeModeHold = heading;
        }
      #endif
      #ifdef ALLOW_ARM_DISARM_VIA_TX_ROLL
      } else if ( (rcData[ROLL] < MINCHECK)  && f.ARMED) {
        if (rcDelayCommand == 20) f.ARMED = 0; // rcDelayCommand = 20 => 20x20ms = 0.4s = time to wait for a specific RC command to be acknowledged
      } else if ( (rcData[ROLL] > MAXCHECK) && rcData[PITCH] < MAXCHECK && !f.ARMED && calibratingG == 0 && f.ACC_CALIBRATED) {
        if (rcDelayCommand == 20) {
          f.ARMED = 1;
          headFreeModeHold = heading;
        }
      #endif
      } else
        rcDelayCommand = 0;
    } else if (rcData[THROTTLE] > MAXCHECK && !f.ARMED) {
      if (rcData[YAW] < MINCHECK && rcData[PITCH] < MINCHECK) {        // throttle=max, yaw=left, pitch=min
        if (rcDelayCommand == 20) calibratingA=400;
        rcDelayCommand++;
      } else if (rcData[YAW] > MAXCHECK && rcData[PITCH] < MINCHECK) { // throttle=max, yaw=right, pitch=min
        if (rcDelayCommand == 20) f.CALIBRATE_MAG = 1; // MAG calibration request
        rcDelayCommand++;
      } else if (rcData[PITCH] > MAXCHECK) {
         conf.angleTrim[PITCH]+=2;writeParams(1);
         #if defined(LED_RING)
           blinkLedRing();
         #endif
      } else if (rcData[PITCH] < MINCHECK) {
         conf.angleTrim[PITCH]-=2;writeParams(1);
         #if defined(LED_RING)
           blinkLedRing();
         #endif
      } else if (rcData[ROLL] > MAXCHECK) {
         conf.angleTrim[ROLL]+=2;writeParams(1);
         #if defined(LED_RING)
           blinkLedRing();
         #endif
      } else if (rcData[ROLL] < MINCHECK) {
         conf.angleTrim[ROLL]-=2;writeParams(1);
         #if defined(LED_RING)
           blinkLedRing();
         #endif
      } else {
        rcDelayCommand = 0;
      }
    }
    #if defined(LED_FLASHER)
      led_flasher_autoselect_sequence();
    #endif

    uint16_t auxState = 0;
    for(i=0;i<4;i++)
      auxState |= (rcData[AUX1+i]<1300)<<(3*i) | (1300<rcData[AUX1+i] && rcData[AUX1+i]<1700)<<(3*i+1) | (rcData[AUX1+i]>1700)<<(3*i+2);
    for(i=0;i<CHECKBOXITEMS;i++)
      rcOptions[i] = (auxState & conf.activate[i])>0;

    // note: if FAILSAFE is disable, failsafeCnt > 5*FAILSAVE_DELAY is always false
    if (( rcOptions[BOXACC] || (failsafeCnt > 5*FAILSAVE_DELAY) ) && ACC ) {
      // bumpless transfer to Level mode
      if (!f.ACC_MODE) {
        errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
        f.ACC_MODE = 1;
      }
    } else {
      // failsafe support
      f.ACC_MODE = 0;
    }

    if (rcOptions[BOXARM] == 0) f.OK_TO_ARM = 1;
    if (f.ACC_MODE) {STABLEPIN_ON;} else {STABLEPIN_OFF;}

    if (rcOptions[BOXPASSTHRU]) {f.PASSTHRU_MODE = 1;}
    else {f.PASSTHRU_MODE = 0;}

  } else { // not in rc loop
    static uint8_t taskOrder=0; // never call all functions in the same loop, to avoid high delay spikes
  }

  computeIMU();
  // Measure loop rate just afer reading the sensors
  currentTime = micros();
  cycleTime = currentTime - previousTime;
  previousTime = currentTime;


  //**** 俯仰、滚转和偏航的PID算法
  for(axis=0;axis<3;axis++)
    {
        if (f.ACC_MODE && axis<2 )
        { //LEVEL MODE
          // 50 degrees max inclination
          errorAngle = constrain(2*rcCommand[axis] + GPS_angle[axis],-500,+500) - angle[axis] + conf.angleTrim[axis]; //16 bits is ok here

          PTerm      = (int32_t)errorAngle*conf.P8[PIDLEVEL]/100 ;                          // 32 bits is needed for calculation: errorAngle*P8[PIDLEVEL] could exceed 32768   16 bits is ok for result

          PTerm = constrain(PTerm,-conf.D8[PIDLEVEL]*5,+conf.D8[PIDLEVEL]*5);

          errorAngleI[axis]  = constrain(errorAngleI[axis]+errorAngle,-10000,+10000);    // WindUp     //16 bits is ok here
          ITerm              = ((int32_t)errorAngleI[axis]*conf.I8[PIDLEVEL])>>12;            // 32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result
        }
          else
        { //ACRO MODE or YAW axis
          if (abs(rcCommand[axis])<350)
                error =  rcCommand[axis]*10*8/conf.P8[axis] ; // 16 bits is needed for calculation: 350*10*8 = 28000      16 bits is ok for result if P8>2 (P>0.2)
          else
                error = (int32_t)rcCommand[axis]*10*8/conf.P8[axis] ; // 32 bits is needed for calculation: 500*5*10*8 = 200000   16 bits is ok for result if P8>2 (P>0.2)
          error -= gyroData[axis];

          PTerm = rcCommand[axis];

          errorGyroI[axis]  = constrain(errorGyroI[axis]+error,-16000,+16000);          // WindUp   16 bits is ok here
          if (abs(gyroData[axis])>640)
                errorGyroI[axis] = 0;
          ITerm = (errorGyroI[axis]/125*conf.I8[axis])>>6;                                   // 16 bits is ok here 16000/125 = 128 ; 128*250 = 32000
        }
        if (abs(gyroData[axis])<160)
            PTerm -=  gyroData[axis]*dynP8[axis]/10/8; // 16 bits is needed for calculation   160*200 = 32000         16 bits is ok for result
        else
            PTerm -= (int32_t)gyroData[axis]*dynP8[axis]/10/8; // 32 bits is needed for calculation

        delta          = gyroData[axis] - lastGyro[axis]; // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
        lastGyro[axis] = gyroData[axis];
        deltaSum       = delta1[axis]+delta2[axis]+delta;
        delta2[axis]   = delta1[axis];
        delta1[axis]   = delta;

        if (abs(deltaSum)<640)
            DTerm = (deltaSum*dynD8[axis])>>5;                       // 16 bits is needed for calculation 640*50 = 32000           16 bits is ok for result
        else
            DTerm = ((int32_t)deltaSum*dynD8[axis])>>5;              // 32 bits is needed for calculation

        axisPID[axis] =  PTerm + ITerm - DTerm;
  }

  mixTable();
  writeMotors();
}
