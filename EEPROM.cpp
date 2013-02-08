#include <avr/eeprom.h>

#define EEPROM_CONF_VERSION 161

void readEEPROM() {
  uint8_t i;

  eeprom_read_block((void*)&conf, (void*)0, sizeof(conf));
  for(i=0;i<6;i++) {
    lookupPitchRollRC[i] = (2500+conf.rcExpo8*(i*i-25))*i*(int32_t)conf.rcRate8/2500;
  }
  for(i=0;i<11;i++) {
    int16_t tmp = 10*i-conf.thrMid8;
    uint8_t y = 1;
    if (tmp>0) y = 100-conf.thrMid8;
    if (tmp<0) y = conf.thrMid8;
    lookupThrottleRC[i] = 10*conf.thrMid8 + tmp*( 100-conf.thrExpo8+(int32_t)conf.thrExpo8*(tmp*tmp)/(y*y) )/10; // [0;1000]
    lookupThrottleRC[i] = MINTHROTTLE + (int32_t)(MAXTHROTTLE-MINTHROTTLE)* lookupThrottleRC[i]/1000;            // [0;1000] -> [MINTHROTTLE;MAXTHROTTLE]
  }
}

void writeParams(uint8_t b) {
  conf.checkNewConf = EEPROM_CONF_VERSION; // make sure we write the current version into eeprom
  eeprom_write_block((const void*)&conf, (void*)0, sizeof(conf));
  readEEPROM();
  if (b == 1) blinkLED(15,20,1);
}

void checkFirstTime() {
  if (EEPROM_CONF_VERSION == conf.checkNewConf) return;
  conf.P8[ROLL]  = 40;  conf.I8[ROLL] = 30; conf.D8[ROLL]  = 23;
  conf.P8[PITCH] = 40; conf.I8[PITCH] = 30; conf.D8[PITCH] = 23;
  conf.P8[YAW]   = 85;  conf.I8[YAW]  = 45;  conf.D8[YAW]  = 0;
  conf.P8[PIDALT]   = 16; conf.I8[PIDALT]   = 15; conf.D8[PIDALT]   = 7;

  conf.P8[PIDPOS]  = POSHOLD_P * 100;     conf.I8[PIDPOS]    = POSHOLD_I * 100;       conf.D8[PIDPOS]    = 0;
  conf.P8[PIDPOSR] = POSHOLD_RATE_P * 10; conf.I8[PIDPOSR]   = POSHOLD_RATE_I * 100;  conf.D8[PIDPOSR]   = POSHOLD_RATE_D * 1000;
  conf.P8[PIDNAVR] = NAV_P * 10;          conf.I8[PIDNAVR]   = NAV_I * 100;           conf.D8[PIDNAVR]   = NAV_D * 1000;

  conf.P8[PIDLEVEL] = 70; conf.I8[PIDLEVEL] = 10; conf.D8[PIDLEVEL] = 100;
  conf.P8[PIDMAG] = 40;

  conf.P8[PIDVEL] = 0;  conf.I8[PIDVEL] = 0;  conf.D8[PIDVEL] = 0;

  conf.rcRate8 = 90; conf.rcExpo8 = 65;
  conf.rollPitchRate = 0;
  conf.yawRate = 0;
  conf.dynThrPID = 0;
  conf.thrMid8 = 50; conf.thrExpo8 = 0;
  for(uint8_t i=0;i<CHECKBOXITEMS;i++) {conf.activate[i] = 0;}
  conf.angleTrim[0] = 0; conf.angleTrim[1] = 0;
  conf.powerTrigger1 = 0;

  writeParams(0); // this will also (p)reset checkNewConf with the current version number again.
}
