/**************************************************************************************/
/***************   Global RX related variables  接收机相关参数？   ********************/
/**************************************************************************************/

//RAW RC values will be store here
static uint8_t rcChannel[8]  = {ROLLPIN, PITCHPIN, YAWPIN, THROTTLEPIN, AUX1PIN,AUX2PIN,AUX3PIN,AUX4PIN};
static uint8_t PCInt_RX_Pins[PCINT_PIN_COUNT] = {PCINT_RX_BITS}; // if this slowes the PCINT readings we can switch to a define for each pcint bit



/**************************************************************************************/
/***************                   RX Pin Setup                    ********************/
/**************************************************************************************/
void configureReceiver() {
  /******************    Configure each rc pin for PCINT    ***************************/
  #if defined(STANDARD_RX)
    #if defined(MEGA)
      DDRK = 0;  // defined PORTK as a digital port ([A8-A15] are consired as digital PINs and not analogical)
    #endif
    // PCINT activation
    for(uint8_t i = 0; i < PCINT_PIN_COUNT; i++)
    { // i think a for loop is ok for the init.
      PCINT_RX_PORT |= PCInt_RX_Pins[i];
      PCINT_RX_MASK |= PCInt_RX_Pins[i];
    }
    PCICR = PCIR_PORT_BIT;
}

/**************************************************************************************/
/***************               Standard RX Pins reading            ********************/
/**************************************************************************************/
#if defined(STANDARD_RX)
  // predefined PC pin block (thanks to lianj)
  #define RX_PIN_CHECK(pin_pos, rc_value_pos)                                                        \
    if (mask & PCInt_RX_Pins[pin_pos]) {                                                             \
      if (!(pin & PCInt_RX_Pins[pin_pos])) {                                                         \
        dTime = cTime-edgeTime[pin_pos]; if (900<dTime && dTime<2200) rcValue[rc_value_pos] = dTime; \
      } else edgeTime[pin_pos] = cTime;                                                              \
    }
  // port change Interrupt
  ISR(RX_PC_INTERRUPT)
   {
    //this ISR is common to every receiver channel, it is call everytime a change state occurs on a RX input pin
    uint8_t mask;
    uint8_t pin;
    uint16_t cTime,dTime;
    static uint16_t edgeTime[8];
    static uint8_t PCintLast;

    pin = RX_PCINT_PIN_PORT; // RX_PCINT_PIN_PORT indicates the state of each PIN for the arduino port dealing with Ports digital pins

    mask = pin ^ PCintLast;   // doing a ^ between the current interruption and the last one indicates wich pin changed
    sei();                    // re enable other interrupts at this point, the rest of this interrupt is not so time critical and can be interrupted safely
    PCintLast = pin;          // we memorize the current state of all PINs [D0-D7]

    cTime = micros();         // micros() return a uint32_t, but it is not usefull to keep the whole bits => we keep only 16 bits

    #if (PCINT_PIN_COUNT > 0)
      RX_PIN_CHECK(0,2);
    #endif
    #if (PCINT_PIN_COUNT > 1)
      RX_PIN_CHECK(1,4);
    #endif
    #if (PCINT_PIN_COUNT > 2)
      RX_PIN_CHECK(2,5);
    #endif
    #if (PCINT_PIN_COUNT > 3)
      RX_PIN_CHECK(3,6);
    #endif
    #if (PCINT_PIN_COUNT > 4)
      RX_PIN_CHECK(4,7);
    #endif
    #if (PCINT_PIN_COUNT > 5)
      RX_PIN_CHECK(5,0);
    #endif
    #if (PCINT_PIN_COUNT > 6)
      RX_PIN_CHECK(6,1);
    #endif
    #if (PCINT_PIN_COUNT > 7)
      RX_PIN_CHECK(7,3);
    #endif

    #if defined(FAILSAFE) && !defined(PROMICRO)
      if (mask & 1<<THROTTLEPIN) {  // If pulse present on THROTTLE pin (independent from ardu version), clear FailSafe counter  - added by MIS
        if(failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0; }
    #endif
  }
#endif


/**************************************************************************************/
/***************          combine and sort the RX Datas            ********************/
/**************************************************************************************/
uint16_t readRawRC(uint8_t chan)
 {
  uint16_t data;
  uint8_t oldSREG;
  oldSREG = SREG; cli(); // Let's disable interrupts
  data = rcValue[rcChannel[chan]]; // Let's copy the data Atomically

  SREG = oldSREG; sei();// Let's enable the interrupts

  return data; // We return the value correctly copied when the IRQ's where disabled
}

/**************************************************************************************/
/***************          compute and Filter the RX data           ********************/
/**************************************************************************************/
void computeRC() {
  static int16_t rcData4Values[8][4], rcDataMean[8];
  static uint8_t rc4ValuesIndex = 0;
  uint8_t chan,a;
  #if !(defined(RCSERIAL) || defined(OPENLRSv2MULTI)) // dont know if this is right here
    rc4ValuesIndex++;
    for (chan = 0; chan < 8; chan++) {
      rcData4Values[chan][rc4ValuesIndex%4] = readRawRC(chan);
      rcDataMean[chan] = 0;
      for (a=0;a<4;a++) rcDataMean[chan] += rcData4Values[chan][a];
      rcDataMean[chan]= (rcDataMean[chan]+2)/4;
      if ( rcDataMean[chan] < rcData[chan] -3)  rcData[chan] = rcDataMean[chan]+2;
      if ( rcDataMean[chan] > rcData[chan] +3)  rcData[chan] = rcDataMean[chan]-2;
    }
  #endif
}
