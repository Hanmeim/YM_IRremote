#include "YoungMakerIR.h"
// Provides ISR
#include <avr/interrupt.h>
uint8_t _ir_flag_a;
uint8_t _ir_flag_b;
uint8_t _ir_pin;
uint8_t _irRead;
volatile irparams_t irparams;
bool MATCH(uint8_t measured_ticks, uint8_t desired_us)
{
  // Serial.print(measured_ticks);Serial.print(",");Serial.println(desired_us);
  return (measured_ticks >= desired_us - (desired_us >> 2) - 1 && measured_ticks <= desired_us + (desired_us >> 2) + 1); //判断前后25%的误差
}





/**************************************
* @函数名称：  ISR(TIMER_INTR_NAME)
* @功    能：  定时器中断，红外接收器使用 
* @参    数：  TIMER_INTR_NAME    中断程序地址
*              
* @返 回 值：  无
* @作    者：  （xiaoxu更改）  2018.12.14    
* @备    注：  
***************************************/

ISR(TIMER_INTR_NAME)
{
  uint8_t irdata = (uint8_t)digitalRead(_ir_pin);

  irparams.timer++; // One more 50us tick
  if (irparams.rawlen >= RAWBUF) {

    irparams.rcvstate = STATE_STOP;
  }
  switch (irparams.rcvstate) {
    case STATE_IDLE: // In the middle of a gap
      if (irdata == MARK) {
        irparams.rawlen = 0;
        irparams.timer = 0;
        irparams.rcvstate = STATE_MARK;
      }
      break;
    case STATE_MARK: // timing MARK
      if (irdata == SPACE) {   // MARK ended, record time
        irparams.rawbuf[irparams.rawlen++] = irparams.timer;
        irparams.timer = 0;
        irparams.rcvstate = STATE_SPACE;
      }
      break;
    case STATE_SPACE: // timing SPACE
      if (irdata == MARK) { // SPACE just ended, record it
        irparams.rawbuf[irparams.rawlen++] = irparams.timer;
        irparams.timer = 0;
        irparams.rcvstate = STATE_MARK;
      }
      else { // SPACE
        if (irparams.timer > GAP_TICKS) {
          // big SPACE, indicates gap between codes
          // Mark current code as ready for processing
          // Switch to STOP
          // Don't reset timer; keep counting space width
          irparams.rcvstate = STATE_STOP;
        }
      }
      break;
    case STATE_STOP: // waiting, measuring gap
      if (irdata == MARK) { // reset gap timer
        irparams.timer = 0;
      }
      break;
  }
  // irparams.lastTime = new_time;
}

YoungMakerIR::YoungMakerIR()
{

}

YoungMakerIR::YoungMakerIR(uint8_t pin)
{
  _ir_pin = pin;
  sendPin = pin;
}

void YoungMakerIR::begin(uint8_t pin)
{
  _ir_pin = pin;
  pinMode(_ir_pin, INPUT);
  // _buzz_ir = 1;
  cli();
  // setup pulse clock timer interrupt
  //Prescale /8 (16M/8 = 0.5 microseconds per tick)
  // Therefore, the timer interval can range from 0.5 to 128 microseconds
  // depending on the reset value (255 to 0)
  TIMER_CONFIG_NORMAL();

  //Timer2 Overflow Interrupt Enable
  TIMER_ENABLE_INTR;

  // TIMER_RESET;

  sei();  // enable interrupts

  // initialize state machine variables
  irparams.rcvstate = STATE_IDLE;
  irparams.rawlen = 0;

  lastIRTime = 0.0;
  irDelay = 0;
  irIndex = 0;
  irRead = 0;
  irReady = false;
  irBuffer = "";
  irPressed = false;

}

void YoungMakerIR::end() {
  EIMSK &= ~(1 << INT0);
}




// Decodes the received IR message
// Returns 0 if no data ready, 1 if data ready.
// Results of decoding are stored in results
boolean  YoungMakerIR::decode() {
  rawbuf = irparams.rawbuf;
  rawlen = irparams.rawlen;

  if (irparams.rcvstate != STATE_STOP) {
    return 0;
  }

  if (decodeNEC()) {
    begin(_ir_pin);
    return 1;
  }
  begin(_ir_pin);
  return 0;
}

// NECs have a repeat only 4 items long
boolean  YoungMakerIR::decodeNEC() {
  uint32_t data = 0;
  int offset = 0; // Skip first space
  // Initial mark
  if (!MATCH(rawbuf[offset], NEC_HDR_MARK / 50)) {
    return 0;
  }
  offset++;
  // Check for repeat
  if (rawlen == 3 &&
      MATCH(rawbuf[offset], NEC_RPT_SPACE / 50) &&
      MATCH(rawbuf[offset + 1], NEC_BIT_MARK / 50)) {
    bits = 0;
    // results->value = REPEAT;
    // Serial.println("REPEAT");
    decode_type = NEC;
    return 1;
  }
  if (rawlen < 2 * NEC_BITS + 3) {
    return 0;
  }
  // Initial space
  if (!MATCH(rawbuf[offset], NEC_HDR_SPACE / 50)) {
    return 0;
  }
  offset++;
  for (int i = 0; i < NEC_BITS; i++) {
    if (!MATCH(rawbuf[offset], NEC_BIT_MARK / 50)) {
      return 0;
    }
    offset++;
    if (MATCH(rawbuf[offset], NEC_ONE_SPACE / 50)) {
      //data = (data << 1) | 1;
      data = (data >> 1) | 0x80000000;
    }
    else if (MATCH(rawbuf[offset], NEC_ZERO_SPACE / 50)) {
      //data <<= 1;
      data >>= 1;
    }
    else {
      return 0;

    }
    offset++;
  }
  // Success
  bits = NEC_BITS;
  value = data;
  decode_type = NEC;
  return 1;
}


unsigned char YoungMakerIR::getCode() {
  if (!_ir_flag_b)
  {
    begin(_ir_pin);
    _ir_flag_b = 1;
  }

  if (decode())
  {
    irRead = ((value >> 8) >> 8) & 0xff;
    _ir_flag_a = 0;
  }
  if (_ir_flag_a)
  {
    irRead = 0;
  }
  _ir_flag_a++;
  return irRead;

}

boolean YoungMakerIR::keyPressed(unsigned char r) {
  //  if (!_buzz_ir)
  //    begin(_ir_pin);
  if (decode())
  {
    irRead = ((value >> 8) >> 8) & 0xff;
    lastIRTime = millis() / 1000.0;
    irPressed = true;
    if (irRead == 0xa || irRead == 0xd) {
      irIndex = 0;
      irReady = true;
    } else {
      irBuffer += irRead;
      irIndex++;
      if (irIndex > 64) {
        irIndex = 0;
        irBuffer = "";
      }
    }
    irDelay = 0;
  } else {
    irDelay++;
    if (irRead > 0) {
      if (irDelay > 5000) {
        irRead = 0;
        irDelay = 0;
      }
    }
  }
  ///////////
  irIndex = 0;
  if (millis() / 1000.0 - lastIRTime > 0.2) {
    return false;
  }
  return irRead == r;
}
boolean YoungMakerIR::keyPressed() {
  //  if (!_buzz_ir)
  //    begin(_ir_pin);

  if (decode())
    return 1;
  else
    return 0;
}



/**************************************
* @函数名称：  sendNEC
* @功    能：  读取接收到的NEC红外码，并进行红外解码。 
* @参    数：  Raw_data  要发送的数据
*              
* @返 回 值：  无
* @作    者：  （xiaoxu更改）  2018.12.14    
* @备    注：   用户使用发送接口，发送原型为(0xFF << 16) | (Raw_data << 8) | (~Raw_data & 0x0000FF)的32位数据，且Raw_data数据高低位已进行互换。
***************************************/
#if SEND_NEC
void  YoungMakerIR::sendNEC (uint8_t Raw_data)
{
  pinMode(sendPin, OUTPUT);
  int nbits = 32;
  Raw_data = (Raw_data << 4) | (Raw_data >> 4);
  Raw_data = ((Raw_data << 2) & 0xcc) | ((Raw_data >> 2) & 0x33);
  Raw_data = ((Raw_data << 1) & 0xaa) | ((Raw_data >> 1) & 0x55);
  Serial.println(Raw_data, HEX);
  unsigned long data = (0xFF << 16) | (Raw_data << 8) | (~Raw_data & 0x0000FF);
  enableIROut(38);

  // Header
  mark(NEC_HDR_MARK);
  space(NEC_HDR_SPACE);

  // Data
  for (unsigned long  mask = 1UL << (nbits - 1);  mask;  mask >>= 1) {
    if (data & mask) {
      mark(NEC_BIT_MARK);
      space(NEC_ONE_SPACE);         //发送1
    } else {
      mark(NEC_BIT_MARK);
      space(NEC_ZERO_SPACE);        //发送0
    }
  }

  // Footer
  mark(NEC_BIT_MARK);
  space(0);  // Always end with the LED off
}
#endif



/**************************************
* @函数名称：  enableIROut
* @功    能：  以NEC协议发送红外码 
* @参    数：  uint8_t khz  频率
*              
* @返 回 值：  无
* @作    者：  （xiaoxu更改）  2018.12.14    
* @备    注：   
***************************************/
void  YoungMakerIR::enableIROut (uint8_t khz)
{

  periodTime = (1000U + khz / 2) / khz; // = 1000/khz + 1/2 = round(1000.0/khz)
  periodOnTime = periodTime * DUTY_CYCLE / 100U - PULSE_CORRECTION;

  // Disable the Timer2 Interrupt (which is used for receiving IR)
  TIMER_DISABLE_INTR; //Timer2 Overflow Interrupt

  pinMode(sendPin, OUTPUT);
  SENDPIN_OFF(sendPin); // Turn off the LED to save power

  TIMER_CONFIG_KHZ(khz); 
}




void YoungMakerIR::mark(unsigned int time)
{
#ifdef USE_SOFT_CARRIER
  unsigned long start = micros();
  unsigned long stop = start + time;
  if (stop + periodTime < start)
    // Counter wrap-around, happens very seldomly, but CAN happen.
    // Just give up instead of possibly damaging the hardware.
    return;

  unsigned long nextPeriodEnding = start;
  unsigned long now = micros();
  while (now < stop) {
    SENDPIN_ON(sendPin);
    sleepMicros(periodOnTime);
    SENDPIN_OFF(sendPin);
    nextPeriodEnding += periodTime;
    sleepUntilMicros(nextPeriodEnding);
    now = micros();
  }
#else
  TIMER_ENABLE_PWM; // Enable pin 3 PWM output
  if (time > 0) custom_delay_usec(time);
#endif
}

//+=============================================================================
// Leave pin off for time (given in microseconds)
// Sends an IR space for the specified number of microseconds.
// A space is no output, so the PWM output is disabled.
//
void  YoungMakerIR::space (unsigned int time)
{
  TIMER_DISABLE_PWM; // Disable pin 3 PWM output
  if (time > 0) custom_delay_usec(time);
}


void YoungMakerIR::custom_delay_usec(unsigned long uSecs) {
  if (uSecs > 4) {
    unsigned long start = micros();
    unsigned long endMicros = start + uSecs - 4;
    if (endMicros < start) { // Check if overflow
      while ( micros() > start ) {} // wait until overflow
    }
    while ( micros() < endMicros ) {} // normal wait
  }

}



void inline YoungMakerIR::sleepMicros(unsigned long us)
{
#ifdef USE_SPIN_WAIT
  sleepUntilMicros(micros() + us);
#else
  if (us > 0U) // Is this necessary? (Official docu https://www.arduino.cc/en/Reference/DelayMicroseconds does not tell.)
    delayMicroseconds((unsigned int) us);
#endif
}

void inline YoungMakerIR::sleepUntilMicros(unsigned long targetTime)
{
#ifdef USE_SPIN_WAIT
  while (micros() < targetTime)
    ;
#else
  unsigned long now = micros();
  if (now < targetTime)
    sleepMicros(targetTime - now);
#endif
}
