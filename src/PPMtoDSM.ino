/*
  PPM to DSM v1.07 - June 2011
  
  JR version v0.01 - April 2015

  Sends DSM2 or DSMX signal using Spektrum TX module.
  Based on the code by daniel_arg, modifications by AS aka c2po.
  More modifications by JR aka JR63.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/


#include "Arduino.h"
#include <avr/interrupt.h>


//#define DEBUG


/*

Description of the first byte in the DSM Header

DSM2/DSMX	flight		0x18
DSM2/DSMX	range test	0x38
DSM2/DSMX	bind		0x98

DSM2		flight		0x10
DSM2		range test	0x30
DSM2		bind		0x90

France		flight		0x00
France		range test	0x20
France		bind		0x80

So the bit meaning of the first byte is:

bit 7	-	bind			1 -> enabled		0 -> disabled
bit 6	-	unknown, always 0
bit 5	-	range test		1 -> enabled		0 -> disabled
bit 4	-	NOT France		1 -> not France		0 -> France
bit 3	-	DSMX			1 -> enabled		0 -> disabled
bit 2	-	unknown, always 0
bit 1	-	unknown, always 0
bit 0	-	unknown, always 0

resulting in the following defines:

*/


#define DSM_BIND		0x80
#define DSM_RANGE		0x20
#define DSM_NOT_FRANCE		0x10
#define DSM_DSMX		0x08

#define DSM_HEADER_0		DSM_NOT_FRANCE | DSM_DSMX
#define DSM_HEADER_1		0x00

//#define PWM_MID			1500
//#define PWM_SUB			1000
#define PWM_MID			1520					// Futaba midpoint is 1520 us ...
#define PWM_SUB			1008					// ... so with 10bit we have min = 1008, max = 2031 resulting in 0 to 1023 for DSM pulse

#define P_COR			10					// add some us for global correction

#define CHANNEL_MAPPING		1,2,3,4,5,6				// choose your channel mapping
#define THROTTLE_CHANNEL	1					// choose your throttle channel
#define PWM_MIN_THROTTLE	1008					// choose your min throttle

#define CAPTURE_RISING		(1<<CS11) | (1<<ICES1)			// prescaler = 8, capture using rising edge
#define CAPTURE_FALLING		(1<<CS11)				// prescaler = 8, capture using falling edge

#define CAPTURE_EDGE		CAPTURE_FALLING				// choose edge

#define TICKS_PER_uS		1					// number of timer ticks per 1 microsecond with prescaler = 8 and CPU 8MHz
#define MAX_CHANNELS		8					// maximum number of channels we can store, don't increase this above 8
#define MIN_IN_PULSE		( 750 * TICKS_PER_uS)			// valid pulse must be at least   750us
#define MAX_IN_PULSE		(2250 * TICKS_PER_uS)			// valid pulse must be less than 2250us
#define SYNC_GAP_LEN		(5000 * TICKS_PER_uS)			// we assume a space at least 5000us is sync

#define ACQUIRE_FRAMES		10					// must have this many consecutive valid frames to transition to the ready state

#define DSM_CHANNELS		6					// max number of DSM channels transmitted

#define BINDING_PIN		4					// Pin used to bind
#define BINDING_LED		5					// Pin used for binding in process LED
#define PPM_OK_LED		6					// Pin used for PPM ok signal LED
#define RF_OK_PIN		7					// Pin used for RF ok signal
#define GREEN_LED		13					// Pin used for board LED

#define FLASH_LED		250					// LED flash interval in ms


typedef enum {
    NULL_ST = -1, NOT_SYNCED, ACQUIRE, READY, FAILSAFE
} State_t;


static State_t State;							// this will be one of the following states: NULL_ST, NOT_SYNCED, ACQUIRE, READY, FAILSAFE
static byte acquireCount;						// counts the number of times ACQUIRE state has been repeated

static byte ChannelNum;							// number of channels detected so far in the frame (first channel is 1)
static byte ChannelCnt;							// the total number of channels detected in a complete frame

static int Pulses  [MAX_CHANNELS + 1];					// array holding channel pulses width value in microseconds
static int Failsafe[MAX_CHANNELS + 1];					// array holding channel fail safe values

static byte DSM_Header[2] = {DSM_HEADER_0, DSM_HEADER_1};
static byte DSM_Channel[DSM_CHANNELS * 2];

static byte ChanIndex[] = {CHANNEL_MAPPING};				// PPM to DSM channel mapping table

static byte newDSM = 0;


/* ---------- ---------- ---------- Sync ---------- ---------- ---------- */
static void processSync(void)						// sync pulse was detected so reset the channel to first and update the system state
{
  Pulses[0] = ICR1 / TICKS_PER_uS;					// save the sync pulse duration for debugging
  if (State == READY) {                      
    if (ChannelNum != ChannelCnt)					// if the number of channels is unstable, go into failsafe
      State = FAILSAFE;                     
  } else {                                    
    if (State == NOT_SYNCED) {              
      State = ACQUIRE;							// this is the first sync pulse, we need one more to fill the channel data array
      acquireCount = 0;                       
    } else {                                
      if (State == ACQUIRE) {             
        if (++acquireCount > ACQUIRE_FRAMES) {
          State = READY;						// this is the ACQUIRE_FRAMESth sync and all channel data is ok so flag that channel data is valid
          ChannelCnt = ChannelNum;					// save the number of channels detected
        }                                   
      } else {
        if (State == FAILSAFE) {            
          if (ChannelNum == ChannelCnt)					// did we get good pulses on all channels?
            State = READY;                  
        }                                   
      }
    }                                       
  }                                         
  ChannelNum = 0;							// reset the channel counter
}


/* ---------- ---------- ---------- Interrupt ---------- ---------- ---------- */
ISR (TIMER1_OVF_vect)
{
  if (State == READY) {
    State = FAILSAFE;							// use fail safe values if signal lost
    ChannelNum = 0;							// reset the channel count
  }                                         
}                                           


ISR (TIMER1_CAPT_vect)							// we want to measure the time to the end of the pulse
{
  TCNT1 = 0;								// reset the counter
  if (ICR1 >= SYNC_GAP_LEN) {						// is the space between pulses big enough to be the SYNC
    processSync();                          
  } else {                                     
    if (ChannelNum < MAX_CHANNELS) {					// check if its a valid channel pulse and save it
      if ((ICR1 >= MIN_IN_PULSE) && (ICR1 <= MAX_IN_PULSE)) {		// check for valid channel data
        Pulses[++ChannelNum] = ICR1 / TICKS_PER_uS;			// store pulse length as microseconds
	if (ChannelNum == DSM_CHANNELS)					// if we saw DSM_CHANNELS count channels
          newDSM = 1;							// we are ready to send these new ones as DSM frame
      } else {
        if (State == READY) {
          State = FAILSAFE;						// use fail safe values if input data invalid
          ChannelNum = 0;						// reset the channel count
        }
      }
    }
  }
}


/* ---------- ---------- ---------- Class ---------- ---------- ---------- */
class PPM_Decode {

public:
  PPM_Decode(void)							// Constructor
  {
    // empty                                
  }                                         

  void begin(void)
  {
    ChannelNum = 0;
    ChannelCnt = 0;
    State  = NOT_SYNCED;
    pinMode(8, INPUT);							// timer 1 interrupt handler uses pin 8 as input, do not change it
    TCCR1A = 0x00;							// COM1A1 = 0, COM1A0 = 0  =>  disconnect Pin OC1 from timer/counter 1
									// PWM11  = 0, PWM10  = 0  =>  PWM operation disabled
    TCCR1B = CAPTURE_EDGE;						// set capture and prescaler
									// 8 MHz clock with prescaler 8 means TCNT1 increments every 1 uS
    TIMSK1 = _BV(ICIE1) | _BV (TOIE1);					// enable input capture and overflow interrupts for timer 1
    
    for (byte ch = 1; ch <= MAX_CHANNELS; ch++)
      Failsafe[ch] = Pulses[ch] = PWM_MID;				// set midpoint as default values for pulses and failsafe
									// set the throttle channel to min throttle
    Failsafe[THROTTLE_CHANNEL] = Pulses[THROTTLE_CHANNEL] = PWM_MIN_THROTTLE;
  }

  State_t getState(void)
  {
    return State;
  }

  byte getChannelNum(void)
  {
    return ChannelNum;
  }

  byte getChannelCnt(void)
  {
    return ChannelCnt;
  }

  int getChannelData(uint8_t channel)					// this is the access function for channel data
  {
    int result = 0;							// default value
    if (channel <= MAX_CHANNELS)  {
      if ((State == FAILSAFE) && (Failsafe[channel] > 0))
        result = Failsafe[channel];					// return the channels failsafe value if set and state is failsafe
      else
        if ((State == READY) || (State == FAILSAFE)) {
          cli();							// disable interrupts
          result = Pulses[channel] + P_COR;				// return the last valid pulse width for this channel
          sei();							// enable interrupts
        }
    }
    return result;
  }
  
};


PPM_Decode PPM = PPM_Decode();


void setup(void)
{
#ifdef DEBUG
  Serial.begin(115200);							// print values on the screen
#else
  Serial.begin(125000);							// closest speed for DSM module, otherwise it won't work
#endif
  
  PPM.begin();
  
  pinMode(BINDING_PIN, INPUT_PULLUP);
  pinMode(BINDING_LED, OUTPUT);
  pinMode(PPM_OK_LED,  OUTPUT);
  pinMode(RF_OK_PIN,   OUTPUT);
  pinMode(GREEN_LED,   OUTPUT);
  
  digitalWrite(BINDING_LED, HIGH);					// turn on the binding LED
  while (PPM.getState() != READY)					// wait until PPM data is stable and ready
    delay(20);

  DSM_Header[0] |= DSM_BIND;						// set the bind flag
  while (digitalRead(BINDING_PIN) == LOW) {				// while bind button pressed at power up
    if (PPM.getState() == READY) {					// and if PPM data is stable and ready
      if (millis() % FLASH_LED >= FLASH_LED / 2)			// flash binding LED
        digitalWrite(BINDING_LED, HIGH);
      else
        digitalWrite(BINDING_LED, LOW);
      pulseToDSM();							// get current PPM data as failsafe for the receiver to bind
      sendDSM();
      delay(20);
    }
  }
  DSM_Header[0] &= ~DSM_BIND;						// clear the bind flag
  
  digitalWrite(BINDING_LED, LOW);					// turn off the binding LED
}


void loop(void)
{
  if (millis() % (FLASH_LED * 2) < FLASH_LED / 10)			// flash the board LED - we are alive
    digitalWrite(GREEN_LED, HIGH);
  else
    digitalWrite(GREEN_LED, LOW);
    
  if (PPM.getState() == READY) {					// if ready
    if (millis() % (FLASH_LED * 2) < FLASH_LED / 10)			// flash the PPM ok LED
      digitalWrite(PPM_OK_LED, HIGH);
    else
      digitalWrite(PPM_OK_LED, LOW);
    digitalWrite(BINDING_LED, LOW);					// turn off binding LED
    digitalWrite(RF_OK_PIN, LOW);					// turn on RF ok LED in the radio
    
    if (newDSM) {							// if new DSM data available
      pulseToDSM();							// get current data
      sendDSM();							// send DSM frame
      newDSM = 0;							// frame send
    }
  } else {								// else failsafe
    digitalWrite(PPM_OK_LED, LOW);					// turn off the PPM ok LED
    digitalWrite(BINDING_LED, HIGH);					// turn on binding LED
    digitalWrite(RF_OK_PIN, HIGH);					// turn off RF ok LED in the radio, alarm will sound
    
    pulseToDSM();							// get current PPM data
    sendDSM();								// send DSM frame
    delay(20);								// but send not faster than 20ms
  }
}


void pulseToDSM(void)
{
  int pulse;
  for (byte i=0; i<DSM_CHANNELS; i++) {
    pulse = PPM.getChannelData(ChanIndex[i]) - PWM_SUB;
    pulse = constrain(pulse, 0, 0x3FF);
    DSM_Channel[i*2]   = (byte)(i << 2) | highByte(pulse);
    DSM_Channel[i*2+1] = lowByte(pulse);
  }
}


#ifndef DEBUG

void sendDSM(void)
{
    Serial.write(DSM_Header, 2);
    Serial.write(DSM_Channel, DSM_CHANNELS * 2);
}

#else

void sendDSM(void)
{
    int value;
    Serial.print(millis(), DEC);
    Serial.print("   ");
    serialPrintHex(DSM_Header[0]);
    Serial.print("   ");
    serialPrintHex(DSM_Header[1]);
    Serial.print("   ");
    for (byte i=0; i < DSM_CHANNELS; i++) {				// print channels 1 to 6 in hex and dec
      serialPrintHex(DSM_Channel[i*2]);
      Serial.print(" ");
      serialPrintHex(DSM_Channel[i*2+1]);
      value = (DSM_Channel[i*2]&0x03)<<8 | DSM_Channel[i*2+1];
      Serial.print(" (");
      if (value < 1000)
        Serial.print(" ");
      if (value < 100)
        Serial.print(" ");
      if (value < 10)
        Serial.print(" ");
      Serial.print(value, DEC);
      Serial.print(")  ");
    }
    if (PPM.getChannelData(7) < 1000)
      Serial.print(" ");
    Serial.print(PPM.getChannelData(7), DEC);				// channel 7
    Serial.print("  ");
    if (PPM.getChannelData(8) < 1000)
      Serial.print(" ");
    Serial.print(PPM.getChannelData(8), DEC);				// channel 8
    Serial.print("  ");
    if (PPM.getChannelData(0) < 10000)
      Serial.print(" ");
    Serial.print(PPM.getChannelData(0), DEC);				// sync pulse length
    Serial.print("  ");
    Serial.println(" ");
}


void serialPrintHex(byte b)
{
    byte b1 = (b >> 4) & 0x0F;
    byte b2 = (b & 0x0F);
    char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
    char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;
    Serial.print(c1);
    Serial.print(c2);
}

#endif
