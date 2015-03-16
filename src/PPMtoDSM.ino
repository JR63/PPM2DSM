/*
  PPM to DSM v1.07 - June 2011
  
  JR version v0.01 - March 2015

  Sends DSM2 or DSMX signal using Spektrum TX module.
  Based on the code by daniel_arg, modifications by AS aka c2po, modifications by JR aka JR63.

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


typedef enum {
    NULL_ST = -1, NOT_SYNCHED, ACQUIRING, READY, FAILSAFE
} State_t;


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

#define DSM_BIND	0x80
#define DSM_RANGE	0x20
#define DSM_NOT_FRANCE	0x10
#define DSM_DSMX	0x08

#define DSM_HEADER_0	0x00      //	DSM_NOT_FRANCE | DSM_DSMX    // TODO check if we can do DSMX
#define DSM_HEADER_1	0x00

//#define PWM_MID	1500
//#define PWM_SUB	1000
#define PWM_MID		1520			// Futaba midpoint is 1520 us ...
#define PWM_SUB		1008			// ... so with 10bit we have min = 1008, max = 2031 resulting in 0 to 1023 for DSM pulse

#define CHANNEL_MAPPING	3,1,2,4,5,6		// choose your channel mapping
#define TROTTLE_CHANNEL	3			// choose your throttle channel
#define PWM_MIN_TROTTLE	1008			// choose your min throttle

#define CAPTURE_RISING	(1<<CS11) | (1<<ICES1)	// prescaler = 8, capture using rising edge
#define CAPTURE_FALLING	(1<<CS11)		// prescaler = 8, capture using falling edge

#define CAPTURE_EDGE	CAPTURE_FALLING		// choose edge

#define TICKS_PER_uS      1                 // number of timer ticks per 1 microsecond with prescaler = 8 and CPU 8MHz
#define MAX_CHANNELS      8                 // maximum number of channels we can store, don't increase this above 8
#define MIN_IN_PULSE  ( 750 * TICKS_PER_uS) // valid pulse must be at least   750us
#define MAX_IN_PULSE  (2250 * TICKS_PER_uS) // valid pulse must be less than 2250us
#define SYNC_GAP_LEN  (5000 * TICKS_PER_uS) // we assume a space at least 5000us is sync
#define VALID_FRAMES     10                 // must have this many consecutive valid frames to transition to the ready state.

#define P_COR             1                 // sub some us for global correction

#define DSM_CHANNELS      6                 // Max number of DSM Channels transmitted

#define BINDING_PIN       4                 // Pin used to bind
#define BINDING_LED       5                 // Pin used for Binding in process LED
#define PPM_OK_LED        6                 // Pin used for PPM Ok signal LED
#define RF_OK_PIN         7                 // Pin used for RF Ok signal
#define GREEN_LED        13                 // Pin used for board LED

#define FLASH_LED       250                 // LED flash interval in ms

static int Pulses  [MAX_CHANNELS + 1];      // array holding channel pulses width value in microseconds
static int Failsafe[MAX_CHANNELS + 1];      // array holding channel fail safe values
static byte ChannelNum;                     // number of channels detected so far in the frame (first channel is 1)
static byte ChannelCnt;                     // the total number of channels detected in a complete frame
static State_t State;                       // this will be one of the following states: Null, Not_Synched, Acquiring, Ready, Failsafe
static byte stateCount;                     // counts the number of times this state has been repeated

static byte DSM_Header[2] = {DSM_HEADER_0, DSM_HEADER_1};
static byte DSM_Channel[DSM_CHANNELS * 2] = {0x00,0xAA,0x05,0xFF,0x09,0xFF,0x0D,0xFF,0x13,0x54,0x14,0xAA};
static byte DSM_Sent = 0;
static byte ChanIndex[] = {CHANNEL_MAPPING};// PPM to DSM Channel Mapping Table
static byte count;
static int pulse;

/* ---------- ---------- ---------- Sync ---------- ---------- ---------- */
static void processSync() {                 // sync pulse was detected so reset the channel to first and update the system state
  Pulses[0] = ICR1 / TICKS_PER_uS;          // save the sync pulse duration for debugging
  if (State == READY) {                      
    if (ChannelNum != ChannelCnt)           // if the number of channels is unstable, go into failsafe
      State = FAILSAFE;                     
  }                                         
  else {                                    
    if (State == NOT_SYNCHED) {              
      State = ACQUIRING;                    // this is the first sync pulse, we need one more to fill the channel data array
      stateCount = 0;                       
    } else {                                
      if (State == ACQUIRING) {             
        if (++stateCount > VALID_FRAMES) {
          State = READY;                    // this is the second sync and all channel data is ok so flag that channel data is valid
          ChannelCnt = ChannelNum;          // save the number of channels detected
        }                                   
      } else                                
        if (State == FAILSAFE) {            
          if (ChannelNum == ChannelCnt)     // did we get good pulses on all channels?
            State = READY;                  
        }                                   
    }                                       
  }                                         
  ChannelNum = 0;                           // reset the channel counter
}

/* ---------- ---------- ---------- Interrupt ---------- ---------- ---------- */
ISR (TIMER1_OVF_vect) {
  if (State == READY) {
    State = FAILSAFE;                       // use fail safe values if signal lost
    ChannelNum = 0;                         // reset the channel count
  }                                         
}                                           

ISR (TIMER1_CAPT_vect) {                    // we want to measure the time to the end of the pulse
  TCNT1 = 0;                                // reset the counter
  if (ICR1 >= SYNC_GAP_LEN)                 // is the space between pulses big enough to be the SYNC
    processSync();                          
  else                                      
    if (ChannelNum < MAX_CHANNELS) {        // check if its a valid channel pulse and save it
      if ((ICR1 >= MIN_IN_PULSE) && (ICR1 <= MAX_IN_PULSE)) // check for valid channel data
        Pulses[++ChannelNum] = ICR1 / TICKS_PER_uS;         // store pulse length as microseconds
      else
        if (State == READY) {
          State = FAILSAFE;                 // use fail safe values if input data invalid
          ChannelNum = 0;                   // reset the channel count
        }
    }
}

/* ---------- ---------- ---------- Class ---------- ---------- ---------- */
class PPM_Decode {

public:
  PPM_Decode() {                            // Constructor
    // empty                                
  }                                         

  void begin() {
    pinMode(8, INPUT);                      // Timer1 interrupt handler uses pin 8 as input, do not change it
    ChannelNum = 0;
    State  = NOT_SYNCHED;
    TCCR1A = 0x00;                          // COM1A1=0, COM1A0=0 => Disconnect Pin OC1 from Timer/Counter 1
                                            // PWM11=0,  PWM10=0  => PWM Operation disabled
    TCCR1B = CAPTURE_EDGE;                  // set capture and prescaler
                                            // 8 MHz clock with prescaler 8 means TCNT1 increments every 1 uS
    TIMSK1 = _BV(ICIE1) | _BV (TOIE1);      // enable input capture and overflow interrupts for timer 1
    for (byte ch = 1; ch <= MAX_CHANNELS; ch++)
      Failsafe[ch] = Pulses[ch] = PWM_MID;  // set midpoint as default values for pulses and failsafe
					    // set the throttle channel to min throttle
    Failsafe[TROTTLE_CHANNEL] = Pulses[TROTTLE_CHANNEL] = PWM_MIN_TROTTLE;
  }

  State_t getState() {
    return State;
  }

  byte getChannelCnt() {
    return ChannelCnt;
  }

  void setFailsafe(byte ch, int value) {    // pulse width to use if invalid data, value of 0 uses last valid data
    if ((ch > 0) && (ch <= MAX_CHANNELS))
      Failsafe[ch] = value;
  }

  void setFailsafe() {                      // setFailsafe with no arguments sets failsafe for all channels to their current values
    if (State == READY)                     // useful to capture current tx settings as failsafe values
      for (byte ch = 1; ch <= MAX_CHANNELS; ch++)
        Failsafe[ch] = Pulses[ch];
  }

  int getChannelData(uint8_t channel) {     // this is the access function for channel data
    int result = 0;                         // default value
    if (channel <= MAX_CHANNELS)  {
      if ((State == FAILSAFE) && (Failsafe[channel] > 0 ))
        result = Failsafe[channel];         // return the channels failsafe value if set and State is Failsafe
      else
        if ((State == READY) || (State == FAILSAFE)) {
          cli();                            // disable interrupts
          result = Pulses[channel] - P_COR; // return the last valid pulse width for this channel
          sei();                            // enable interrupts
        }
    }
    return result;
  }
};

PPM_Decode Receiver = PPM_Decode();

void setup() {
  delay(100);
  #ifdef DEBUG
  Serial.begin(115200);                     // print values on the screen
  #else
  Serial.begin(125000);                     // closest speed for DSM module, otherwise it won't work
  #endif
  Receiver.begin();
  pinMode(BINDING_PIN, INPUT_PULLUP);
  pinMode(BINDING_LED, OUTPUT);
  pinMode(PPM_OK_LED,  OUTPUT);
  pinMode(RF_OK_PIN,   OUTPUT);
  pinMode(GREEN_LED,   OUTPUT);
  delay(100);

  DSM_Header[0] = DSM_Header[0] | DSM_BIND;	// set the bind flag
  digitalWrite(BINDING_LED, HIGH);          	// turn on the binding LED
  while (digitalRead(BINDING_PIN) == LOW) {  	// if bind button pressed at the power up
    if (millis() % FLASH_LED > FLASH_LED / 2)
      digitalWrite(BINDING_LED, HIGH);      	// flash binding LED
    else
      digitalWrite(BINDING_LED, LOW);
    sendDSM();
    delay(20);
  }
  digitalWrite(BINDING_LED,LOW);            	// turn off the binding LED
  DSM_Header[0] = DSM_Header[0] & ~DSM_BIND;	// clear the bind flag
  count = 50;
  while (Receiver.getState() != READY && count-- > 0) // wait 5 sec or until PPM data is stable and ready
    delay(100);
}

void loop() {
  if (millis() % (FLASH_LED * 4) < FLASH_LED / 16)
    digitalWrite(GREEN_LED, HIGH);          // flash the board LED - we are alive
  else
    digitalWrite(GREEN_LED, LOW);
  if (Receiver.getState() == READY) {       // if PPM is ready
    if (millis() % (FLASH_LED * 4) < FLASH_LED / 16)
      digitalWrite(PPM_OK_LED, HIGH);       // flash the PPM OK LED
    else
      digitalWrite(PPM_OK_LED, LOW);
    digitalWrite(BINDING_LED, LOW);         // turn off binding LED
    digitalWrite(RF_OK_PIN, LOW);           // turn on RF OK LED in the radio
  } else {
    digitalWrite(BINDING_LED, HIGH);        // turn on binding LED
    digitalWrite(PPM_OK_LED, LOW);          // turn off the PPM OK LED
    digitalWrite(RF_OK_PIN, HIGH);          // turn off RF OK LED in the radio, alarm will sound
  }
  if (Receiver.getState() == READY || Receiver.getState() == FAILSAFE) {
    if (ChannelNum == 0 || ChannelNum == ChannelCnt) {  // during sync pulse or in failsafe
      if (DSM_Sent == 0) {                  // if DSM frame is not sent yet
        for (byte i=0; i<DSM_CHANNELS; i++) { // get receiver data
          pulse = Receiver.getChannelData(ChanIndex[i]) - PWM_SUB;
          pulse = constrain(pulse, 0, 0x3FF);
          DSM_Channel[i*2]   = (byte)(i << 2) | highByte(pulse);
          DSM_Channel[i*2+1] = lowByte(pulse);
        }
        sendDSM();                          // send frame
        DSM_Sent = 1;                       // frame sent flag
      } else {
        if (Receiver.getState() == FAILSAFE) {
          delay(20);                        // in case of failsafe
          DSM_Sent = 0;                     // reset flag after delay
        }
      }
    } else {
      if (ChannelNum == 1)                  // after first channel is received
        DSM_Sent = 0;                       // reset flag for the next frame
    }
  }
}

#ifndef DEBUG

void sendDSM() {
    Serial.write(DSM_Header, 2);
    Serial.write(DSM_Channel, DSM_CHANNELS * 2);
}

#else

void sendDSM() {
    int value;
    Serial.print(DSM_Header[0], HEX);
    Serial.print("   ");
    Serial.print(DSM_Header[1], HEX);
    Serial.print("   ");
    for (byte i=0; i < DSM_CHANNELS; i++) { // print channels 1 to 6 in Hex and Dec
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
    if (Receiver.getChannelData(7) < 1000)
      Serial.print(" ");
    Serial.print(Receiver.getChannelData(7), DEC);  // channel 7
    Serial.print("  ");
    if (Receiver.getChannelData(8) < 1000)
      Serial.print(" ");
    Serial.print(Receiver.getChannelData(8), DEC);  // channel 8
    Serial.print("  ");
    if (Receiver.getChannelData(0) < 10000)
      Serial.print(" ");
    Serial.print(Receiver.getChannelData(0), DEC);  // sync pulse length
    Serial.print("  ");
    Serial.println(" ");
    delay(200);
}

void serialPrintHex(byte b) {
    byte b1 = (b >> 4) & 0x0F;
    byte b2 = (b & 0x0F);
    char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
    char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;
    Serial.print(c1);
    Serial.print(c2);
}

#endif
