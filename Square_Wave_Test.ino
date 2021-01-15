/*
                        Square wave output using a bit counter
                        Vernon Billingsley 2021

                        MIDI In
                        Ouput on pin D8
                        Pulse adj on A0


   Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission
    notice shall be included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.

*/

#include <MIDI.h>
#include <elapsedMillis.h>

/************************* Defines ********************************/
#define DEBUG 0

#if DEBUG == 1
#define dprint(expression) Serial.print("# "); Serial.print( #expression ); Serial.print( ": " ); Serial.println( expression )
#define dshow(expression) Serial.println( expression )
#else
#define dprint(expression)
#define dshow(expression)
#endif

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

//create an instance of MIDI receive on hardware serial
MIDI_CREATE_INSTANCE(HardwareSerial, Serial, myMidi);

elapsedMillis adc_time;
elapsedMicros pin_high;

/************************** Variables *****************************/
uint16_t counter = 0;
volatile boolean new_count = false;

uint8_t oct_bit;
volatile byte clk_arg = 0;
volatile boolean new_clk = false;

/*Time to check the adc and change the mux */
volatile uint16_t adc_check = 16; /* 16 mS */
/*Tell when there is a new sample */
boolean new_sample = false;
/*Store the adc read */
uint16_t adc_read = 0;

/*About the filter  */
const float alpha = .125;
/*Store the previous filtered sample */
uint32_t f_v[3];

/* (16,000,000 Hz divided by */
/* (C7, 2093.005 Hz to C8, 4186.009 Hz  */
/*  times 2^5)) - 1    */
volatile uint16_t clk[] = {
  238, 224, 212, 200, 189, 178, 168, 158, 149, 141, 133, 125, 118,
};

/*The note time for C7 to C8 in microseconds */
uint16_t on_time [] = {
  29, 28, 26, 25, 23, 22, 21, 19, 18, 17, 16, 15, 14,
};

uint16_t wide = 10;
uint16_t new_wide = 0;
uint16_t old_wide = 0;
float pulse_width = 0.5;


/**************************  Functions ****************************/
void handleNoteOn(byte inChannel, byte inNote, byte inVelocity)
{
  oct_bit = (inNote - 36) / 12;
  clk_arg = inNote - (36 + (oct_bit * 12));
  new_clk = true;

  wide = (on_time[clk_arg] << (8 - oct_bit));
}

void handleNoteOff(byte inChannel, byte inNote, byte inVelocity)
{

}

uint16_t filter(uint16_t value) {
  /*Calculate the new value */
  f_v[1] = (float)alpha * value + (1 - alpha) * f_v[0];
  /*Store the old value */
  f_v[0] = f_v[1];

  /*Return the filtered value */
  return f_v[1];
}

/******************************************************************/
/*************************** Setup ********************************/
/******************************************************************/
void setup() {
  if (DEBUG) {
    Serial.begin(115200);
  }
  /************************* Setup Pins ***************************/
  DDRB |= _BV (0); // pinMode (8, OUTPUT);

  //Setup the handlers for the  midi
  myMidi.setHandleNoteOn(handleNoteOn);
  myMidi.setHandleNoteOff(handleNoteOff);
  //Start the midi channel 10
  myMidi.begin(MIDI_CHANNEL_OMNI);

  /*************************  Setup ADC ***************************/
  /*Set to Right Adjust for 1024 precision */
  cbi(ADMUX, ADLAR);

  /*Set to VRef to AVCC */
  cbi(ADMUX, REFS1);
  sbi(ADMUX, REFS0);

  /*Set to ADC0 to start */
  cbi(ADMUX, MUX3);
  cbi(ADMUX, MUX2);
  cbi(ADMUX, MUX1);
  cbi(ADMUX, MUX0);

  /*Set prescaler to 64 */
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  sbi(ADCSRA, ADPS0);

  /*Turn off Auto Trigger */
  cbi(ADCSRA, ADATE);

  /*Turn the ADC ON  */
  sbi(ADCSRA, ADEN);

  /*Start the first conversion */
  sbi(ADCSRA, ADSC);


  /*************************  Setup Timer1 ************************/
  cli();                //stop interrupts
  //set timer1
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register
  OCR1A = clk[0];
  // turn on CTC mode
  sbi(TCCR1B, WGM12);
  /*Set prescaler to 1 */
  cbi(TCCR1B, CS12);
  cbi(TCCR1B, CS11);
  sbi(TCCR1B, CS10);
  // enable timer compare interrupt
  sbi(TIMSK1, OCIE1A);
  sei();                //allow interrupts

  wide = on_time[0] << 3 ;

}/**************************  End Setup **************************/

ISR(TIMER1_COMPA_vect) {
  if (new_clk) {
    OCR1A = clk[clk_arg];
    new_clk = false;
  }
  counter ++;
  new_count = true;

}

/******************************************************************/
/**************************** Loop ********************************/
/******************************************************************/
void loop() {
  byte left_adj = 9 - oct_bit;
  if (new_count) {
    /*This line will toggle the pin acording to the state of the */
    /*bit pointed to in the counter   */
    //((counter >> left_adj) & 0b0000000000000001) == 0 ? PORTB &= ~_BV (0) : PORTB |= _BV (0);

    /*************************************************************/
    /*These lines will set the pin high acording to the bit in the */
    /*counter but uses a timer to turn off the pin    */
    /*The width can be adjusted using the adc  */

    if ((counter >> left_adj) & 0b0000000000000001 == 1) {
      /* Set pin high */
      PORTB |= _BV (0);
      /*Reset the on time counter */
      pin_high = 0;
    }
    new_count = false;
  }

  if (pin_high >= new_wide) {
    /*Set the pin low */
    PORTB &= ~_BV (0);
  }
  /*********************************************************/


  /*Read the midi on ever loop */
  myMidi.read();

  /*  Check A0 acording to the timer */
  if (adc_time >= adc_check) {
    /*Check to see if ADC has finished */
    if (!(bitRead(ADCSRA, ADSC))) {
      /*Read and store the results  */
      uint8_t temp_adcl = ADCL;
      uint16_t temp_adc = (ADCH << 8) + temp_adcl;
      /*Keep a running average */
      adc_read = (adc_read + temp_adc) / 2;
      /*Start the next conversion */
      sbi(ADCSRA, ADSC);
      /*Set the boolean */
      new_sample = true;
    }
    /*Reset the adc count */
    adc_time = 0;
  }

  /************ Update the new sample *****************/
  if (new_sample) {
    /*Send the sample to the filter */
    uint16_t filtered_read = filter(adc_read);
    /*Remap the sample to .1 to .9 */
    pulse_width =  map(filtered_read, 0, 1024, 10, 90) * .01;
    /*Calculte the new width of the pulse */
    uint16_t temp_wide = (float) wide * pulse_width ;
    /*If the value has changed, update the width time */
    if (temp_wide != old_wide) {
      new_wide = temp_wide;
      old_wide = temp_wide;
    }
    new_sample = false;
  }

}/*************************** End Loop *****************************/
