//=================================================================================================
// Name: FrequencyMultiplier.ino
// Date: 2016-11-29
// Uses: Digitla frequency multiplier example.
// Author: Andrew Que (http://www.DrQue.net)
//
//                                (C) Copyright 2016 by Andrew Que
//                                       http://www.bb7.com/
//=================================================================================================
#include <avr/sleep.h>

// Pin for multiplied output.
// NOTE: Fixed to the output for OCR1A.
int OUTPUT_PIN = 9;

// Pin for input frequency.
// NOTE: Fixed to the input capture pin.
int INPUT_PIN = 8;

// How many divisions to break the rotation into.
// This is basically the multiplication factor for the incoming frequency.
int MULTIPLIER = 8;

// Timer count when the most recent input pulse arrived.
static unsigned volatile currentTime;

// Timer count when the previous input pulse arrived.
static unsigned volatile lastTime;

// Width of the incoming input pulse (in timer counts).
static unsigned volatile inputPulseWidth;

// Period of a sub-division of the input pulse.
static unsigned volatile divisionPeriod;

// Current subdivision.
static unsigned volatile divisionCount = 0;

int currentOutput;
int freq;
int i;
unsigned long timePassed;
unsigned int interval = 1000;

//-------------------------------------------------------------------------------------------------
// USES:
//   Interrupt for timer 1 compare A.  Used to set new compare value to toggle output pin.
//-------------------------------------------------------------------------------------------------
ISR( TIMER1_COMPA_vect )
{
  // Lag factor.  Used to correct phase once each rotation.  Typically zero.
  int lag = 0;

  // Count this division.
  divisionCount += 1;

  // If half way through the cycle calculate a correction factor for phase.
  if ( ( MULTIPLIER / 2 ) == divisionCount )
    lag = OCR1A - ( currentTime + ( inputPulseWidth / 2 ) );

  // Rollover at pulse count.
  if ( MULTIPLIER == divisionCount )
    divisionCount = 0;

  // Set new next compare value.
  OCR1A += ( divisionPeriod / 2 ) - lag;
}

//-------------------------------------------------------------------------------------------------
// USES:
//   Interrupt for timer 1 capture event.  Used to measure the width of the input pin
// pulse and calculate the division period.
//-------------------------------------------------------------------------------------------------
ISR( TIMER1_CAPT_vect )
{
  // Disable interrupts.
  noInterrupts();

  // Grab input capture.
  // This is the time at which the input pulse had a rising edge.
  currentTime = ICR1;

  // Measure the time between this pulse and the previous pulse.
  inputPulseWidth = currentTime - lastTime;

  // Make the last time the current time for next measurement.
  lastTime = currentTime;

  // Calculate period of divisions.
  // Include long and short divisions.
  divisionPeriod = inputPulseWidth / ( MULTIPLIER / 2);

  // Enable compare output.
  TCCR1A |= 1 << COM1A0;

  // Enable interrupts.
  interrupts();
}

//-------------------------------------------------------------------------------------------------
// USES:
//   Setup I/O and timers for operation.
//-------------------------------------------------------------------------------------------------
void setup()
{
  
  // Set sleep mode for main loop--interrupts will do all the work.
  set_sleep_mode( SLEEP_MODE_IDLE );
  sleep_enable();

  // Setup outputs.
  pinMode( OUTPUT_PIN,  OUTPUT );
  

  //---------------------------------
  // Setup timer 1 for input capture.
  //---------------------------------

  // Disable timer for setup.
  TCCR1B =   0x00;

  // Reset timer count.
  TCNT1 = 0x00;

  // Clear interrupt flags.
  TIFR1 = 0x00;

  // Enable interrupts.
  TIMSK1 =
      ( 1 << ICIE1 )   // Timer/Counter1, Input Capture Interrupt Enable.
    | ( 1 << OCIE1A ); // Timer/Counter1, Output Compare A Match Interrupt Enable.

  // Timer/Counter1 Control Register A to default normal operation.
  // (There were changed by the Arduino startup code.)
  TCCR1A = 0;
  TCCR1B = 0;

  // Setup Input Capture and clock divide rate.
  TCCR1B =
      ( 1 << ICNC1 ) // Enable Input Capture Noise Canceler.
    | ( 1 << ICES1 ) // Set Input Capture Edge Select to rising edge.
    | ( 1 << CS11 ) | ( 1 << CS10 );  // Clock Select to clk/64.
  
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(5, INPUT);
  freq = 0;
  i = 0;
  analogWrite(11,190);
  digitalWrite(12, LOW);
  Serial.begin(9600);
}

//-------------------------------------------------------------------------------------------------
// USES:
//   Main loop.  Just sleeps and allows interrupts to do the work.
//-------------------------------------------------------------------------------------------------
void loop()
{
  // Put the device to sleep:
  
  
  //should detect a change in output, if rising edge increment freq
  if (digitalRead(5) == HIGH && currentOutput == 0) {
    freq ++;
    //Serial.println(currentOutput); 
    currentOutput = 1;
  } else if (digitalRead(5) == LOW && currentOutput == 1){
    currentOutput = 0;
    //Serial.println("low"); 
  }
  
  timePassed = millis();
  if (timePassed >= (i * interval)) {
    Serial.println(freq);
    freq = 0;
    i ++;
  }
  //sleep_mode();
}
