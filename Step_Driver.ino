/*

  Step_Driver
  
  2013-08-09
  David Hash
  dchash@gmail.com
  
  Runs a star tracking mount based on an external calibration
  
  Tracking function is a cubic polynomial approximation of
  the true arcsin curve
  
  Operation:
    Turning on puts tracker into safe mode
      - to begin tracking, press input button
      - tracking will begin 2 seconds after button press
    
    To pause tracking, press input button during operation
      - do not hold more than 3 seconds
      - tracker enters pause mode, denoted by an initially solid
          orange LED, followed by flashing orange
      - to resume tracking, hold button until green and orange LED
          illuminate; tracking will resume a few seconds after
        
    To reverse to starting position, press and hold input button
      during operation, and wait for the green LED to illuminate (3 sec)
      - after alterating LED colors, tracker will reverse to its
          starting position and enter safe mode
      - tracking can be restarted in the same manner as during first
          operation
    
    If allowed to track to electronically limited maximum, tracker
      will stop moving and, after a brief delay, reverse and return
	  to its starting position.
      - Tracker will then enter safe mode and can be restarted.
	  
	  
	  BOARD:
	  
	  Sparkfun Pro Micro 5V, 16 MHz
	  
	  NOTE: there are some commands not compatible with standard Arduino,
	  e.g., the TXLED0; and TXLED1; directives

*/

const int RXLED = 17; // RX LED pin (orange) LOW = enabled
const int DIRPIN = 6; // Direction pin; HIGH = track direction
const int STEPPIN = 10; // Step pin to driver
const int CMDPIN = 2; // Command (button) pin

// Max step count
const unsigned long smax = 441600;

// Coefficients of polynomial fit
const float c_3 = 129.5071e-6;
const float c_2 = 6.136969e-3;
const float c_1 = 63.02467;

const float micro_to_full = 0.0003125;
   // Full rotations per microstep
   
unsigned long t_loop; // keeps track of time inside loop (ms)
float t_pulse; // keeps track of desired pulse time
byte reset = 0; // used for reversing logic

void setup()
{
  pinMode(RXLED, OUTPUT);
  pinMode(DIRPIN, OUTPUT);
  pinMode(STEPPIN, OUTPUT);
  pinMode(CMDPIN, INPUT_PULLUP);
  
  digitalWrite(DIRPIN, HIGH);
  // HIGH = open (sidereal)
  // LOW = close (antisidereal; reverse mode)
  
  //Serial.begin(9600);
  //Serial1.begin(9600);
}

void loop()
{
  // Initially in safe mode; both LEDs lit
  
  digitalWrite(RXLED, LOW);
  TXLED1; // board-specific command to turn green LED on
  
  while(digitalRead(CMDPIN))
  {
    // Safe to unplug during this loop, or can restart
    // guiding from start (i = 1 at breakout) by pressing
    // button to pull CMDPIN low
  }
  
  // LEDs off
  digitalWrite(RXLED, HIGH);
  TXLED0;
  
  delay(2000);
  
  unsigned long i;
  float i_sm;
  float i_sm2; // squared
  float i_sm3; // cubed (these values can become very large, hence float datatype)
  
  unsigned long tic; // used for timing/debugging over serial
  unsigned long toc;
  
  t_loop = millis(); // make note of the start time (ms)
  
  for (i = 1; i < smax; i++)
  {
    i_sm = i*micro_to_full; // convert counter from steps to driveshaft rotations
    i_sm2 = i_sm*i_sm; // rotations squared
    i_sm3 = i_sm2*i_sm; // rotations cubed
    
    // calculate the pulse time -- this is the heart of the mount operation
    // empirical polynomial fit (3rd order), offset by the loop start time
    // to allow for continuation after being reversed/reset
    t_pulse = 1000*((c_3 * i_sm3) + (c_2 * i_sm2) + (c_1 * i_sm)) + t_loop;
    
    // wait until it is time to pulse
    // the majority of the time is spent inside this loop checking button state
    // while waiting to pulse the motor
    while(t_pulse > millis())
    {
      unsigned long t_pause = millis(); // log time for pause timer
      
      // Command input received (at this point could be to either pause or reverse)
      if (!digitalRead(CMDPIN))
      {
        
        // Indicate command received (long orange pulse), wait 3 seconds
        digitalWrite(RXLED, LOW);
        delay(3000);
        digitalWrite(RXLED, HIGH);
        
        // Button is no longer being held down, enter pause mode
        if (digitalRead(CMDPIN))
        {
          // Denoted by flashing orange LED
          // loop waits until button is pressed again to resume operation
          while (digitalRead(CMDPIN))
          {
            digitalWrite(RXLED, LOW);
            delay(10);
            digitalWrite(RXLED, HIGH);
            delay(900);
          }
          
          // Double flash to indicate tracking will resume
          digitalWrite(RXLED, LOW);
          TXLED1;
          delay(1000);
          digitalWrite(RXLED, HIGH);
          TXLED0;
          delay(250);
          digitalWrite(RXLED, LOW);
          TXLED1;
          delay(100);
          digitalWrite(RXLED, HIGH);
          TXLED0;
          
          // offset pulse by pause time + 1 second before
          // returing to standard timing loop
          t_loop = t_loop + millis() - t_pause + 1000;
        }
        else // enter reverse mode if button was still being held down
        {
          // indicate reverse with alternating LED sequence
          TXLED1;
          digitalWrite(RXLED, HIGH);
          delay(500);
          TXLED0;
          digitalWrite(RXLED, LOW);
          delay(500);
          TXLED1;
          digitalWrite(RXLED, HIGH);
          delay(500);
          TXLED0;
          digitalWrite(RXLED, LOW);
          delay(500);
          TXLED1;
          digitalWrite(RXLED, HIGH);
          delay(500);
          TXLED0;
          delay(1000);
          
          digitalWrite(DIRPIN, LOW); // reverse direction
          
          for (; i > 1; i--) // rewind through all traveled points
          {
            TXLED1; // Green LED indicates reverse travel
            digitalWrite(STEPPIN, HIGH);
            
            delayMicroseconds(100);
            
            TXLED0;
            digitalWrite(STEPPIN, LOW);
            
            delayMicroseconds(100);
          }
          
          digitalWrite(DIRPIN, HIGH); // reset to forward direction
          
          // lights on to indicate safe mode
          digitalWrite(RXLED, LOW);
          TXLED1;
          
          while(digitalRead(CMDPIN))
          {
            // Safe to unplug power during this loop, or can restart
            // guiding from start (i = 1 at breakout) by pressing button
          }
          
          // lights off
          digitalWrite(RXLED, HIGH);
          TXLED0;
          
          delay(1000); // delay to wait for finger to lift off button
          
          t_loop = millis() + 1000; // reset time offset + small delay
          reset = 1; // reset flag to skip next pulse
        }
      }
    }
    
    // PULSE THE MOTOR!
    if (!reset) // usually this will be the case
    {
      TXLED1;
      digitalWrite(STEPPIN, HIGH);
      
      
      delay(1);
      
      TXLED0;
      digitalWrite(STEPPIN, LOW);
    }
    else // unless the mount just executed a reverse loop, then don't
    {
      reset = 0; // return to state
    }
    
  } // --- Loop runs to extinction, waits 5 seconds at end of loop, then rewinds and waits --- //


  // // ALTERNATIVE: Wait for user input before reversing
  // // Indicates waiting for reverse command
  //TXLED1;
  
  //while(digitalRead(CMDPIN))
  //{
  //  // Wait for reverse command
  //}
  
  delay(5000); // pause 5 seconds and then reverse to starting position and safe mode

  digitalWrite(DIRPIN, LOW); // reverse mode
          
  for (; i > 1; i--) // rewind
  {
    TXLED1; // Green LED indicates reverse travel
    digitalWrite(STEPPIN, HIGH);
    
    delayMicroseconds(100);
    
    TXLED0;
    digitalWrite(STEPPIN, LOW);
    
    delayMicroseconds(100);
  }
  
  digitalWrite(DIRPIN, HIGH); // reset to forward
  
  delay(1000);
  
  digitalWrite(RXLED, LOW);
  TXLED1;
  
  while(digitalRead(CMDPIN))
  {
    // Safe to unplug during this loop, or can restart
    // guiding from start (i = 1 at breakout)
  }
  
  digitalWrite(RXLED, HIGH);
  TXLED0;
  
  delay(1000); // delay for button press
}
