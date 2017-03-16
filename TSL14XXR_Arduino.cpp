/*
  TSL14XXR_Arduino.cpp
  Library for reading AMS TSL14XXR family linear photodiode array sensors
  and and IC Haus AMS TSL14XXR compatable sensors, for use on 16 mhz Arduino
  boards and similiar Arduino boards.

  ** (For Teensy 3.X boards, use TSL14XXR_Teensy36 library instead!) ***
  This Arduino library is different from the Teensy36 version of
  the library, because Teensy 3.X boards has a dual ADC where both analog
  input pins can be read simultaneously by the two ADCs.

  Created by Douglas Mayhew,  March 14, 2017.

  MIT License

  Copyright (c) 2017 Douglas Mayhew

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#include "Arduino.h"
#include "TSL14XXR_Arduino.h"

// ===========================================================================================================
// Arduino Pins for the TSL1402R sensor: (right side comments show sensor connections) =======================
// (Note all AMS and IC Haus linear array sensors of this family use similiar inputs and
// outputs and work with this library. Refer to the data sheet for your particular model
// of sensor for sensors not shown below)
byte _CLKpin = 4; // <-- Arduino pin delivering the clock pulses to pin 3(CLK) of the TSL1402R
byte _SIpin = 5;  // <-- Arduino pin delivering the SI (serial-input) pulse to pin 2 of the TSL1402R
byte _Apin1 = 1;  // <-- Arduino pin connected to pin 4 (analog output 1) of the TSL1402R
byte _Apin2 = 2;  // <-- Arduino pin connected to pin 8 (analog output 2) (parallel mode)

// Default Pins for the TSL1410R sensor: =====================================================================
// byte _CLKpin = 4;  // <-- Arduino pin delivering the clock pulses to pin 4(CLK) of the TSL1410R
// byte _SIpin =  5;  // <-- Arduino pin delivering the SI (serial-input) pulse to pin 2 of the TSL1410R
// byte _Apin1 =  1;  // <-- Arduino pin connected to pin 6 (analog output 1) of the TSL1410R
// byte _Apin2 =  2;  // <-- Arduino pin connected to pin 12 (analog output 2) (parallel mode)

//============================================================================================================

// (flushSensorPixelsBeforeRead Notes):
// True = Clock the pixels out, and re-read them again before exiting, remember to add exposureMicroseconds.
// False = Read the pixel values as they are, which have been collecting light since the previous pass.
// Not flushing is about twice as fast, but is not usable with long delays between reads of the sensor,
// long calculations, etc., because the photodiodes saturate while waiting to be read out on the following 
// read cycle. If you set true, remember to set exposureMicroseconds around 3000 to 5000 to bring up the 
// signal level. If you set to false, remember to set exposureMicroseconds to 0;
bool _flushPixelsBeforeRead;

uint16_t _sample1; // temporary place to hold A0 ADC value
uint16_t _sample2; // temporary place to hold A1 ADC value
uint16_t _pixels;  // total number of pixels in the sensor, must be an even number
uint16_t _halfPixels; // half the total pixels

// (exposureMicroseconds Notes): is the number of extra microseconds to allow light to accumulate on all
// the photodiodes. The integration time of the current program / measurement cycle is ~3ms (On Arduino 16 Mhz). 
// Adding delay results in a higher signal level for all pixels.
// If set too low, the sensor may deliver low signals, and set it too high, and it saturates the signal.
// If you do not flush pixels (flushPixelsBeforeRead = false), set exposureMicroseconds to 0,
// because the signal is probably high already, and adding delay makes it likely to saturate hard
// at max signal level for all pixels. If set to zero, we skip the delayMicroseconds() function
// entirely, which might help reduce jitter by not yielding the thread.
uint16_t _exposureMicroseconds;

// A small delay prior to reading each pixel
// Helps reduce artifacts on Arduino.
// Set at 10 to 40 if you use it; generally not higher than 40.
// If set to zero, we skip the delayMicroseconds() function entirely (inside the library), which 
// might help reduce jitter by not yielding the thread.
uint16_t _stabilizeMicroseconds;

TSL14XXR_Arduino::TSL14XXR_Arduino(int CLKpin, int SIpin, int Apin1, int Apin2)
{
  _CLKpin = CLKpin & 0xFF;
  _SIpin = SIpin & 0xFF;
  _Apin1 = Apin1 & 0xFF;
  _Apin2 = Apin2 & 0xFF;
  //init(256, 0, false);
}

// init ======================================================================================================
void TSL14XXR_Arduino::init(uint16_t pixels, bool flushPixelsBeforeRead,
                            uint16_t exposureMicroseconds, uint16_t stabilizeMicroseconds)
{
  // Initialize Arduino pins
  pinMode(_CLKpin, OUTPUT);
  pinMode(_SIpin, OUTPUT);

  pinMode(_Apin1, INPUT);
  pinMode(_Apin2, INPUT);

  _pixels = pixels;
  _halfPixels = _pixels >> 1;
  _flushPixelsBeforeRead = flushPixelsBeforeRead;
  _exposureMicroseconds = exposureMicroseconds;
  _stabilizeMicroseconds = stabilizeMicroseconds;

  // Set output pins low:
  PORTD &= ~(0x01 << _CLKpin); // CLKpin LOW
  PORTD &= ~(0x01 << _SIpin);  // SIpin LOW

  // Clock out any existing SI pulse through the ccd register:
  for (int i = 0; i < _pixels + 2; i++)
  { // for each pixel plus 1
    PORTD |= (0x01 << _CLKpin);  // CLKpin HIGH
    PORTD &= ~(0x01 << _CLKpin); // CLKpin LOW
  }

  // Create a new SI pulse...
  PORTD |= (0x01 << _SIpin);   // SIpin HIGH
  PORTD |= (0x01 << _CLKpin);  // CLKpin HIGH
  PORTD &= ~(0x01 << _SIpin);  // SIpin LOW
  PORTD &= ~(0x01 << _CLKpin); // CLKpin LOW

  // ... and clock out that same SI pulse through the sensor register:
  for (int i = 0; i < _pixels + 2; i++)
  { // for each pixel plus 1
    PORTD |= (0x01 << _CLKpin);  // CLKpin HIGH
    PORTD &= ~(0x01 << _CLKpin); // CLKpin LOW
  }
}
// end init ==================================================================================================

void TSL14XXR_Arduino::readBytesSeries(uint8_t * data)
{
  if (_flushPixelsBeforeRead)
  {
    // Create a new SI pulse:
    PORTD |= (0x01 << _SIpin);   // SIpin HIGH
    PORTD |= (0x01 << _CLKpin);  // CLKpin HIGH
    PORTD &= ~(0x01 << _SIpin);  // SIpin LOW
    PORTD &= ~(0x01 << _CLKpin); // CLKpin LOW

    // flush the pixels
    for (int i = 0; i < _pixels; i ++)
    {
      PORTD |= (0x01 << _CLKpin);  // CLKpin HIGH
      PORTD &= ~(0x01 << _CLKpin); // CLKpin LOW
    }
  }

  if (_exposureMicroseconds > 0)
  {
    // Add _exposureMicroseconds integration time
    delayMicroseconds(_exposureMicroseconds);
  }

  // Create a new SI pulse:
  PORTD |= (0x01 << _SIpin);   // SIpin HIGH
  PORTD |= (0x01 << _CLKpin);  // CLKpin HIGH
  PORTD &= ~(0x01 << _SIpin);  // SIpin LOW
  PORTD &= ~(0x01 << _CLKpin); // CLKpin LOW

  if (_stabilizeMicroseconds > 0)
  {
    // read the pixels (series mode)
    for (int i = 0; i < _pixels; i++)
    {
      // wait for the analog pixel voltage to stabilize
      delayMicroseconds(_stabilizeMicroseconds);

      // read the pixel
      _sample1 = analogRead(_Apin1); // analogRead sample 1

      // send one clock pulse to the sensor
      PORTD |= (0x01 << _CLKpin);  // CLKpin HIGH
      PORTD &= ~(0x01 << _CLKpin); // CLKpin LOW

      // write two uint_8 (bytes) to data[] for each pixel
      data[i << 1] = (_sample1 >> 8) & 0xFF; // AOpin1 HighByte
      data[(i << 1) + 1] = _sample1 & 0xFF;  // AOpin1 LowByte
    }
  } else
  {
    // read the pixels (series mode)
    for (int i = 0; i < _pixels; i++)
    {
      // send one clock pulse to the sensor
      PORTD |= (0x01 << _CLKpin);  // CLKpin HIGH
      PORTD &= ~(0x01 << _CLKpin); // CLKpin LOW

      // write two uint_8 (bytes) to data[] for each pixel
      data[i << 1] = (_sample1 >> 8) & 0xFF; // AOpin1 HighByte
      data[(i << 1) + 1] = _sample1 & 0xFF;  // AOpin1 LowByte
    }
  }

  // The array that was passed into this function should now be filled with sensor pixel data.

}

void TSL14XXR_Arduino::readBytesParallel(uint8_t * data)
{
  if (_flushPixelsBeforeRead)
  {
    // Create a new SI pulse:
    PORTD |= (0x01 << _SIpin);   // SIpin HIGH
    PORTD |= (0x01 << _CLKpin);  // CLKpin HIGH
    PORTD &= ~(0x01 << _SIpin);  // SIpin LOW
    PORTD &= ~(0x01 << _CLKpin); // CLKpin LOW

    // flush the pixels
    for (int i = 0; i < _pixels; i ++)
    {
      PORTD |= (0x01 << _CLKpin);  // CLKpin HIGH
      PORTD &= ~(0x01 << _CLKpin); // CLKpin LOW
    }
  }

  if (_exposureMicroseconds > 0)
  {
    // Add _exposureMicroseconds integration time
    delayMicroseconds(_exposureMicroseconds);
  }

  // Create a new SI pulse:
  PORTD |= (0x01 << _SIpin);   // SIpin HIGH
  PORTD |= (0x01 << _CLKpin);  // CLKpin HIGH
  PORTD &= ~(0x01 << _SIpin);  // SIpin LOW
  PORTD &= ~(0x01 << _CLKpin); // CLKpin LOW

  if (_stabilizeMicroseconds > 0)
  {
    // read the pixels (parallel mode)
    for (int i = 0; i < _halfPixels; i++)
    {
      delayMicroseconds(_stabilizeMicroseconds); // wait for the analog pixel voltage to stabilize

      _sample1 = analogRead(_Apin1);    // analogRead sample 1
      _sample2 = analogRead(_Apin2);    // analogRead sample 2

      PORTD |= (0x01 << _CLKpin);  // CLKpin HIGH
      PORTD &= ~(0x01 << _CLKpin); // CLKpin LOW

      // for each of the 2 pixel values, write two uint_8 (bytes) to data[]
      // Pixel 1:
      data[i << 1] = (_sample1 >> 8) & 0xFF;         // AOpin1 HighByte
      data[(i << 1) + 1] = _sample1 & 0xFF;          // AOpin1 LowByte

      // Pixel 2:
      data[(i + _halfPixels) << 1] = (_sample2 >> 8) & 0xFF;   // AOpin2 HighByte
      data[((i + _halfPixels) << 1) + 1] = _sample2 & 0xFF;    // AOpin2 LowByte
    }
  }
  else
  {
    // read the pixels (parallel mode)
    for (int i = 0; i < _halfPixels; i++)
    {
      _sample1 = analogRead(_Apin1);    // analogRead sample 1
      _sample2 = analogRead(_Apin2);    // analogRead sample 2

      PORTD |= (0x01 << _CLKpin);  // CLKpin HIGH
      PORTD &= ~(0x01 << _CLKpin); // CLKpin LOW

      // for each of the 2 pixel values, write two uint_8 (bytes) to data[]
      // Pixel 1:
      data[i << 1] = (_sample1 >> 8) & 0xFF;         // AOpin1 HighByte
      data[(i << 1) + 1] = _sample1 & 0xFF;          // AOpin1 LowByte

      // Pixel 2:
      data[(i + _halfPixels) << 1] = (_sample2 >> 8) & 0xFF;   // AOpin2 HighByte
      data[((i + _halfPixels) << 1) + 1] = _sample2 & 0xFF;    // AOpin2 LowByte
    }
  }
  // The array that was passed into this function should now be filled with sensor pixel data.

}

void TSL14XXR_Arduino::readIntsSeries(uint16_t * data)
{
  if (_flushPixelsBeforeRead)
  {
    // Create a new SI pulse:
    PORTD |= (0x01 << _SIpin);   // SIpin HIGH
    PORTD |= (0x01 << _CLKpin);  // CLKpin HIGH
    PORTD &= ~(0x01 << _SIpin);  // SIpin LOW
    PORTD &= ~(0x01 << _CLKpin); // CLKpin LOW

    // flush the pixels
    for (int i = 0; i < _pixels; i ++)
    {
      PORTD |= (0x01 << _CLKpin);  // CLKpin HIGH
      PORTD &= ~(0x01 << _CLKpin); // CLKpin LOW
    }
  }

  if (_exposureMicroseconds > 0)
  {
    // Add _exposureMicroseconds integration time
    delayMicroseconds(_exposureMicroseconds);
  }

  // Create a new SI pulse:
  PORTD |= (0x01 << _SIpin);   // SIpin HIGH
  PORTD |= (0x01 << _CLKpin);  // CLKpin HIGH
  PORTD &= ~(0x01 << _SIpin);  // SIpin LOW
  PORTD &= ~(0x01 << _CLKpin); // CLKpin LOW

  if (_stabilizeMicroseconds > 0)
  {
    for (int i = 0; i < _pixels; i++)
    { // for each of pixels (series mode)
      delayMicroseconds(_stabilizeMicroseconds); // wait for the analog pixel voltage to stabilize

      _sample1 = analogRead(_Apin1); // analogRead sample 1

      PORTD |= (0x01 << _CLKpin);  // CLKpin HIGH
      PORTD &= ~(0x01 << _CLKpin); // CLKpin LOW

      // for each pixel value, write one uint_16 (short integer) to data[]
      data[i] = _sample1; // each pixel
    }
  }
  else
  {
    for (int i = 0; i < _pixels; i++)
    { // for each of pixels (series mode)
      _sample1 = analogRead(_Apin1); // analogRead sample 1

      PORTD |= (0x01 << _CLKpin);  // CLKpin HIGH
      PORTD &= ~(0x01 << _CLKpin); // CLKpin LOW

      // for each pixel value, write one uint_16 (short integer) to data[]
      data[i] = _sample1; // each pixel
    }
  }

  // The array that was passed into this function should now be filled with sensor pixel data.

}

void TSL14XXR_Arduino::readIntsParallel(uint16_t * data)
{
  if (_flushPixelsBeforeRead)
  {
    // Create a new SI pulse:
    PORTD |= (0x01 << _SIpin);   // SIpin HIGH
    PORTD |= (0x01 << _CLKpin);  // CLKpin HIGH
    PORTD &= ~(0x01 << _SIpin);  // SIpin LOW
    PORTD &= ~(0x01 << _CLKpin); // CLKpin LOW

    // flush the pixels
    for (int i = 0; i < _pixels; i ++)
    {
      PORTD |= (0x01 << _CLKpin);  // CLKpin HIGH
      PORTD &= ~(0x01 << _CLKpin); // CLKpin LOW
    }
  }

  if (_exposureMicroseconds > 0)
  {
    // Add _exposureMicroseconds integration time
    delayMicroseconds(_exposureMicroseconds);
  }

  // Create a new SI pulse:
  PORTD |= (0x01 << _SIpin);   // SIpin HIGH
  PORTD |= (0x01 << _CLKpin);  // CLKpin HIGH
  PORTD &= ~(0x01 << _SIpin);  // SIpin LOW
  PORTD &= ~(0x01 << _CLKpin); // CLKpin LOW

  if (_stabilizeMicroseconds > 0)
  {
    for (int i = 0; i < _halfPixels; i++)
    { // for half the pixels (parallel mode)
      delayMicroseconds(_stabilizeMicroseconds); // wait for the analog pixel voltage to stabilize
      _sample1 = analogRead(_Apin1); // analogRead sample 1
      _sample2 = analogRead(_Apin2); // analogRead sample 2

      PORTD |= (0x01 << _CLKpin);  // CLKpin HIGH
      PORTD &= ~(0x01 << _CLKpin); // CLKpin LOW

      // for each of the 2 pixel values, write one uint_16 (short integer) to data[]
      data[i] = _sample1; // lower half of pixels
      data[(i + _halfPixels)] = _sample2; // upper half of pixels
    }
  }
  else
  {
    for (int i = 0; i < _halfPixels; i++)
    { // for half the pixels (parallel mode)
      delayMicroseconds(_stabilizeMicroseconds); // wait for the analog pixel voltage to stabilize
      _sample1 = analogRead(_Apin1); // analogRead sample 1
      _sample2 = analogRead(_Apin2); // analogRead sample 2

      PORTD |= (0x01 << _CLKpin);  // CLKpin HIGH
      PORTD &= ~(0x01 << _CLKpin); // CLKpin LOW

      // for each of the 2 pixel values, write one uint_16 (short integer) to data[]
      data[i] = _sample1; // lower half of pixels
      data[(i + _halfPixels)] = _sample2; // upper half of pixels
    }
  }

  // The array that was passed into this function should now be filled with sensor pixel data.

}
