/*
  TSL14XXR_Arduino.h
  Library for reading AMS TSL14XXR family linear photodiode array sensors
  and and IC Haus AMS TSL14XXR compatable sensors, for use on 16 mhz Arduino
  boards and similiar Arduino boards.

  (For Teensy 3.X boards, use TSL14XXR_Teensy36 library instead!
  This Arduino library is different from the TSL14XXR_Teensy36 version of
  the library, because Teensy 3.X boards has a dual ADC where both analog
  input pins can be read simultaneously by the two ADCs, and we make use of
  that feature in the TSL14XXR_Teensy36 library.)

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

#ifndef TSL14XXR_Arduino_h
#define TSL14XXR_Arduino_h

#include "Arduino.h"

class TSL14XXR_Arduino
{
  public:
    TSL14XXR_Arduino(int CLKpin, int SIpin, int Apin1, int Apin2);
    void init(uint16_t pixels, bool flushPixelsBeforeRead,
              uint16_t exposureMicroseconds, uint16_t stabilizeMicroseconds);
    void readBytesSeries(uint8_t * data);
    void readBytesParallel(uint8_t * data);
    void readIntsSeries(uint16_t * data);
    void readIntsParallel(uint16_t * data);

  private:
    byte _CLKpin;
    byte _SIpin;
    byte _Apin1;
    byte _Apin2;

    bool _flushPixelsBeforeRead;
    uint16_t _pixels;
    uint16_t _halfPixels;
    uint16_t _exposureMicroseconds;
    uint16_t _stabilizeMicroseconds;
    uint16_t _sample1;
    uint16_t _sample2;
};

#endif
