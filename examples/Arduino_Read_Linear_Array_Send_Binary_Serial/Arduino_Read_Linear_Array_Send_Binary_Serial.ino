/* Ardino_Read_Linear_Array_Send_Binary_Serial

  Library for reading AMS TSL14XXR family linear photodiode array sensors
  and and IC Haus AMS TSL14XXR compatable sensors, for use on 16 mhz Arduino
  boards and similiar Arduino boards.

  Created by Douglas Mayhew,  March 10, 2017.

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

  (For Teensy 3.X boards, use Teensy_36_TSL141XXR_To_ILI9341_Display_and_Serial library instead!
  
  This Arduino library is different from the TSL14XXR_Teensy36 version of
  the library, because Teensy 3.X boards has a dual ADC where both analog
  input pins can be read simultaneously by the two ADCs, and we make use of
  that feature in the TSL14XXR_Teensy36 library.)

  Features include:
  You may use with a PC running matching Processing app to xy plot the pixel values, and smoothing &
  subpixel resolution shadow detection from serial data.
  Binary sensor data sent via binary serial with frame sync and encoding to prevent sync byte collisions
  Subpixel resolution shadow width and position calculation that runs here on Arduino
  All pixel data and shadow info telemetry sent over encoded, framed binary serial.
  This sketch is intended to work with seperate Processing.org sketch for viewing live streaming data.
  Other versions of this software exist for running up to 15X faster on Teensy 3.x
  See https://github.com/Mr-Mayhem for updates, variations, and visualizer Processing sketch.
  For PJRC Teensy web forum discussion, see:
  https://forum.pjrc.com/threads/39376-New-library-and-example-Read-TSL1410R-Optical-Sensor-using-Teensy-3-x/

*/

// Includes ==================================================================================================
#include <TSL14XXR_Arduino.h>

// Define various ADC prescaler:==============================================================================
const unsigned char PS_16 = (1 << ADPS2);
const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

// ===========================================================================================================
// Arduino Pins for the TSL1402R sensor: (right side comments show sensor connections) =======================
// (Note all AMS and IC Haus linear array sensors of this family use similiar inputs and
// outputs and work with this library. Refer to the data sheet for your particular model
// of sensor for sensors not shown below)
#define CLKpin 4 // <-- Arduino pin delivering the clock pulses to pin 3(CLK) of the TSL1402R
#define SIpin 5  // <-- Arduino pin delivering the SI (serial-input) pulse to pin 2 of the TSL1402R
#define Apin1 1  // <-- Arduino pin connected to pin 4 (analog output 1) of the TSL1402R
#define Apin2 2  // <-- Arduino pin connected to pin 8 (analog output 2) (parallel mode)

// Default Pins for the TSL1410R sensor: =====================================================================
// #define CLKpin 4 // <-- Arduino pin delivering the clock pulses to pin 4(CLK) of the TSL1410R
// #define SIpin 5  // <-- Arduino pin delivering the SI (serial-input) pulse to pin 2 of the TSL1410R
// #define Apin1 1  // <-- Arduino pin connected to pin 6 (analog output 1) of the TSL1410R
// #define Apin2 2  // <-- Arduino pin connected to pin 12 (analog output 2) (parallel mode)

//// Pins for the ILI9341 Display ==============================================================================
//#define TFT_DC      15
//#define TFT_CS      10
//#define TFT_RST     4 // the screen resets (from all white) by this pin going low for over 100 microseconds
//#define TFT_MOSI    11
//#define TFT_SCLK    13
//#define TFT_MISO    12

// Create Sensor Object ======================================================================================
TSL14XXR_Arduino Sensor(CLKpin, SIpin, Apin1, Apin2);  // sensor object, TSL14XXR_Arduino.h

// constants =================================================================================================
// set the number of pixels on the linear photodiode array sensor. Should be even number
const uint16_t SENSOR_PIXELS = 256; //<<
const uint16_t SENSOR_BYTES = SENSOR_PIXELS * 2;
const uint16_t HALF_SENSOR_PIXELS = SENSOR_PIXELS / 2;
const uint32_t ENCODED_BYTES = (SENSOR_BYTES * 2) + 4; // Double the size of BYTES to gurantee enough room

const uint8_t SPECIAL_BYTE_MINUS1 = 252; // one less than special byte used in encoder for avoiding =>
const uint8_t SPECIAL_BYTE = 253;        // any bytes that => this value are encoded as two bytes
const uint8_t START_MARKER = 254;        // marks the beginning of a serial data frame
const uint8_t END_MARKER = 255;          // sync byte for receiver which uses readBytesUntil()

const uint8_t MODE_DEBUG = 0;            // indicates a serial data frame contains a debug message
const uint8_t MODE_SENSOR_PIXELS = 1;    // indicates a serial data frame contains sensor pixel data
const uint8_t MODE_SHADOW_DATA = 2;      // indicates a serial data frame contains sensor shadow data

// shadow position and width calculation constants
const float sensorPixelSpacingX = 0.0635;           // 63.5 microns
const float sensorPixelsPerMM = 15.74803149606299;  // number of pixels per mm in sensor TSL1402R and TSL1410R
const float sensorWidthAllPixels = 16.256;          // millimeters

// Arrays: ===================================================================================================
// holds data from sensor as integers, fastest for shadow detection, not needed if using sensorByteArray[]
uint16_t sensorIntArray[SENSOR_PIXELS];

//// holds data from sensor as integers, slower, here just to test library readBytes functions.
//uint8_t sensorByteArray[SENSOR_BYTES];

// array for convolution kernel, a gaussian bell curve of 9 points
float kernel[9];
// if you enlarge the kernel length, this should match kernel length

// array for convolution output, size must match kernel size
float output[9];

// holds shadow width and position for sending over serial
uint8_t smallDataArray[8];

// holds shadow width and position encoded into a frame, for sending over serial
uint8_t smallDataArrayEncoded[20];

// kernel length variables ===================================================================================
uint32_t KERNEL_LENGTH;        // number of discrete values in the kernel array, set in setup()
uint32_t KERNEL_LENGTH_MINUS1; // kernel length minus 1, used to reduce math in loops
uint32_t HALF_KERNEL_LENGTH;   // Half the kernel length, used to correct convoltion phase shift

// General Variables =========================================================================================
float input;               // The input data value of a given sensor pixel

float cOutPrev;            // the previous convolution output y value
float cOut;                // the current convolution output y value

uint32_t outerPtrX;        // outer loop pointer
uint32_t outerCount;       // outer loop counter
uint32_t innerPtrX;        // inner loop pointer, used only during convolution

float drawPtrXLessK;
float drawPtrXLessKlessD1;

//uint32_t loopCount;   // increments by one on each loop(); Resets when it reaches updateDisplayModulus

// Main Settings (Designed to be set in Setup(), not here) ===================================================
//bool useDisplay;               // write to ILI9341 Display
bool sendPixelDataViaSerial;   // Sends encoded pixel data frames to a PC via USB Serial
bool sendShadowDataViaSerial;  // Sends encoded shadow position and width data frames to a PC via USB Serial
//uint32_t updateDisplayModulus; // one display update happens every updateDisplayModulus loops in the main loop()

// flushSensorPixelsBeforeRead = true means flush the pixels because they will get saturated
// while CPU is away too long doing other things, or leave the pixels alone if the delay
// between reads is small. Not flushing is much faster, but not usable with long
// delays between reads of the sensor, because the photodiodes saturate while waiting around.
bool flushSensorPixelsBeforeRead;

// exposureMicroseconds is the number of extra microseconds to allow light to accumulate on all
// the photodiodes. The integration time of the current program / measurement cycle is ~3ms
// (On Arduino 16 Mhz). Adding delay results in a higher signal level for all pixels.
// If set too low, the sensor may deliver low signals, and set it too high, and it saturates the signal.
// If you do not flush pixels (flushPixelsBeforeRead = false), set exposureMicroseconds to 0,
// because the signal is probably high already, and adding delay makes it likely to saturate hard
// at max signal level for all pixels. If set to zero, we skip the delayMicroseconds() function
// entirely (inside the library), which might help reduce jitter by not yielding the thread.
uint16_t exposureMicrosec;

// A small delay prior to reading each pixel
// Helps reduce artifacts on Arduino.
// Set at 10 to 40 if you use it; generally not higher than 40.
// If set to zero, we skip the delayMicroseconds() function entirely (inside the library), which
// might help reduce jitter by not yielding the thread.
uint16_t stabilizeMicroseconds;

// Shadow Detector & Subpixel Variables ======================================================================

// x index position of greatest negative y difference peak found in 1st difference data
float negPeakX;

// x index position of greatest positive y difference peak found in 1st difference data
float posPeakX;

float pixelWidthX;         // integer difference between the two peaks without subpixel precision

float negPeakLeftY;        // y value of left neighbor (x - 1) of greatest 1st difference negative peak
float negPeakCenterY;      // y value of 1st difference (x) of greatest negative peak
float negPeakRightY;       // y value of right neighbor (x + 1) of greatest 1st difference negative peak

float posPeakLeftY;        // y value of left neighbor (x - 1) of greatest 1st difference positive peak
float posPeakCenterY;      // y value of 1st difference (x) of greatest positive peak
float posPeakRightY;       // y value of right neighbor (x + 1) of greatest 1st difference positive peak

float negPeakSubPixelX;    // quadratic interpolated negative peak subpixel x position
float posPeakSubPixelX;    // quadratic interpolated positive peak subpixel x position

float preciseWidthX;            // precise shadow width (in pixels with subpixels after decimal point)
float preciseWidthLowPassX;     // precise shadow width filtered with simple running average filter
float preciseWidthXmm;          // precise shadow width converted to mm

float preciseCenterPositionX;   // center position output in pixels
float preciseCenterPositionXlp; // precise center position filtered with simple running average filter
float preciseCenterPositionXmm; // precise center position converted to mm

float CalCoefWidthX;            // corrects mm width by multiplying by this value

String strCenter;               // for text on display
String strWidth;                // for text on display

// diff0Y holds the difference between the current convolution output value and the previous one,
// in the form y[x] - y[x-1]
// If a peak is found in the difference data, this becomes the right side value of the 3 peak values
// which bracket the peak and feed the quadratic interpolation function, which in turn finds the
// subpixel location.
float diff0Y;

// diff1Y holds the difference between the previous convolution output value and the one prior,
// in the form y[x-1] - y[x-2]
// If a peak is found in the difference data, this becomes the center value of the 3 peak values
// which bracket the peak and feed the quadratic interpolation function, which in turn finds the
// subpixel location.
float diff1Y;

// diff2Y holds the difference between the second prior convolution output value and the one prior,
// in the form y[x-2] - y[x-3]
// If a peak is found in the difference data, this becomes the left side value of the 3 peak values
// which bracket the peak and feed the quadratic interpolation function, which in turn finds the
// subpixel location.
float diff2Y;

float negX; // holds X coordinate for the negative peak subpixel position
float cenX; // holds X coordinate for the center subpixel position
float posX; // holds X coordinate for the positive peak subpixel position

// threshold below which peaks are ignored in the 1st difference peak finder
uint32_t diffThresholdY;

// counts the number of subpixel runs in one frame of data
//uint32_t detCounter;

// set true after a negative peak is found with a positive peak, for a complete pair.
bool negPeakFound = false;

uint16_t screenHeight;
uint16_t screenWidth;
uint16_t halfScreenHeight;
uint16_t halfScreenWidth;

// pixel y axis (heightwise) location scaled for the screen
int drawPixelY;

void setup()
{
  // To set up the ADC, first remove bits set by Arduino library, then choose
  // a prescaler: PS_16, PS_32, PS_64 or PS_128:
  ADCSRA &= ~PS_128;
  ADCSRA |= PS_16; // <-- Using PS_32 makes a single ADC conversion take ~30 us

  // Next, assert default setting for the Arduino ADC voltage reference:
  analogReference(DEFAULT);

  // Create the gaussian kernel which we use to smooth the sensor data.
  // The kernel is essentially an array of typically 5 to 10 data points,
  // shaped like a bell curve when xy plotted.
  // The argument is sigma, the sharpness of the peak.
  // We use a sigma of 1.4 which gives a kernel length of 9 elements,
  // and is a good compromise between noise suppression and convolution
  // performance..
  KERNEL_LENGTH = makeGaussKernel1d(1.4);           // should be 9
  KERNEL_LENGTH_MINUS1 = KERNEL_LENGTH - 1;         // should be 8
  HALF_KERNEL_LENGTH = KERNEL_LENGTH_MINUS1 >> 1;   // should be 4

  // enable for test waveform,
  // remember to comment out Sensor.readIntsParallel() in loop(), so it dosen't overwrite the test waveform
  // uint32_t wavelength = 32;
  // uint32_t amplitude = 3200;
  // putPositiveSineWave(sensorIntArray, SENSOR_PIXELS, wavelength, amplitude);

  // putACSineWave(sensorIntArray, SENSOR_PIXELS, 65, 2000)

  // Activate the serial port: ===============================================================================
  // if this is too fast slow it down some, but it worked for me on 16 mhz Arduino UNO
  // Also don't use a cable from a gaming console with one of those ferrite noise suppressors on it,
  // it distorts higher frequency square waves, and wouldn't work for me at 1000000 baud.

  // Turn on the serial port, and set the baud rate to 1 mega baud if possible (1000000)
  Serial.begin(1000000);

  // Main Settings *******************************************************************************************
  // *********************************************************************************************************
  diffThresholdY = 128;   // shadow depths (in neighboring pixels difference value) below this value are ignored
  shadowMin;              // shadow widths (in pixels) below this value are ignored
  shadowMax;              // shadow widths (in pixels) above this value are ignored
  CalCoefWidthX = 0.981;  // corrects mm width by multiplying by this value, I will add auto-calibration later
  //updateDisplayModulus = 100;    // one display update happens every updateDisplayModulus loops in loop()

  // Turn off unneeded features for higher update rates!
  //useDisplay = false;   // write to ILI9341 Display, set to false for higher serial data framerate
  sendPixelDataViaSerial = true;   // Sends encoded pixel data frames to a PC over USB Serial
  sendShadowDataViaSerial = true;  // Sends encoded shadow position and width data frames to a PC via USB Serial

  // (exposureMicroseconds Notes): The number of extra microseconds to allow light to accumulate on all
  // the photodiodes. The integration time of the current program / measurement cycle is ~3ms (On Arduino 16 Mhz).
  // Adding delay results in a higher signal level for all pixels.
  // If set too low, the sensor may deliver low signals, and set it too high, and it saturates the signal.
  // If you do not flush pixels (flushPixelsBeforeRead = false), set exposureMicroseconds to 0,
  // because the signal is probably high already, and adding delay makes it likely to saturate hard
  // at max signal level for all pixels. If set to zero, we skip the delayMicroseconds() function
  // entirely (in the library), which might help reduce jitter by not yielding the thread.
  exposureMicrosec = 0;

  // (flushSensorPixelsBeforeRead Notes):
  // True = Clock the pixels out, and re-read them again before exiting, remember to add exposureMicroseconds.
  // False = Read the pixel values as they are, which have been collecting light since the previous pass.
  // Not flushing is about twice as fast, but is not usable with long delays between reads of the sensor,
  // long calculations, etc., because the photodiodes saturate while waiting to be read out on the following
  // read cycle. If you set true, remember to set exposureMicroseconds around 3000 to 5000 to bring up the
  // signal level. If you set to false, remember to set exposureMicroseconds to 0;
  flushSensorPixelsBeforeRead = false;

  // *********************************************************************************************************
  // End Main Settings ***************************************************************************************

  // Initialize the sensor: ==================================================================================
  //
  // sensorInit arguments are: (uint16_t pixels, bool flushPixelsBeforeRead,
  //                        uint16_t exposureMicroseconds, uint16_t stabilizeMicroseconds)
  Sensor.init(SENSOR_PIXELS, flushSensorPixelsBeforeRead, exposureMicrosec, stabilizeMicroseconds);

  // Display Start =============================================================================================

  //  if (useDisplay)
  //{
  //  tft.begin();
  //    tft.fillScreen(ILI9341_BLACK);
  //    tft.setTextColor(ILI9341_CYAN);
  //    tft.setTextSize(3);
  //    //testFastLines(ILI9341_WHITE, ILI9341_WHITE); // draws a grid once, for testing the screen
  //    delay(200);
  //    tft.useFrameBuffer(true);  // turn on
  //
  //    screenHeight = tft.height();
  //    screenWidth = tft.width();
  //    halfScreenHeight = (screenHeight >> 1);
  //    halfScreenWidth = (screenWidth >> 1);
  //  }

}

// End of Setup() ============================================================================================
// ===========================================================================================================

void loop()
{
  // Slow down for testing. Adjust or comment out as needed.
  //delay(100);

  // fill the intArray with pixel values
  Sensor.readIntsParallel(sensorIntArray); // fills the intArray with pixel values

  // Sends encoded pixel data frames to a PC over USB Serial if enabled
  if (sendPixelDataViaSerial) {
    // encode and send the sensor data via serial to PC
    encodeAndSendIntArray(sensorIntArray, MODE_SENSOR_PIXELS, 0, SENSOR_PIXELS);

  }

  //  if (useDisplay)
  //  {
  //    loopCount++;
  //    if (loopCount % updateDisplayModulus == 0) // every updateDisplayModulus loops
  //    {
  //      loopCount = 0;
  //      // testFastLines(ILI9341_WHITE, ILI9341_WHITE); // draws a grid each loop, for testing the screen
  //      tft.fillScreen(ILI9341_BLACK);
  //      processSensorData(true);
  //      strCenter = String(preciseCenterPositionXmm, 4);
  //      strWidth = String(preciseWidthXmm, 4);
  //
  //      // print text
  //      tft.setCursor(0, 0);
  //      tft.print("Pos: ");
  //      tft.println(strCenter);
  //      tft.print("Wid: ");
  //      tft.print(strWidth);
  //      tft.updateScreen();
  //    } else
  //    {
  //      if (sendShadowDataViaSerial) processSensorData(false);
  //    }
  //  } else
  //  {
  //    if (sendShadowDataViaSerial) processSensorData(false);
  //  }

  if (sendShadowDataViaSerial) sendSensorShadowData(preciseWidthXmm, preciseCenterPositionXmm);

  //  reset the shadow values to zero so they default there if shadows fall below thresholds & tolerances
  preciseWidthX = 0;
  preciseCenterPositionX = 0;
  preciseCenterPositionXmm = 0;
  preciseWidthXmm = 0;

}

// End of Loop() =============================================================================================

void encodeAndSendIntArray(uint16_t * inData, uint8_t modeByte, uint8_t lengthIndicatorByte,
                           uint32_t numOriginalInts) {
  // Function that encodes the passed integer array to avoid sync byte collisions, and adds
  // START_MARKER, modeByte, and lengthIndicatorByte in the beginning, and ends it with END_MARKER.
  // Sends it all out over the serial port.
  // This version simply sends the bytes off on their merry way via serial, because storing them
  // sucks up most of an Arduino UNO's 2 meg of memory.

  // add signalling bytes:
  // insert the START_MARKER
  Serial.write(START_MARKER);
  // insert the modeByte
  Serial.write(modeByte);            // << mode byte must be less than SPECIAL_BYTE
  // insert the lengthIndicatorByte
  Serial.write(lengthIndicatorByte); // << unused byte must be less than SPECIAL_BYTE

  // encodes any bytes of 253 or more into a pair of bytes, 253 0, 253 1 or 253 2, as appropriate.
  // this sends any bytes of value 253 to 255 as two bytes, to prevent collisions with sync.
  // The bytes are decoded on the other side by reversing the process.
  uint32_t dataTotalSend = 3; // 3 because we begin data immediately after the first 3 signalling bytes
  uint8_t byte1;
  uint8_t byte2;
  uint8_t byte1Modified;
  uint8_t byte2Modified;
  uint32_t i;

  for (i = 0; i < numOriginalInts; i++)
  {
    byte1 = (inData[i] >> 8) & 0xFF; // HighByte of inData
    byte2 = inData[i] & 0xFF; // LowByte of inData

    // encode byte1
    if (byte1 > SPECIAL_BYTE_MINUS1)    // if byte1 > 252
    {
      Serial.write(SPECIAL_BYTE);       // write SPECIAL_BYTE 253
      dataTotalSend++;
      byte1Modified = byte1 - SPECIAL_BYTE; // encode the byte
      Serial.write(byte1Modified);          // write the encoded byte
    } else {
      Serial.write(byte1);              // write the original, unaltered byte
    }
    dataTotalSend++;

    // encode byte2
    if (byte2 > SPECIAL_BYTE_MINUS1)    // if byte2 > 252
    {
      Serial.write(SPECIAL_BYTE);       // write SPECIAL_BYTE 253
      dataTotalSend++;
      byte2Modified = byte2 - SPECIAL_BYTE; // encode the byte
      Serial.write(byte2Modified);          // write the encoded byte
    } else {
      Serial.write(byte2);              // write the original, unaltered byte
    }
    dataTotalSend++;
  }

  // insert the END_MARKER
  Serial.write(END_MARKER);
  dataTotalSend++;

}

uint32_t encodeAndSendByteArray(uint8_t * inData, uint8_t modeByte, uint8_t lengthIndicatorByte, uint32_t numOriginalBytes) {
  // Function that encodes the passed byte array to avoid sync byte collisions, prefixes the data sent with
  // START_MARKER, modeByte, and lengthIndicatorByte in the beginning, and ends with END_MARKER.
  // Sends it all out over the serial port.

  // adds signalling bytes, copies inData to OutData, and encodes any
  // bytes of 253 or more into a pair of bytes, 253 0, 253 1 or 253 2, as appropriate

  uint32_t dataTotalSend = 3; // 3 because we offset data to follow the first 3 signalling bytes
  uint8_t byte1;
  uint8_t byte1Modified;
  uint32_t i;

  // insert the START_MARKER
  Serial.write(START_MARKER);

  // insert the modeByte
  Serial.write(modeByte);            // << mode byte must be less than SPECIAL_BYTE

  // insert the lengthIndicatorByte
  Serial.write(lengthIndicatorByte); // << unused byte must be less than SPECIAL_BYTE

  for (i = 0; i < numOriginalBytes; i++)
  {
    byte1 = inData[i];

    // encode byte1
    if (byte1 > SPECIAL_BYTE_MINUS1)        // if byte1 > 252
    {
      Serial.write(SPECIAL_BYTE);           // write SPECIAL_BYTE 253
      dataTotalSend++;
      byte1Modified = byte1 - SPECIAL_BYTE; // encode the byte
      Serial.write(byte1Modified);          // write the encoded byte
    } else {
      Serial.write(byte1);                  // write the original, unaltered byte
    }
    dataTotalSend++;
  }

  // insert the END_MARKER
  Serial.write(END_MARKER);
  dataTotalSend++;

  return dataTotalSend;
}

void processSensorData(bool drawData)
{
  outerCount = 0;          // outer loop counter, increments once each outer loop
  //detCounter = 0;        // counts the number of subpixel shadows
  // increment the outer loop pointer

  //  if (drawData)
  //  {
  //    for (outerPtrX = 0; outerPtrX < SENSOR_PIXELS; outerPtrX++)
  //    {
  //      outerCount++; // lets us index (x axis) on the screen offset from outerPtrX
  //
  //      // shift left by half the kernel size to correct for convolution shift
  //      // (dead-on correct for odd-size kernels)
  //      drawPtrXLessK = outerCount - (HALF_KERNEL_LENGTH);
  //
  //      // same as above, but shift left additional 0.5 to properly place the difference point in-between it's
  //      // parents
  //      drawPtrXLessKlessD1 = drawPtrXLessK - 0.5;
  //
  //      input = sensorIntArray[outerPtrX]; // this is faster integer method, why convert to bytes and back?
  //
  //      // recombine byte pair to get integer pixel value from sensorByteArray
  //      // (slower test option, better to use ints directly from the sensor)
  //      // input = (sensorByteArray[outerPtrX << 1] << 8 | (sensorByteArray[(outerPtrX << 1) + 1] & 0xFF));
  //
  //      // Draw one original sensor pixel
  //      // cast the pixel float value to an int, and divide by 16 to scale it down so it fits vertically on the screen.
  //      drawPixelY = (int) input >> 4;
  //      if (drawPixelY > screenHeight)  drawPixelY = screenHeight; // constrain the value to fit on the screen
  //      tft.drawPixel(outerCount, screenHeight - drawPixelY, ILI9341_WHITE);
  //
  //      // Convolution Inner Loop Convolves the original data with the kernel to smooth it to reduce noise =======
  //      cOutPrev = cOut; // y[output-1] (the previous convolution output value)
  //
  //      // increment the inner loop pointer
  //      for (innerPtrX = 0; innerPtrX < KERNEL_LENGTH_MINUS1; innerPtrX++) {
  //        // convolution: multiply and accumulate
  //        output[innerPtrX] = output[innerPtrX + 1] + (input * kernel[innerPtrX]);
  //      }
  //      // convolution: multiply only, no accumulate
  //      output[KERNEL_LENGTH_MINUS1] = input * kernel[KERNEL_LENGTH_MINUS1];
  //      cOut = output[0]; // y[output] (the current convolution output value)
  //      // end convolution =======================================================================================
  //
  //      if (outerCount > KERNEL_LENGTH_MINUS1) {  // Skip one kernel length of convolution output values,
  //        // which are garbage due to the kernel not being fully immersed in the input signal.
  //
  //        // Draw one sensor pixel after smoothing via convolution with a gaussian kernel
  //        // cast the pixel float value to an int, and divide by 16 to scale it down so it fits vertically on the screen.
  //        drawPixelY = (int) (cOut) >> 4;
  //        if (drawPixelY > screenHeight)  drawPixelY = screenHeight; // constrain the value to fit on the screen
  //        tft.drawPixel(drawPtrXLessK, screenHeight - drawPixelY, ILI9341_PURPLE); // x position uses pre-shifted value
  //
  //        find1stDiffPeaks(drawData);
  //      }
  //    }
  //  } else
  //  {
  for (outerPtrX = 0; outerPtrX < SENSOR_PIXELS; outerPtrX++) {
    outerCount++; // lets us index (x axis) on the screen offset from outerPtrX

    // shift left by half the kernel size to correct for convolution shift
    // (dead-on correct for odd-size kernels)
    drawPtrXLessK = outerCount - (HALF_KERNEL_LENGTH);

    // same as above, but shift left additional 0.5 to properly place the difference point in-between it's
    // parents
    drawPtrXLessKlessD1 = drawPtrXLessK - 0.5;

    input = sensorIntArray[outerPtrX]; // this is faster integer method, why convert to bytes and back?

    // recombine byte pair to get integer pixel value from sensorByteArray
    // (slower test option, better to use ints directly from the sensor)
    // input = (sensorByteArray[outerPtrX << 1] << 8 | (sensorByteArray[(outerPtrX << 1) + 1] & 0xFF));

    // Convolution Inner Loop Convolves the original data with the kernel to smooth it to reduce noise =======
    cOutPrev = cOut; // y[output-1] (the previous convolution output value)

    // increment the inner loop pointer
    for (innerPtrX = 0; innerPtrX < KERNEL_LENGTH_MINUS1; innerPtrX++) {
      // convolution: multiply and accumulate
      output[innerPtrX] = output[innerPtrX + 1] + (input * kernel[innerPtrX]);
    }
    // convolution: multiply only, no accumulate
    output[KERNEL_LENGTH_MINUS1] = input * kernel[KERNEL_LENGTH_MINUS1];
    cOut = output[0]; // y[output] (the current convolution output value)
    // end convolution =======================================================================================

    if (outerCount > KERNEL_LENGTH_MINUS1) {  // Skip one kernel length of convolution output values,
      // which are garbage due to the kernel not being fully immersed in the input signal.

      find1stDiffPeaks(drawData);
    }
  }
  //  } Commented out to balance the above commenting out
}

void find1stDiffPeaks(bool drawData)
{
  // =================== Find the 1st difference and store the last two values  ==============================
  // finds the differences and maintains a history of the previous 2 difference values as well,
  // so we can collect all 3 points bracketing a pos or neg peak, needed to feed the subpixel code.

  diff2Y = diff1Y;    // (left y value)
  diff1Y = diff0Y;    // (center y value)
  // find 1st difference of the convolved data, the difference between adjacent points in the smoothed data.
  diff0Y = cOut - cOutPrev; // (right y value) // << The first difference is the difference between the current
  // convolution output value and the previous one, in the form y[x] - y[x-1]
  // In dsp, this difference is preferably called the "first difference", but some texts call it the
  // "first derivative", and some texts refer to each difference value produced above as a "partial derivative".

  // ====================================== End 1st difference ===============================================

  //  if (drawData)
  //  {
  //    // Draw one first difference (partial derivative) pixel
  //    // cast the pixel float value to an int, and divide by 16 to scale it down so it fits vertically on the screen.
  //    drawPixelY = (int) diff0Y >> 4;
  //    if (drawPixelY > screenHeight)  drawPixelY = screenHeight; // constrain the value to fit on the screen
  //    tft.drawPixel(drawPtrXLessKlessD1, halfScreenHeight - drawPixelY, ILI9341_GREEN); // x position uses pre-shifted value
  //  }

  // ======================================== Peak Finder ====================================================
  if (abs (diff1Y) > diffThresholdY) { // if the absolute value of the peak is above the threshold value
    // if diff1Y is a negative peak relative to the neighboring values
    if (diff1Y < diff0Y && diff1Y < diff2Y) {

      // x-1 and x-0.5 for difference being in-between original data
      negPeakX = (outerPtrX - 0.5) - HALF_KERNEL_LENGTH;
      negPeakRightY = diff0Y;   // y value @ x index -1 (right)
      negPeakCenterY = diff1Y;  // y value @ x index -2 (center) (negative 1st difference peak location)
      negPeakLeftY = diff2Y;    // y value @ x index -3 (left)
      negPeakFound = true;

      // if diff1Y is a positive peak relative to the neighboring values
    } else if (diff1Y > diff0Y && diff1Y > diff2Y) {
      posPeakX = (outerPtrX - 0.5) - HALF_KERNEL_LENGTH;
      posPeakRightY = diff0Y;   // y value @ x index -1 (right)
      posPeakCenterY = diff1Y;  // y value @ x index -2 (center) (positive 1st difference peak location)
      posPeakLeftY = diff2Y;    // y value @ x index -3 (left)

      // insures that pairs of peaks(one negative, one positive) are fed to subpixelCalc
      if (negPeakFound) {

        // calculate, display, and store the subpixel estimate associated with this peak pair
        subpixelCalc(drawData);
        negPeakFound = false; // reset for next time around

        negPeakX = 0;
        negPeakRightY = 0;
        negPeakCenterY = 0;
        negPeakLeftY = 0;

        posPeakX = 0;
        posPeakRightY = 0;
        posPeakCenterY = 0;
        posPeakLeftY = 0;
      }
    }
  }
}

void subpixelCalc(bool drawData) {

  pixelWidthX = posPeakX - negPeakX;

  // check for width in acceptable range, what is acceptable is up to you, within reason.
  // was originally 'pixelWidthX < 103' for filiment width sketch, (15.7pixels per mm, 65535/635=103)
  if (pixelWidthX > shadowMin && pixelWidthX < shadowMax) { // if pixel-based width is within this range

    // sub-pixel edge detection using interpolation
    // from Accelerated Image Processing blog, posting: Sub-Pixel Maximum
    // https://visionexperts.blogspot.com/2009/03/sub-pixel-maximum.html

    // for the subpixel value of the greatest negative peak found above,
    // corresponds with the left edge of a narrow shadow cast upon the sensor
    negPeakSubPixelX = 0.5 * ((negPeakLeftY - negPeakRightY) / (negPeakLeftY - (2 * negPeakCenterY) + negPeakRightY));

    // for the subpixel value of the greatest positive peak found above,
    // corresponds with the right edge of a narrow shadow cast upon the sensor
    posPeakSubPixelX = 0.5 * ((posPeakLeftY - posPeakRightY) / (posPeakLeftY - (2 * posPeakCenterY) + posPeakRightY));

    // original function translated from flipper's filament width sensor;
    //does the same math calculation as above
    // negPeakSubPixelX=((a1-c1) / (a1+c1-(b1*2)))/2;
    // posPeakSubPixelX=((a2-c2) / (a2+c2-(b2*2)))/2;

    preciseWidthX = pixelWidthX + (posPeakSubPixelX - negPeakSubPixelX);

    // apply a simple low pass filter to the Width mm output
    //preciseWidthLowPassX = (preciseWidthLowPassX * 0.9) + (preciseWidthX * 0.1);
    preciseWidthXmm = preciseWidthX * sensorPixelSpacingX * CalCoefWidthX;

    // solve for the center position by adding the left and right pixel and
    // subpixel locations up, and then dividing the sum by 2
    preciseCenterPositionX = (((negPeakX + negPeakSubPixelX) + (posPeakX + posPeakSubPixelX)) / 2);

    // copy the subpixel center value to the detections array,
    //useful downstream for multi-shadow calculations & averaging
    //detections[detCounter] = preciseCenterPosX;
    // increment the detection counter
    //detCounter++;

    // spacing on the sensor die calibration
    preciseCenterPositionXmm = preciseCenterPositionX * sensorPixelSpacingX;

    //    if (drawData) {
    //      // Draw a vertical line to show this shadow's center position on the screen,
    //      // we don't draw subpixel resolution because we draw one pixel per sensor reading,
    //      // so nothing in-between on x axis.
    //      // (we have no zoom-in feature with pixels spread out... yet)
    //      uint16_t ScreenCenX = (uint16_t) preciseCenterPositionX;
    //      tft.drawFastVLine(ScreenCenX, halfScreenHeight - 40, halfScreenHeight + 40, ILI9341_WHITE);
    //    }
  }
}

// available outputs from the subpixel routine

// fractional decimal shadow subpixel falling edge from quadratic interpolation function
//  negPeakSubPixelX

// fractional decimal shadow subpixel rising shadow edge from quadratic interpolation function
//  posPeakSubPixelX

//  preciseWidthX     // the shadow width
//  preciseWidthMMX   // the shadow width in millimeters (mm)
//  preciseCenterPosX // the shadow center position
//  precieseCenterMMX // the shadow center position in millimeters (mm)
//  detCounter        // number of shadows detected (not implemented here yet)

// ===========================================================================================================

void sendSensorShadowData(float value1, float value2) {
  // fill sensor array with one int value converted into 4 bytes

  uint32_t index = 0;

  index += floatTo4Bytes(0, smallDataArray, value1);
  index += floatTo4Bytes(index, smallDataArray, value2);

  uint32_t bytesTX =  encodeHighBytes(smallDataArray, smallDataArrayEncoded,
                                      MODE_SHADOW_DATA, 8, 8);

  // send the completed data frame over serial to a PC
  Serial.write(smallDataArrayEncoded, bytesTX);
}

uint32_t encodeHighBytes(uint8_t * inData, uint8_t * outData,
                         uint8_t modeByte, uint8_t lengthIndicatorByte, uint32_t numOriginalBytes) {

  // adds signalling bytes, copies inData to OutData, and encodes any
  // bytes of 253 or more into a pair of bytes, 253 0, 253 1 or 253 2, as appropriate

  // insert the START_MARKER
  outData[0] = START_MARKER;

  // insert the modeByte
  outData[1] = modeByte;  // << mode byte must be less than SPECIAL_BYTE

  // insert the lengthIndicatorByte
  outData[2] = lengthIndicatorByte; // << unused byte must be less than SPECIAL_BYTE

  uint32_t dataTotalSend = 3; // 3 because we offset data to follow the first 3 signalling bytes

  for (uint32_t n = 0; n < numOriginalBytes; n++)
  {
    if (inData[n] > SPECIAL_BYTE_MINUS1)
    {
      outData[dataTotalSend] = SPECIAL_BYTE;
      dataTotalSend++;
      outData[dataTotalSend] = inData[n] - SPECIAL_BYTE;
    } else
    {
      outData[dataTotalSend] = inData[n];
    }
    dataTotalSend++;
  }

  // insert the END_MARKER
  outData[dataTotalSend] = END_MARKER;
  dataTotalSend++;

  return dataTotalSend;
}

uint32_t encodeIntArray(uint16_t * inData, uint8_t * outData,
                        uint8_t modeByte, uint8_t lengthIndicatorByte, uint32_t numOriginalInts) {

  // adds signalling bytes, copies inData to OutData, and encodes any
  // bytes of 253 or more into a pair of bytes, 253 0, 253 1 or 253 2, as appropriate

  // insert the START_MARKER
  outData[0] = START_MARKER;

  // insert the modeByte
  outData[1] = modeByte;  // << mode byte must be less than SPECIAL_BYTE

  // insert the lengthIndicatorByte
  outData[2] = lengthIndicatorByte; // << unused byte must be less than SPECIAL_BYTE

  uint32_t dataTotalSend = 3; // 3 because we offset data to follow the first 3 signalling bytes

  for (uint32_t n = 0; n < numOriginalInts; n++)
  {
    uint8_t byte1 = (inData[n] >> 8) & 0xFF; // HighByte
    uint8_t byte2 = inData[n] & 0xFF; // LowByte

    // encode byte1
    if (byte1 > SPECIAL_BYTE_MINUS1)
    {
      outData[dataTotalSend] = SPECIAL_BYTE;
      dataTotalSend++;
      outData[dataTotalSend] = byte1 - SPECIAL_BYTE;
    }  else
    {
      outData[dataTotalSend] = byte1;
    }
    dataTotalSend++;

    // encode byte2
    if (byte2 > SPECIAL_BYTE_MINUS1)
    {
      outData[dataTotalSend] = SPECIAL_BYTE;
      dataTotalSend++;
      outData[dataTotalSend] = byte2 - SPECIAL_BYTE;
    } else
    {
      outData[dataTotalSend] = byte2;
    }
    dataTotalSend++;
  }

  // insert the END_MARKER
  outData[dataTotalSend] = END_MARKER;
  dataTotalSend++;
  return dataTotalSend;
}

uint32_t copyIntsToBytes(uint16_t * inData, uint32_t inLength, uint8_t * outData) {
  uint32_t i = 0;

  // convert inData integer values into byte pairs, and copy into outData
  for (i = 0; i < inLength; i++) {
    outData[i << 1] = (inData[i] >> 8) & 0xFF;         // AOpin1 HighByte
    outData[(i << 1) + 1] = inData[i] & 0xFF;
  }
  return i;
}

uint32_t makeGaussKernel1d (double sigma) {
  /**
     This sample code is made available as part of the book "Digital Image
     Processing - An Algorithmic Introduction using Java" by Wilhelm Burger
     and Mark J. Burge, Copyright (C) 2005-2008 Springer-Verlag Berlin,
     Heidelberg, New York.
     Note that this code comes with absolutely no warranty of any kind.
     See http://www.imagingbook.com for details and licensing conditions.

     Date: 2007/11/10

    kernel height rescaling (which normalizes all values to sum to 1) code
    added here in Linerar Array Subpixel Visualizer by
    Doug Mayhew, November 20 2016

    code found also at:
    https://github.com/biometrics/imagingbook/blob/master/src/gauss/GaussKernel1d.java
  */

  // scaling variables
  double sum = 0;
  double scale = 1;

  // make 1D Gauss filter kernel large enough
  // to avoid truncation effects (too small in ImageJ!)
  int center = (int) (3.0 * sigma);
  int kernelLen = 2 * center + 1;
  double kerneldb[kernelLen]; // odd size
  double sigma2 = sigma * sigma;

  for (int i = 0; i < kernelLen; i++) {
    double r = center - i;
    kerneldb[i] =  (double) exp(-0.5 * (r * r) / sigma2);
    sum += kerneldb[i];
    //println("kernel[" + i + "] = " + kerneldb[i]);
  }

  if (sum != 0.0) {
    scale = 1.0 / sum;
  } else {
    scale = 1;
  }

  //println("gaussian kernel scale = " + scale); // print the scale.
  sum = 0; // clear the previous sum
  // scale the kernel values
  for (int i = 0; i < kernelLen; i++) {
    kernel[i] = (float)(kerneldb[i] * scale);
    sum += kernel[i];
    // print the kernel value.
    //println("scaled gaussian kernel[" + i + "]:" + tempFloatArray[i]);
  }

  if (sum != 0.0) {
    scale = 1.0 / sum;
  } else {
    scale = 1;
  }

  // print the new scale. Should be very close to 1.
  //println("gaussian kernel new scale = " + scale);

  //println("KERNEL_LENGTH = " + KERNEL_LENGTH);
  return kernelLen;
}


int floatTo4Bytes(uint32_t index, uint8_t * outData, float floatValue)
{
  uint8_t * b = (uint8_t *) &floatValue;

  outData[index] = b[0];
  index++;

  outData[index] = b[1];
  index++;

  outData[index] = b[2];
  index++;

  outData[index] = b[3];
  index++;
  return index;
}

void debugToPC(String text)
{
  // ASCII is below 128 so no need to encode it
  Serial.write(START_MARKER);
  Serial.write(MODE_DEBUG); // 0
  Serial.write(0);  // must be less than SPECIAL_BYTE
  Serial.print(text);          // the text message to send
  Serial.write(END_MARKER);
}
