// I2C device class (I2Cdev) demonstration Arduino sketch for MPU9150
// 1/4/2013 original by Jeff Rowberg <jeff@rowberg.net> at https://github.com/jrowberg/i2cdevlib
//          modified by Aaron Weiss <aaron@sparkfun.com>
//
// Changelog:
//     2011-10-07 - initial release
//     2013-1-4 - added raw magnetometer output

/* Potential bottlenecks: 
 *  IMU configuration
 *    - occurs in registers (usually 8 bit R/W quantities)
 *    - Check registration data description FIFO Enable, I2C Master Control, Default registers
 *      - I2C Master : sets things up
 *      - FIFO Enable : Configures what data is stored in this buffer? Easy to stream to 
 *                      with output buffer
 *    - clock source
 *    - power mode
 *  I2C configuration - Wire.h library
 *    - clock speed
 *  Uart Serial  
 *  Tools
 *    - millis command returns 
 *    - #define (or const uint8) MCU_9150_PM_REG0 B110101101
 *  Proccess  
 *    (1) Throw some data - see if it speeds up - that's the problem! 
 *    (2) Increase baud rate
 *    (3) I2C Configuration (Wire.h) clock speed
 *    (4) IMU Configuration 
 *      4a clock source
 *      4b power mode
 *      4c FIFO configuration
 *      4c write your own setup function? 
 */

/* ============================================
I2Cdev device library code is placed under the MIT license

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

#define LED_PIN 13
#define TEST_PIN 7
bool blinkState = false;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(250000);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);
    pinMode(TEST_PIN, OUTPUT);
}

void loop() {
    // read raw accel/gyro measurements from device
    //accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    // these methods (and a few others) are also available
    accelgyro.getAcceleration(&ax, &ay, &az); //try and see if it's faster
    accelgyro.getRotation(&gx, &gy, &gz);

    // display tab-separated accel/gyro x/y/z values
    // int16_t i = 8; 
    // Serial.write(i); Serial.write(i << 8); 

    Serial.write(ax); Serial.write(ax << 8); // Serial.write with the buffer argument, with array
    Serial.write(ay); Serial.write(ay << 8); 
    Serial.write(az); Serial.write(az << 8); 
    Serial.write(gx); Serial.write(gx << 8); 
    Serial.write(gy); Serial.write(gy << 8); 
    Serial.write(gz); Serial.write(gz << 8); 
    //Serial.write(mx); Serial.write(mx << 8); 
    //Serial.write(my); Serial.write(my << 8); 
    //Serial.write(mz); Serial.write(mz << 8); 

    // Serial.write(i); Serial.write(i << 8); 

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    digitalWrite(TEST_PIN, blinkState); // See how fast the arduino is toggling
}
