/*
    Kalman Filter Example for MPU6050. Output for processing.
    Read more: http://www.jarzebski.pl/arduino/rozwiazania-i-algorytmy/odczyty-pitch-roll-oraz-filtr-kalmana.html
    GIT: https://github.com/jarzebski/Arduino-KalmanFilter
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski
*/

// CircularBuffer Libraries
#include <CircularBuffer.h>

// MegunoLink Libraries for Exponential Filter
#include "MegunoLink.h"
#include "Filter.h"

#include "helper_3dmath.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include <Wire.h>
#include "MPU6050.h"
#include "KalmanFilter.h"
#include "I2Cdev.h"
MPU6050 mpu;

KalmanFilter kalmanX(0.001, 0.003, 0.03);
KalmanFilter kalmanY(0.001, 0.003, 0.03);

float accPitch = 0;
float accRoll = 0;

float kalPitch = 0;
float kalRoll = 0;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 acc;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector

void setup() 
{
  Serial.begin(115200);

//  // Initialize MPU6050
//  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
//  {
//    delay(500);
//  }

mpu.initialize();
mpu.setDMPEnabled(true);
packetSize = 64;
 
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.CalibrateGyro();
}

void loop()
{
  if (mpu.GetCurrentFIFOPacket(fifoBuffer,packetSize)) {
  mpu.GetAccel(&acc, fifoBuffer);
  Vector gyr = mpu.readNormalizeGyro();
  }
  // Calculate Pitch & Roll from accelerometer (deg)
  accPitch = -(atan2(acc.XAxis, sqrt(acc.YAxis*acc.YAxis + acc.ZAxis*acc.ZAxis))*180.0)/M_PI;
  accRoll  = (atan2(acc.YAxis, acc.ZAxis)*180.0)/M_PI;

  // Kalman filter
  kalPitch = kalmanY.update(accPitch, gyr.YAxis);
  kalRoll = kalmanX.update(accRoll, gyr.XAxis);

  Serial.print(accPitch);
  Serial.print(":");
  Serial.print(accRoll);
  Serial.print(":");
  Serial.print(kalPitch);
  Serial.print(":");
  Serial.print(kalRoll);
  Serial.print(":");
  Serial.print(acc.XAxis);
  Serial.print(":");
  Serial.print(acc.YAxis);
  Serial.print(":");
  Serial.print(acc.ZAxis);
  Serial.print(":");
  Serial.print(gyr.XAxis);
  Serial.print(":");
  Serial.print(gyr.YAxis);
  Serial.print(":");
  Serial.print(gyr.ZAxis);

  Serial.println();
}
