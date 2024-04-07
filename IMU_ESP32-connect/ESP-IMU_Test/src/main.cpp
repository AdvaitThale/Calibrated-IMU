/*
  Author: Advait Thale

  *******************************************************************
                        IMU Interger Converter
  *******************************************************************

  Ths is a test code for determining vibration using IMU. As IMU
  MPU6050 is used which is 6 axis Accelerometer, Gyroscope & Temprature
  sensor. Here, IMUs acceleroeter is used for checking direction of vibrations.
  Refer Register Map and datasheet for furthur details on MPU6050 IMU.

  Pinout of IMU MPU6050:
  ---------------------------------
  |VCC|GND|SCL|SDA|XDA|XCL|AD0|INT|
  ---------------------------------
*/

#include <Arduino.h>
#include <Wire.h> // For comm. with I2C devices
#include <math.h> // Math. Functions

#define ACCELEROMETER_SENSITIVITY 8192.0 // +-2g = 16384 LSB/g
#define GYROSCOPE_SENSITIVITY 131.0       // 250 degrees/s = 131 LSB/degrees/s
#define M_PI 3.14159265359                // Redefine PI
#define BUZZER 11                         // BUZZER Pin

float MAXX, MAXY, MAXZ, MAXT;                    // Calibration variables
int16_t AcX, AcY, AcZ, GyroX, GyroY, GyroZ, Tmp; // 16-bit ints
double x, y, z, tx, t, tf, pitch, roll;
float previousTime, currentTime, elapsedTime;
float gyroAngX, gyroAngY, gyroAngZ;
float AcErrorX, AcErrorY, AcErrorZ;
float GyErrorX, GyErrorY, GyErrorZ;

// Constants for integration
const float dt = 0.01;  // Time step in seconds
float velocity = 0;     // Initial velocity
float displacement = 0; // Initial displacement

// Calibration variables
int AcXcal = -950; // Acceleration Error
int AcYcal = -300;
int AcZcal = 0;
int tcal = -1600; // Temperature correction
int minVal = 265; // Gyroscope correction
int maxVal = 402;

void setup()
{
  Serial.begin(115200);         // Baud Rate
  Wire.begin();                 // Initiate wire lib. and I2C
  Wire.beginTransmission(0x68); // Start transmission to I2C slave
  Wire.write(0x6B);             // Power Management Register (PWR_MGMT_1)
  Wire.write(0);                // Wake up IMU
  Wire.endTransmission(true);   // End transmission to I2C slave
  // beep();
}

void loop()
{
  Wire.beginTransmission(0x68);    // Begin transmission to I2C slave device
  Wire.write(0x3B);                // Starting with register 0x3B (ACCEL_XOUT_H) (Default: Degrees per Sec)
  Wire.endTransmission(false);     // Restarts transmission to I2C slave device
  Wire.requestFrom(0x68, 6, true); // IMU address, request 14 registers in total, true

  // Read registers of Accelerometer and divide raw values by 16384 for +-2g range
  AcX = (Wire.read() << 8 | Wire.read()) / ACCELEROMETER_SENSITIVITY; // 0x3B (ACCEL_XOUT_H) 0x3C (ACCEL_XOUT_L)
  AcY = (Wire.read() << 8 | Wire.read()) / ACCELEROMETER_SENSITIVITY; // 0x3D (ACCEL_YOUT_H) 0x3E (ACCEL_YOUT_L)
  AcZ = (Wire.read() << 8 | Wire.read()) / ACCELEROMETER_SENSITIVITY; // 0x3F (ACCEL_ZOUT_H) 0x40 (ACCEL_ZOUT_L)

  // AcX = Wire.read() << 8 | Wire.read(); // 8 bit shift
  // AcY = Wire.read() << 8 | Wire.read();
  // AcZ = Wire.read() << 8 | Wire.read();

  // AcX /= ACCELEROMETER_SENSITIVITY;
  // AcY /= ACCELEROMETER_SENSITIVITY;
  // AcZ /= ACCELEROMETER_SENSITIVITY;


  Serial.print(AcX);
  Serial.print(" | ");
  Serial.print(AcY);
  Serial.print(" | ");
  Serial.print(AcZ);
  Serial.println(" ");
  delayMicroseconds(100000);
}
