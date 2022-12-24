/*
  Author: Advait Thale

  *******************************************************************
                           IMU Vibration
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

#include <Wire.h> // For comm. with I2C devices
#include <math.h> // Math. Functions

#define ACCELEROMETER_SENSITIVITY 16384.0 // +-2g = 16384 LSB/g
#define GYROSCOPE_SENSITIVITY 131.0       // 250 degrees/s = 131 LSB/degrees/s
#define M_PI 3.14159265359                // Redefine PI
#define dt 0.01                           // 10 ms Sample rate
#define BUZZER 11                         // BUZZER Pin


float MAXX, MAXY, MAXZ, MAXT; // Calibration variables
int16_t AcX, AcY, AcZ, GyroX, GyroY, GyroZ, Tmp;       // 16-bit ints
double x, y, z, tx, t, tf, pitch, roll;
float previousTime, currentTime, elapsedTime;
float gyroAngX, gyroAngY, gyroAngZ;
float AcErrorX, AcErrorY, AcErrorZ;
float GyErrorX, GyErrorY, GyErrorZ;

// Calibration variables
int AcXcal = -950; // Acceleration Error
int AcYcal = -300;
int AcZcal = 0;
int tcal = -1600; // Temperature correction
int minVal = 265; // Gyroscope correction
int maxVal = 402;

void setup()
{
  Wire.begin();                // Initiate wire lib. and I2C
  Wire.beginTransmission(0x68); // Start transmission to I2C slave
  Wire.write(0x6B);            // Power Management Register (PWR_MGMT_1)
  Wire.write(0);               // Wake up IMU
  Wire.endTransmission(true);  // End transmission to I2C slave
  Serial.begin(115200);        // Baud Rate
  pinMode(BUZZER, OUTPUT);     // Set to OUTPUT for Buzzer Pin
  beep();
}

void loop()
{
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time
  elapsedTime = (currentTime - previousTime) / 1000; // Dividing by 1000 to get Elapsed time in seconds

  Wire.beginTransmission(0x68);     // Begin transmission to I2C slave device
  Wire.write(0x3B);                 // Starting with register 0x3B (ACCEL_XOUT_H) (Default: Degrees per Sec)
  Wire.endTransmission(false);      // Restarts transmission to I2C slave device
  Wire.requestFrom(0x68, 14, true); // IMU address, request 14 registers in total, true

  // Read registers of Accelerometer and divide raw values by 16384 for +-2g range
  AcX = (Wire.read() << 8 | Wire.read()) / ACCELEROMETER_SENSITIVITY; // 0x3B (ACCEL_XOUT_H) 0x3C (ACCEL_XOUT_L)
  AcY = (Wire.read() << 8 | Wire.read()) / ACCELEROMETER_SENSITIVITY; // 0x3D (ACCEL_YOUT_H) 0x3E (ACCEL_YOUT_L)
  AcZ = (Wire.read() << 8 | Wire.read()) / ACCELEROMETER_SENSITIVITY; // 0x3F (ACCEL_ZOUT_H) 0x40 (ACCEL_ZOUT_L)

  // Read register of Temperature data
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) 0x42 (TEMP_OUT_L)

  // Read registers of Gyroscope and divide raw values by 131 for 250 deg/s range
  GyroX = (Wire.read() << 8 | Wire.read()) / GYROSCOPE_SENSITIVITY; // 0x43 (GYRO_XOUT_H) 0x44 (GYRO_XOUT_L)
  GyroY = (Wire.read() << 8 | Wire.read()) / GYROSCOPE_SENSITIVITY; // 0x45 (GYRO_YOUT_H) 0x46 (GYRO_YOUT_L)
  GyroZ = (Wire.read() << 8 | Wire.read()) / GYROSCOPE_SENSITIVITY; // 0x47 (GYRO_ZOUT_H) 0x48 (GYRO_ZOUT_L)

  gyroAngX = (gyroAngX + GyroX) * elapsedTime; // Converting into degrees (deg=deg/s*s)
  gyroAngY = (gyroAngY + GyroY) * elapsedTime;
  gyroAngZ = (gyroAngZ + GyroZ) * elapsedTime;

  //  int xAng = map(AcX, minVal, maxVal, -90, 90);
  //  int yAng = map(AcY, minVal, maxVal, -90, 90);
  //  int zAng = map(AcZ, minVal, maxVal, -90, 90);

  //  x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);//Angular Conversion rad to deg
  //  y = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
  //  z = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);

  // Complementary Filter to add up Gyroscope values and Accelerometer
  double Fx = (0.96 * GyroX) + (0.04 * AcX);
  double Fy = (0.96 * GyroY) + (0.04 * AcY);
  double Fz = (0.96 * GyroZ) + (0.04 * AcZ);

  // Temperature Calculation
  tx = Tmp + tcal;
  t = (tx / 340) + 36.53;    // Temperature in degrees C (from datasheet)
  //tf = (t * 9 / 5) + 32; // Celsius to Fahrenheit

  //  Serial.print(" ");
  //  Serial.print(gyroAngX);
  //  Serial.print(" | ");
  //  Serial.print(abs(gyroAngY));
  //  Serial.print(" | ");
  //  Serial.println(gyroAngZ);

  Serial.print("MAXX "); Serial.print(Fx);
  Serial.print(" MAXY "); Serial.print(Fy);
  Serial.print(" MAXZ "); Serial.print(Fz);
  Serial.print(" MAXT: "); Serial.println(t);

  //  Serial.print("MAXX "); Serial.print(AcX + AcXcal);
  //  Serial.print(" MAXY "); Serial.print(AcY + AcYcal);
  //  Serial.print(" MAXZ "); Serial.print(AcZ + AcZcal);
  //  Serial.print(" MAXT: "); Serial.println(t);
  //Serial.print(" fahrenheit = "); Serial.println(tf);
  
}

void calculate_error() {
  int c = 0;
  // Get Accelerometer values 1000 times
  while (c < 1000) {
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true);

    // Read registers of Accelerometer and divide raw values by 16384 for +-2g range
    AcX = (Wire.read() << 8 | Wire.read()) / ACCELEROMETER_SENSITIVITY; // 0x3B (ACCEL_XOUT_H) 0x3C (ACCEL_XOUT_L)
    AcY = (Wire.read() << 8 | Wire.read()) / ACCELEROMETER_SENSITIVITY; // 0x3D (ACCEL_YOUT_H) 0x3E (ACCEL_YOUT_L)
    AcZ = (Wire.read() << 8 | Wire.read()) / ACCELEROMETER_SENSITIVITY; // 0x3F (ACCEL_ZOUT_H) 0x40 (ACCEL_ZOUT_L)

    // Add up
    AcErrorX = AcErrorX + ((atan((AcY) / sqrt((AcX * AcX) + (AcZ * AcZ))) * 180 / M_PI)); // "sqrt(pow((AcX), 2) + pow((AcZ), 2))" also works fine
    AcErrorY = AcErrorY + ((atan(-1 * (AcX) / sqrt((AcY * AcY) + (AcZ * AcZ))) * 180 / M_PI));
    c++;
  }
  // Divide the sum by 1000 to get the error value
  AcErrorX = AcErrorX / 1000;
  AcErrorY = AcErrorY / 1000;
  AcErrorZ = AcErrorZ / 1000;
  c = 0;

  // Get Gyro values 1000 times
  while (c < 1000) {
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true);

    // Read registers of Gyroscope and divide raw values by 131 for 250 deg/s range
    GyroX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) 0x44 (GYRO_XOUT_L)
    GyroY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) 0x46 (GYRO_YOUT_L)
    GyroZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) 0x48 (GYRO_ZOUT_L)

    // Add up
    GyErrorX = GyErrorX + (GyroX / GYROSCOPE_SENSITIVITY);
    GyErrorY = GyErrorY + (GyroY / GYROSCOPE_SENSITIVITY);
    GyErrorZ = GyErrorZ + (GyroZ / GYROSCOPE_SENSITIVITY);
    c++;
  }

  // Divide the sum by 1000 to get error value
  GyErrorX = GyErrorX / 1000;
  GyErrorY = GyErrorY / 1000;
  GyErrorZ = GyErrorZ / 1000;

}

void beep() {
  digitalWrite(BUZZER, HIGH);
  delay(100);
  digitalWrite(BUZZER, LOW);
}

//  //conversion of accelerometer values into pitch and roll
//  pitch = atan(AcX / sqrt((AcY * AcY) + (AcZ * AcZ))); //pitch calculation
//  roll = atan(AcX / sqrt((AcX * AcX) + (AcZ * AcZ)));  //roll calculation
