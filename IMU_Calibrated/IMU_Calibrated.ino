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
#define M_PI 3.14159265359
#define dt 0.01                           // 10 ms Sample rate



int16_t AcX, AcY, AcZ, GyroX, GyroY, GyroZ, Tmp;       // 16-bit ints
double x, y, z, t, tx, tf, pitch, roll;
float previousTime, currentTime, elapsedTime;
float gyroAngX, gyroAngY, gyroAngZ;

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
  Wire.beginTransmission(MPU); // Start transmission to I2C slave
  Wire.write(0x6B);            // Power Management Register (PWR_MGMT_1)
  Wire.write(0);               // Wake up IMU
  Wire.endTransmission(true);  // End transmission to I2C slave
  Serial.begin(115200);        // Baud Rate
}

void loop()
{
  previousTime = currentTime;        //Previous time is stored before the actual time read
  currentTime = millis();            //Current time
  elapsedTime = (currentTime - previousTime) / 1000; //finding elapsed time and dividing by 1000 to get in seconds

  Wire.beginTransmission(MPU);      // Begin transmission to I2C slave device
  Wire.write(0x3B);                 // Starting with register 0x3B (ACCEL_XOUT_H) (Default: Degrees per Sec)
  Wire.endTransmission(false);      // restarts transmission to I2C slave device
  Wire.requestFrom(0x68, 14, true); // IMU address, request 14 registers in total, true

  //read register of Accelerometer data and divide raw value by 16384 for +-2g range
  AcX = (Wire.read() << 8 | Wire.read()) / ACCELEROMETER_SENSITIVITY; // 0x3B (ACCEL_XOUT_H) 0x3C (ACCEL_XOUT_L)
  AcY = (Wire.read() << 8 | Wire.read()) / ACCELEROMETER_SENSITIVITY; // 0x3D (ACCEL_YOUT_H) 0x3E (ACCEL_YOUT_L)
  AcZ = (Wire.read() << 8 | Wire.read()) / ACCELEROMETER_SENSITIVITY; // 0x3F (ACCEL_ZOUT_H) 0x40 (ACCEL_ZOUT_L)

  //read register of Gyroscope data and divide raw value by 131 for 250deg/s range
  GyroX = (Wire.read() << 8 | Wire.read()) / GYROSCOPE_SENSITIVITY; // 0x43 (GYRO_XOUT_H) 0x44 (GYRO_XOUT_L)
  GyroY = (Wire.read() << 8 | Wire.read()) / GYROSCOPE_SENSITIVITY; // 0x45 (GYRO_YOUT_H) 0x46 (GYRO_YOUT_L)
  GyroZ = (Wire.read() << 8 | Wire.read()) / GYROSCOPE_SENSITIVITY; // 0x47 (GYRO_ZOUT_H) 0x48 (GYRO_ZOUT_L)

  //read register of Temperature data
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) 0x42 (TEMP_OUT_L)

  gyroAngX = gyroAngX + GyroX * elapsedTime; // Converting into degrees (deg=deg/s*s)
  gyroAngY = gyroAngY + GyroY * elapsedTime;
  gyroAngZ = gyroAngZ + GyroZ * elapsedTime;

  //  int xAng = map(AcX, minVal, maxVal, -90, 90);
  //  int yAng = map(AcY, minVal, maxVal, -90, 90);
  //  int zAng = map(AcZ, minVal, maxVal, -90, 90);

  //  x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);//Angular Conversion rad to deg
  //  y = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
  //  z = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);

  //  double Fx = (0.96 * GyX) + (0.04 * AcX);
  //  double Fy = (0.96 * GyY) + (0.04 * AcY);
  //  double Fz = (0.96 * GyZ) + (0.04 * AcZ);

  tx = Tmp + tcal;         // Temperature Calculation
  t = tx / 340 + 36.53;    // Temperature in degrees C (from datasheet)
  //tf = (t * 9 / 5) + 32; // Celsius to Fahrenheit

  //  //conversion of accelerometer values into pitch and roll
  //  pitch = atan(AcX / sqrt((AcY * AcY) + (AcZ * AcZ))); //pitch calculation
  //  roll = atan(AcX / sqrt((AcX * AcX) + (AcZ * AcZ)));  //roll calculation
  //
  //  //converting radians into degrees
  //  pitch = pitch * (180.0 / 3.14);
  //  roll = roll * (180.0 / 3.14) ;
  //
  //  Serial.print("Pitch = "); Serial.print(pitch);
  //  Serial.print(" Roll = "); Serial.println(roll);
  //  Serial.print(" ");
  //  Serial.print(gyroAngX);
  //  Serial.print(" | ");
  //  Serial.print(abs(gyroAngY));
  //  Serial.print(" | ");
  //  Serial.println(gyroAngZ);
  //  // float newPos = 0 + prevPos;

  Serial.print("MAXX "); Serial.print(AcX + AcXcal);
  Serial.print(" MAXY "); Serial.print(AcY + AcYcal);
  Serial.print(" MAXZ "); Serial.print(AcZ + AcZcal);
  Serial.print(" MAXT: "); Serial.println(t);
  //Serial.print(" fahrenheit = "); Serial.println(tf);

}

//void beep(){
//  beeps when turned ON
//}
//
//void ComplementaryFilter(short accData[3], short gyrData[3], float *pitch, float *roll)
//{
//  float pitchAcc, rollAcc;
//
//  // Integrate the gyroscope data -> int(angularSpeed) = angle
//  *pitch += ((float)gyrData[0] / GYROSCOPE_SENSITIVITY) * dt;   // Angle around the X-axis
//  *roll -= ((float)gyrData[1] / GYROSCOPE_SENSITIVITY) * dt;    // Angle around the Y-axis
//
//  // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
//  int forceMagnitudeApprox = abs(accData[0]) + abs(accData[1]) + abs(accData[2]);
//
//  if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
//  {
//    // Turning around the X axis results in a vector on the Y-axis
//
//    pitchAcc = atan2f((float)accData[1], (float)accData[2]) * 180 / M_PI;
//    *pitch = *pitch * 0.98 + pitchAcc * 0.02;
//
//    // Turning around the Y axis results in a vector on the X-axis
//    rollAcc = atan2f((float)accData[0], (float)accData[2]) * 180 / M_PI;
//    *roll = *roll * 0.98 + rollAcc * 0.02;
//  }
//}


void calculate_error() {
  //Get Accelerometer values 1000 times
  while (c < 1000) {
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    
    // Add up
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt( AccX * AccX + (AccZ * AccZ))) * 180 / PI)); // " sqrt(pow((AccX), 2) + pow((AccZ), 2)) " also works fine
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(AccY * AccY) + (AccZ * AccZ))) * 180 / PI));
    c++;
  }
  //Divide the sum by 1000 to get the error value
  AccErrorX = AccErrorX / 1000;
  AccErrorY = AccErrorY / 1000;
  AccErrorZ = AccErrorZ / 1000;
  
  c = 0;
  // Get Gyro values 1000 times
  while (c < 1000) {
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true);
    
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    
    // Add up
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  
  // Divide the sum by 1000 to get error value
  GyroErrorX = GyroErrorX / 1000;
  GyroErrorY = GyroErrorY / 1000;
  GyroErrorZ = GyroErrorZ / 1000;
