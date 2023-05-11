

#include <Wire.h>
#include <math.h> // Math. Functions

int16_t AcX, AcY, AcZ;       // 16-bit Ints
float x, y, z; // Calibration variables

void setup() {
  Wire.begin();                // Initiate wire lib. and I2C
  Wire.beginTransmission(0x68); // Start transmission to I2C slave
  Wire.write(0x6B);            // Power Management Register (PWR_MGMT_1)
  Wire.write(0);               // Wake up IMU
  Wire.endTransmission(true);  // End transmission to I2C slave
  Serial.begin(115200);        // Baud Rate
}

void loop() {
   float pitch, roll, AcX, AcY, AcZ;
   Wire.beginTransmission(0x68);     // Begin transmission to I2C slave device
   Wire.write(0x3B);                 // Starting with register 0x3B (ACCEL_XOUT_H) (Default: Degrees per Sec)
   Wire.endTransmission(false);      // Restarts transmission to I2C slave device
   Wire.requestFrom(0x68, 14, true); // IMU address, request 14 registers in total, true
  
 // Read registers of Accelerometer and divide raw values by 16384 LSB/g for +-2g range 
   AcX = (Wire.read() << 8 | Wire.read()) / 16384; // 0x3B (ACCEL_XOUT_H) 0x3C (ACCEL_XOUT_L)
   AcY = (Wire.read() << 8 | Wire.read()) / 16384; // 0x3D (ACCEL_YOUT_H) 0x3E (ACCEL_YOUT_L)
   AcZ = (Wire.read() << 8 | Wire.read()) / 16384; // 0x3F (ACCEL_ZOUT_H) 0x40 (ACCEL_ZOUT_L)
  

  //conversion of accelerometer values into pitch and roll
  pitch = atan(AcX / sqrt((AcY * AcY) + (AcZ * AcZ))); //pitch calculation
  roll = atan(AcX / sqrt((AcX * AcX) + (AcZ * AcZ)));  //roll calculation

  Serial.print("Roll: "); Serial.println(roll);
  Serial.print("Pitch: "); Serial.println(pitch);


}
