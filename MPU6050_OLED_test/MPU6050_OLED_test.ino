/*
  Author: Advait Thale

  *******************************************************************
  *                        MPU6050 OLED Test                        *    
  *******************************************************************

  Ths is a test code for displaying Roll, Pitch and Yaw of system via IMU. Here, as IMU
  MPU6050 is used which is 6 axis Accelerometer, Gyroscope and Temprature sensor.
  Refer Register Map and datasheet for furthur details on MPU6050 IMU.
  
  Pinout of IMU MPU6050:
  ---------------------------------
  |VCC|GND|SCL|SDA|XDA|XCL|AD0|INT| 
  ---------------------------------

  Pinout of 128x64 OLED display:
  -----------------
  |VCC|GND|SCL|SDA|
  -----------------
  
*/
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#define i2c_Address 0x3c    // Initialize with the I2C address 0x3C 
#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 64    // OLED display height, in pixels
#define OLED_RESET -1       // QT-PY / XIAO
#define vol_rd 1           // Defining analogpin for reading battery voltage

Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const int MPU = 0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int minVal=265;
int maxVal=402;
float r, p, y;


void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);                 //configure power mode and clock mode
  Wire.write(0);
  Wire.endTransmission(true);
  display.begin(i2c_Address, true); // Address 0x3C default
  display.setContrast (5);          // dim display
  display.cp437(true);
  display.display();
  display.clearDisplay();           // Clear the buffer.
  pinMode(vol_rd, INPUT);
  Serial.begin(115200);
}

void loop() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); //Accelerometer Measurement Register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,6,true);//14
  AcX = Wire.read() << 8 | Wire.read();//8 bit shift
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();

  int xAng = map(AcX,minVal,maxVal,-90,90);
  int yAng = map(AcY,minVal,maxVal,-90,90);
  int zAng = map(AcZ,minVal,maxVal,-90,90);

  r = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
  p = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
  y = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);

  Serial.print("Roll= ");Serial.print(r);
  Serial.print(" | Pitch= ");Serial.print(p);
  Serial.print(" | Yaw= ");Serial.println(y);

  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(80, 10);
  display.print("BAT= ");
  display.setCursor(100, 10);
  display.print(analogRead(vol_rd));
  display.setCursor(0, 25);
  display.print("Roll: ");
  display.setCursor(32, 25);
  display.print(r);
  display.setCursor(0, 40);
  display.print("Pitch: ");
  display.setCursor(40, 40);
  display.print(p);
  display.setCursor(0, 55);
  display.print("Yaw: ");
  display.setCursor(30, 55);
  display.print(y);
 }
