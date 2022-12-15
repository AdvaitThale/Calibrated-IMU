/*
  Author: Advait Thale

  *******************************************************************
                         IMU - ESP32 Connect
  *******************************************************************

  Ths is a test code for reading all values of IMUs sensors. Here, as IMU
  MPU6050 is used which is 6 axis Accelerometer, Gyroscope and Temprature sensor.
  Inertial Measurement Unit are calibrated according to PID(Proportional-Integral-
  Derivative) control system. Refer Register Map and datasheet for furthur details
  on MPU6050 IMU.

  Pinout of IMU MPU6050:
  ---------------------------------
  |VCC|GND|SCL|SDA|XDA|XCL|AD0|INT|
  ---------------------------------
*/


#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>            // Library for I2C comm
#include <math.h>            // To include math functions
#include <Adafruit_GFX.h>    // Adafruit OLED graphics lib
#include <Adafruit_SH110X.h> // Adafruit OLED lib

#define i2c_Address 0x3c    // Initialize with the I2C address 0x3C 
#define SCREEN_WIDTH 128    // Display Width Px
#define SCREEN_HEIGHT 64    // Display Height Px
#define OLED_RESET -1       // QT-PY / XIAO

Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/* SSID & Password*/
const char* ssid = "Advait's ESP";          // Enter SSID
const char* password = "password";  //Enter Password

/* IP Address details */
IPAddress local_ip(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

WebServer server(80);

uint8_t IMU1pin = 18;
bool IMU1status = LOW;

uint8_t IMU2pin = 19;
bool IMU2status = LOW;

const int MPU = 0x68;                               //I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;          //16-bit integers
int AcXcal, AcYcal, AcZcal, GyXcal, GyYcal, GyZcal, tcal; //calibration variables
double t, tx, tf, pitch, roll;

void setup() {
  pinMode(IMU1pin, OUTPUT);
  pinMode(IMU2pin, OUTPUT);
  Wire.begin();                     // initiate wire library and I2C
  Wire.beginTransmission(MPU);      // begin transmission to I2C slave device
  Wire.write(0x6B);                 // PWR_MGMT_1 register
  Wire.write(0);                    // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);       // ends transmission to I2C slave device
  display.begin(i2c_Address, true); // Address 0x3C default
  display.setContrast (5);          // dim display
  display.cp437(true);
  display.display();
  display.clearDisplay();           // Clear the buffer.
  Serial.begin(115200);

  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  delay(100);

  server.on("/", handle_OnConnect);
  server.on("/imu1on", handle_imu1on);
  server.on("/imu1off", handle_imu1off);
  server.on("/imu2on", handle_imu2on);
  server.on("/imu2off", handle_imu2off);
  server.onNotFound(handle_NotFound);

  server.begin();
  Serial.println("HTTP server started");
}
void loop() {
  Wire.beginTransmission(MPU); //begin transmission to I2C slave device
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false); //restarts transmission to I2C slave device
  Wire.requestFrom(MPU, 14, true); //request 14 registers in total

  GyXcal = 480; // Gyro Error
  GyYcal = 170;
  GyZcal = 210;

  AcXcal = -950; // Acceleration Error
  AcYcal = -300;
  AcZcal = 0;

  tcal = -1600; // Temperature Error

  //read register of accelerometer data
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) 0x40 (ACCEL_ZOUT_L)

  //read register of temperature data
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) 0x42 (TEMP_OUT_L)

  //read register of gyroscope data
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) 0x48 (GYRO_ZOUT_L)
  
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

  Serial.print("MAXX "); Serial.print(AcX + AcXcal);
  Serial.print(" MAXY "); Serial.print(AcY + AcYcal);
  Serial.print(" MAXZ "); Serial.print(AcZ + AcZcal);
  
  Serial.print(" MAXT: "); Serial.println(t);
  //Serial.print(" fahrenheit = "); Serial.println(tf);

//  Serial.print("Gyroscope: ");
//  Serial.print("X = "); Serial.print(GyX + GyXcal);
//  Serial.print(" Y = "); Serial.print(GyY + GyYcal);
//  Serial.print(" Z = "); Serial.println(GyZ + GyZcal);

  server.handleClient();
  if (IMU1status)
  {
    digitalWrite(IMU1pin, HIGH);
  }
  else
  {
    digitalWrite(IMU1pin, LOW);
  }

  server.handleClient();
  if (IMU2status)
  {
    digitalWrite(IMU2pin, HIGH);
  }
  else
  {
    digitalWrite(IMU2pin, LOW);
  }
}

void handle_OnConnect() {
  IMU1status = LOW;
  IMU2status = LOW;
  Serial.println("GPIO18 Status: OFF | GPIO19 Status: OFF");
  server.send(200, "text/html", SendHTML(IMU1status, IMU2status));
}

void handle_imu1on() {
  IMU1status = HIGH;
  Serial.println("GPIO18 Status: ON");
  server.send(200, "text/html", SendHTML(true, IMU1status));
}

void handle_imu1off() {
  IMU1status = LOW;
  Serial.println("GPIO18 Status: OFF");
  server.send(200, "text/html", SendHTML(false, IMU1status));
}

void handle_imu2on() {
  IMU2status = HIGH;
  Serial.println("GPIO19 Status: ON");
  server.send(200, "text/html", SendHTML(false, IMU2status));
}

void handle_imu2off() {
  IMU2status = LOW;
  Serial.println("GPIO19 Status: OFF");
  server.send(200, "text/html", SendHTML(true, IMU2status));
}

void handle_NotFound() {
  server.send(404, "text/plain", "Not found");
}

String SendHTML(uint8_t imu1stat, uint8_t imu2stat) {
  String ptr = "<!DOCTYPE html> <html>\n";
  ptr += "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  ptr += "<meta content=\"Advait Thale\" name=\"author\"><meta content=\"20\" http-equiv=\"refresh\">\n";
  ptr += "<title>IMU Control</title>\n";
  ptr += "<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";
  ptr += "body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n";
  ptr += ".button {display: block;width: 80px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n";
  ptr += ".button-on {background-color: #009933;}\n";
  ptr += ".button-on:active {background-color: #2980b9;}\n";
  ptr += ".button-off {background-color: #ff0000;}\n";
  ptr += ".button-off:active {background-color: #34495e;}\n";
  ptr += "p {font-size: 20px;color: #888;margin-bottom: 10px;}\n";
  ptr += "#box {border-radius: 20px;background: #d9d9d9;padding: 20px;width: 120px;height: 60px;}";
  ptr += "footer {text-align: center;text-decoration: none;position: fixed;left: 0;bottom: 0;width: 100%;height: 3%;padding: 2px;background-color: #00004d;color: #f2f2f2;}\n";
  ptr += ".column {float: left; width: 33.33 %; padding: 15px;}\n";
  ptr += ".row: after {content: ""; display: table; clear: both;}\n";
  ptr += "</style>\n";
  ptr += "</head>\n";

  ptr += "<body>\n";
  ptr += "<h1>IMU COMM.</h1>\n";
  ptr += "<h3>DASHBOARD</h3>\n";

  if (t > 50) {
ptr += "<p>TEMP. OVERLOAD<svg width="170" height="15"><rect x="15" y="5" rx="7" ry="7" width="20" height="10" style="fill: #ff8800"/></svg> </p>";
  }
  else(t <= 50) {
ptr += "<p>TEMP. OVERLOAD<svg width="170" height="15"><rect x="15" y="5" rx="7" ry="7" width="20" height="10" style="fill: #009933"/></svg> </p>";
  }

  ptr += "<div class=\"row\">\n";
  if (imu1stat)
  {
    ptr += "<div class=\"column\"> <nobr><p>IMU-1:<div style=\"color:#009933\"> ONLINE</div></p><a class=\"button button-off\" href=\"/imu1off\">OFF</a></nobr><p id=\"box\">Yaw: </br>Pitch: </br>Roll: </p> </div>\n";
  }
  else
  {
    ptr += "<div class=\"column\"> <nobr><p>IMU-1:<div style=\"color:#e60000\"> OFFLINE</div></p><a class=\"button button-on\" href=\"/imu1on\">ON</a></nobr><p id=\"box\">Yaw: </br>Pitch: </br>Roll: </p> </div>\n";
  }
  if (imu2stat)
  {
    ptr += "<div class=\"column\"> <nobr><p>IMU-2:<div style=\"color:#009933\"> ONLINE</div></p><a class=\"button button-off\" href=\"/imu2off\">OFF</a></nobr><p id=\"box\">Yaw: </br>Pitch: </br>Roll: </p></div>\n";
  }
  else
  {
    ptr += "<div class=\"column\"> <nobr><p>IMU-2: <div style=\"color:#e60000\"> OFFLINE</div></p><a class=\"button button-on\" href=\"/imu2on\">ON</a></nobr><p id=\"box\">Yaw: </br>Pitch: </br>Roll: </p></div>\n";
  }
  ptr += "</div>";
  ptr += "<footer>INERTIAL MEASUREMENT UNIT CONTROL DASHBOARD Author: Advait Thale</footer>";
  ptr += "</body>\n";
  ptr += "</html>\n";
  return ptr;
}



//
// int16_t GyroX, GyroY, GyroZ;
// float previousTime, currentTime, elapsedTime;
// float gyroAngX, gyroAngY, gyroAngZ;
//
// 
// void setup()
// {
//  Wire.begin();                      
//  Wire.beginTransmission(MPU);      
//  Wire.write(0x6B);                  
//  Wire.write(0);                  
//  Wire.endTransmission(true);
//  Serial.begin(115200);
// }
// 
// void loop()
// {
// previousTime = currentTime;        //Previous time is stored before the actual time read
// currentTime = millis();            //Current time 
// elapsedTime = (currentTime-previousTime)/1000; //finding elapsed time and dividing by 1000 to get in seconds
//  
// Wire.beginTransmission(MPU);
// Wire.write(0x43); //Gyro Measurement Register
// Wire.endTransmission(false);
// Wire.requestFrom(MPU, 6, true); //6+6+2 registers
// 
// GyroX = (Wire.read()<<8|Wire.read()) / 131.0; // For 250deg/s range divide raw value by 131.0
// GyroY = (Wire.read()<<8|Wire.read()) / 131.0;
// GyroZ = (Wire.read()<<8|Wire.read()) / 131.0;
//  
//  
//  gyroAngX = gyroAngX + GyroX * elapsedTime; //Converting into degrees (deg=deg/s*s)
//  gyroAngY = gyroAngY + GyroY * elapsedTime;
//  gyroAngZ = gyroAngZ + GyroZ * elapsedTime;
//  
//  Serial.print(" ");
//  Serial.print(gyroAngX);
//  Serial.print(" | ");
//  Serial.print(abs(gyroAngY));
//  Serial.print(" | ");
//  Serial.println(gyroAngZ);
//  // float newPos = 0 + prevPos; 
// }
//
// int minVal=265;
//int maxVal=402;
// 
//double x;
//double y;
//double z;
// 
//void setup()
//{
//  Wire.begin();
//  Wire.beginTransmission(MPU);
//  Wire.write(0x6B); //configure power mode and clock mode
//  Wire.write(0);
//  Wire.endTransmission(true);
//  Serial.begin(115200);
//}
//void loop()
//{
//  Wire.beginTransmission(MPU);
//  Wire.write(0x3B); //Accelerometer Measurement Register
//  Wire.endTransmission(false);
//  Wire.requestFrom(MPU,14,true);
//  AcX = Wire.read() << 8 | Wire.read();//8 bit shift
//  AcY = Wire.read() << 8 | Wire.read();
//  AcZ = Wire.read() << 8 | Wire.read();
//
//  int xAng = map(AcX,minVal,maxVal,-90,90);
//  int yAng = map(AcY,minVal,maxVal,-90,90);
//  int zAng = map(AcZ,minVal,maxVal,-90,90);
// 
//  x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);//Angular Conversion rad to deg
//  y = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
//  z = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);
//
//  Serial.print("Roll= ");Serial.print(x);
//  Serial.print(" | Pitch= ");Serial.print(y);
//  Serial.print(" | Yaw= ");Serial.println(z);
//
//  delay(100);
//}
//
//
//
///*
//  Fx = (0.96 * GyX) + (0.04 * AcX);
//  Fy = (0.96 * GyY) + (0.04 * AcY);
//  Fz = (0.96 * GyZ) + (0.04 * AcZ);
//*/
//
//#define ACCELEROMETER_SENSITIVITY 8192.0
//#define GYROSCOPE_SENSITIVITY 65.536
//#define M_PI 3.14159265359
//#define dt 0.01             // 10 ms sample rate!    
// 
//void ComplementaryFilter(short accData[3], short gyrData[3], float *pitch, float *roll)
//{
//    float pitchAcc, rollAcc;               
// 
//    // Integrate the gyroscope data -> int(angularSpeed) = angle
//    *pitch += ((float)gyrData[0] / GYROSCOPE_SENSITIVITY) * dt; // Angle around the X-axis
//    *roll -= ((float)gyrData[1] / GYROSCOPE_SENSITIVITY) * dt;    // Angle around the Y-axis
//
//    // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
//    int forceMagnitudeApprox = abs(accData[0]) + abs(accData[1]) + abs(accData[2]);
//    if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
//    {
//  // Turning around the X axis results in a vector on the Y-axis
//        pitchAcc = atan2f((float)accData[1], (float)accData[2]) * 180 / M_PI;
//        *pitch = *pitch * 0.98 + pitchAcc * 0.02;
// 
//  // Turning around the Y axis results in a vector on the X-axis
//        rollAcc = atan2f((float)accData[0], (float)accData[2]) * 180 / M_PI;
//        *roll = *roll * 0.98 + rollAcc * 0.02;
//    }
//}
