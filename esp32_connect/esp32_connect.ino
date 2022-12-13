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
#include <Wire.h>     // Library for I2C comm
#include <math.h>     // To include math functions

/* SSID & Password*/
const char* ssid = "Cunt";          // Enter SSID
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
  Wire.begin();                 //initiate wire library and I2C
  Wire.beginTransmission(MPU);  //begin transmission to I2C slave device
  Wire.write(0x6B);             // PWR_MGMT_1 register
  Wire.write(0);                // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);   //ends transmission to I2C slave device
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

 //Acceleration data correction
  AcXcal = -950;
  AcYcal = -300;
  AcZcal = 0;

  //Temperature correction
  tcal = -1600;

  //Gyro correction
  GyXcal = 480;
  GyYcal = 170;
  GyZcal = 210;
  
  //read accelerometer data
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) 0x40 (ACCEL_ZOUT_L)

  //read temperature data
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) 0x42 (TEMP_OUT_L)

  //read gyroscope data
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) 0x48 (GYRO_ZOUT_L)

  //temperature calculation
  tx = Tmp + tcal;
  t = tx / 340 + 36.53; //equation for temperature in degrees C from datasheet
  tf = (t * 9 / 5) + 32; //fahrenheit

   //printing values to serial port
   // Serial.print("Angle: ");
    Serial.print("Pitch = "); Serial.print(pitch);
    Serial.print(" Roll = "); Serial.println(roll);
  
    Serial.print("Accelerometer: ");
    Serial.print("X = "); Serial.print(AcX + AcXcal);
    Serial.print(" Y = "); Serial.print(AcY + AcYcal);
    Serial.print(" Z = "); Serial.println(AcZ + AcZcal); 

    /*Serial.print("Temperature in celsius = "); Serial.print(t);  
    Serial.print(" fahrenheit = "); Serial.println(tf);  */
  
    Serial.print("Gyroscope: ");
    Serial.print("X = "); Serial.print(GyX + GyXcal);
    Serial.print(" Y = "); Serial.print(GyY + GyYcal);
    Serial.print(" Z = "); Serial.println(GyZ + GyZcal);

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
  ptr += "<h1>APOLLO COMMS</h1>\n";
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
