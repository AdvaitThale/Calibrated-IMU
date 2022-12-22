/*
  Author: Advait Thale
  *******************************************************************
                            IMU Connect
  *******************************************************************
  The data is visualized on web server which is runned by ESP32 itself
  on browser via WiFi. Type IP Address 192.168.1.1 on browser after
  connecting to ESP32.

*/
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <math.h> // Math. Functions

int16_t AcX, AcY, AcZ, Tmp;       // 16-bit Ints
float MAXX, MAXY, MAXZ, MAXT; // Calibration variables
double t, tx, tf;

int AcXcal = -950; // Acceleration Error
int AcYcal = -300;
int AcZcal = 0;
int tcal = -1600; // Temperature correction

/*Put your SSID & Password*/
const char* ssid = "Cunt";          // Enter SSID here
const char* password = "password";  //Enter Password here

/* Put IP Address details */
IPAddress local_ip(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
WebServer server(80);


void setup() {
  Wire.begin();                // Initiate wire lib. and I2C
  Wire.beginTransmission(0x68); // Start transmission to I2C slave
  Wire.write(0x6B);            // Power Management Register (PWR_MGMT_1)
  Wire.write(0);               // Wake up IMU
  Wire.endTransmission(true);  // End transmission to I2C slave
  Serial.begin(115200);        // Baud Rate
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password); // Set it to NULL, if you don’t want to set a password.
  //  WiFi.softAP(const char* ssid, const char* password, int channel, int ssid_hidden, int max_connection)
  //  ssid: name for the access point – maximum of 63 characters;
  //  password: minimum of 8 characters; set to NULL if you want the access point to be open;
  //  channel: Wi - Fi channel number (1 - 13)
  //  ssid_hidden: (0 = broadcast SSID, 1 = hide SSID)
  //  max_connection: maximum simultaneous connected clients (1 - 4)
  WiFi.softAPConfig(local_ip, gateway, subnet);
  delay(100);

  server.on("/", handle_OnConnect);
  server.onNotFound(handle_NotFound);
  server.begin();

}

void loop() {
  server.handleClient();
  Wire.beginTransmission(0x68);      // begin transmission to I2C slave device
  Wire.write(0x3B);                 // starting with register 0x3B (ACCEL_XOUT_H) (Default: Degrees per Sec)
  Wire.endTransmission(false);      // restarts transmission to I2C slave device
  Wire.requestFrom(0x68, 14, true);  // request 14 registers in total
  
  // Read register of Accelerometer data
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) 0x40 (ACCEL_ZOUT_L)

  // Read register of Temperature data
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) 0x42 (TEMP_OUT_L)

  tx = Tmp + tcal;         // Temperature Calculation
  t = tx / 340 + 36.53;    // Temperature in degrees C (from datasheet)
  //tf = (t * 9 / 5) + 32; // Celsius to Fahrenheit
  Serial.print("MAXX "); Serial.print(AcX + AcXcal);
  Serial.print(" MAXY "); Serial.print(AcY + AcYcal);
  Serial.print(" MAXZ "); Serial.print(AcZ + AcZcal);
  Serial.print(" MAXT: "); Serial.println(t);
}

void getIMUData() {
 
}
void calculate_error() {
  // Read accelerometer values 1000 times
  while (c < 1000) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt( AccX * AccX + (AccZ * AccZ))) * 180 / PI)); // " sqrt(pow((AccX), 2) + pow((AccZ), 2)) " also works fine
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(AccY * AccY) + (AccZ * AccZ))) * 180 / PI));
    c++;
  }
  
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 1000;
  AccErrorY = AccErrorY / 1000;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  
void handle_OnConnect() {
  loop();
  MAXX = AcX + AcXcal;
  MAXY = AcY + AcYcal;
  MAXZ = AcZ + AcZcal;
  MAXT = t; 
  server.send(200, "text/html", SendHTML(MAXX, MAXY, MAXZ, MAXT));
}

void handle_NotFound() {
  server.send(404, "text/plain", "Oops, Page Not Found!!");
}

String SendHTML(float MAXX, float MAXY, float MAXZ, float MAXT) {
  String ptr = "<!DOCTYPE html> <html>\n";
  ptr += "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  ptr += "<meta content=\"Advait Thale\" name=\"author\"><meta content=\"1\" http-equiv=\"refresh\">\n";
  ptr += "<title>IMU Dashboard</title>\n";
  ptr += "<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";
  ptr += "body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;}\n";
  ptr += "p {font-size: 24px;color: #444444;margin-bottom: 10px;}\n";
  ptr += "footer {text-align: center;text-decoration: none;position: fixed;left: 0;bottom: 0;width: 100%;height: 3%;padding: 2px;background-color: #00004d;color: #f2f2f2;}\n";
  ptr += "</style>\n";
  ptr += "</head>\n";
  ptr += "<body>\n";
  ptr += "<div id=\"webpage\">\n";
  ptr += "<h1>IMU Monitor</h1>\n";
   if (t >= 50) {
    ptr += "<p>TEMP. OVERLOAD<svg width=\"170\" height=\"15\"><rect x=\"15\" y=\"5\" rx=\"7\" ry=\"7\" width=\"20\" height=\"10\" style=\"fill: #e60000\"/></svg> </p>";
  }
  else if(t > 25 && t < 50) {
    ptr += "<p>TEMP. OVERLOAD<svg width=\"170\" height=\"15\"><rect x=\"15\" y=\"5\" rx=\"7\" ry=\"7\" width=\"20\" height=\"10\" style=\"fill: #ff8800\"/></svg> </p>";
  }
  else {
    ptr += "<p>TEMP. OVERLOAD<svg width=\"170\" height=\"15\"><rect x=\"15\" y=\"5\" rx=\"7\" ry=\"7\" width=\"20\" height=\"10\" style=\"fill: #009933\"/></svg> </p>";
  }
  
  ptr += "<p> MAXX: ";
  ptr += (float)MAXX;
  ptr += "</p>";
  ptr += "<p> MAXY: ";
  ptr += (float)MAXY;
  ptr += "</p>";
  ptr += "<p> MAXZ: ";
  ptr += (float)MAXZ;
  ptr += "</p>";
  ptr += "<p> MAXT: ";
  ptr += (float)MAXT;
  ptr += "</p>";
  ptr += "</div>\n";
  ptr += "<footer>INERTIAL MEASUREMENT UNIT DASHBOARD -> @Author: Advait Thale</footer>";
  ptr += "</body>\n";
  ptr += "</html>\n";
  return ptr;
}
