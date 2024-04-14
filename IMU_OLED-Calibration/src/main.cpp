#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#define ACCELEROMETER_SENSITIVITY 16384.0 // +-2g = 16384 LSB/g
#define GYROSCOPE_SENSITIVITY 131.0       // 250 degrees/s = 131 LSB/degrees/s
// #define M_PI 3.14159265359                // Redefine PI
#define dt 0.01             // 10 ms Sample rate
#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 64    // OLED display height, in pixels
#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3c // 0x3D for 128x64, 0x3C for 128x32
#define BUZZER 11           // BUZZER Pin
#define TOGGLE_ANGLE 3
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);



int16_t AcX, AcY, AcZ, GyX, GyY, GyZ, GyroX, GyroY, GyroZ, Tmp; // 16-bit ints
int xAng, yAng, zAng;
float previousTime, currentTime, elapsedTime;
float gyroAngX, gyroAngY, gyroAngZ;
float AcErrorX, AcErrorY, AcErrorZ; // Calibration variables
float GyErrorX, GyErrorY, GyErrorZ;
const int MPU = 0x68;
int minVal = 265;
int maxVal = 402;
float r, y, p;

// int getIMU(float x, float y, float z);
void getIMU();
void getAngle();
void printBall();
void printAngles();

void setup()
{
  Serial.begin(115200);
  pinMode(BUZZER, OUTPUT);             // Set to OUTPUT for Buzzer Pin
  pinMode(TOGGLE_ANGLE, INPUT_PULLUP); // Toggle to display angular values
  Wire.begin();                        // Initiate wire lib. and I2C
  Wire.beginTransmission(0x68);        // Start transmission to I2C slave
  Wire.write(0x6B);                    // Power Management Register (PWR_MGMT_1)
  Wire.write(0);                       // Wake up IMU
  Wire.endTransmission(true);          // End transmission to I2C slave
  display.begin(SCREEN_ADDRESS, true); // Address 0x3C default
  // display.setContrast(5);              // Display Contrast
  display.cp437(true);
  display.display();
  display.clearDisplay(); // Clear the buffer.
  // initialize();
}

void loop()
{
  
  getAngle();
}
