#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#define ACCELEROMETER_SENSITIVITY 16384.0 // +-2g = 16384 LSB/g
#define GYROSCOPE_SENSITIVITY 131.0       // 250 degrees/s = 131 LSB/degrees/s
#define M_PI 3.14159265359                // Redefine PI
#define dt 0.01                           // 10 ms Sample rate
#define SCREEN_WIDTH 128                  // OLED display width, in pixels
#define SCREEN_HEIGHT 64                  // OLED display height, in pixels
#define OLED_RESET -1                     // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3c               // 0x3D for 128x64, 0x3C for 128x32
#define BUZZER 11                         // BUZZER Pin
#define TOGGLE_ANGLE 3                    // Toggle to display angular values
#define CENTRE_LED 12                     // LED Pin
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define BITMAP_BALL_HEIGHT 16
#define BITMAP_BALL_WIDTH 16
static const unsigned char PROGMEM ball_12pix_bmp[] =
    {
        B00001111, B11110000,
        B00111111, B11111100,
        B01111111, B11111110,
        B01111111, B11111110,
        B01111111, B11111110,
        B11111110, B11111111,
        B11111110, B11111111,
        B11111000, B00111111,
        B11111110, B11111111,
        B11111110, B11111111,
        B01111111, B11111110,
        B01111111, B11111110,
        B01111111, B11111110,
        B00111111, B11111100,
        B00001111, B11110000};

int16_t AcX, AcY, AcZ, GyX, GyY, GyZ, GyroX, GyroY, GyroZ, Tmp; // 16-bit ints
int xAng, yAng, zAng;
float previousTime, currentTime, elapsedTime;
float gyroAngX, gyroAngY, gyroAngZ;
float AcErrorX, AcErrorY, AcErrorZ; // Calibration variables  // AcErrorX = -1, AcErrorY = 0. , AcErrorZ = -1/360
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
  pinMode(CENTRE_LED, OUTPUT);         // Set to OUTPUT for LED Pin
  Wire.begin();                        // Initiate wire lib. and I2C
  Wire.beginTransmission(0x68);        // Start transmission to I2C slave
  Wire.write(0x6B);                    // Power Management Register (PWR_MGMT_1)
  Wire.write(0);                       // Wake up IMU
  Wire.endTransmission(true);          // End transmission to I2C slave
  display.begin(SCREEN_ADDRESS, true); // Address 0x3C default
  display.setContrast(5);              // Display Contrast
  display.cp437(true);                 // Using IBM standard charset CP437
  display.display();                   // Display = True
  display.clearDisplay();              // Clear the buffer.
  // display.setCursor(64, 32);
  // display.print("AUGHH");
  // initialize();
}

void loop()
{
  if (digitalRead(TOGGLE_ANGLE) == LOW) // Angles displayed after keypressed
  {
    printAngles();
  }
  else
  {
    printBall();
  }
  // printAngles();
  // getAngle();
}

void printBall()
{
  getAngle();
  int roll = map(yAng, -90, 90, 0, 128);
  int pitch = map(xAng, -90, 90, 0, 64);
  display.drawCircle(64, 32, 12, SH110X_WHITE);
  // display.drawLine(0, 32, 128, 32, SH110X_WHITE);
  // display.drawLine(64, 0, 64, 64, SH110X_WHITE);
  if (pitch == 32 && roll == 64)
  {
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(62, 29);                    // Adjusted Centre
    display.print("+");                           // Absolute Centre
    display.drawCircle(64, 32, 12, SH110X_WHITE); // Centre Reference Circle
    digitalWrite(CENTRE_LED, HIGH);               // LED ON
    digitalWrite(LED_BUILTIN, HIGH);              // LED ON
  }
  else
  {
    display.fillCircle(roll, pitch, 12, SH110X_WHITE); // ((270) -90 <= Roll >= 90, 90 <= Pitch >= -90 (270))
    digitalWrite(CENTRE_LED, LOW);                     // LED OFF
    digitalWrite(LED_BUILTIN, LOW);                    // LED OFF
  }
  // display.drawLine(0, 32, 128, 32, SH110X_WHITE);
  // display.drawLine(64, 0, 64, 64, SH110X_WHITE); // x, y, x1, y1
  // display.drawBitmap(roll, pitch, ball_12pix_bmp, BITMAP_BALL_HEIGHT, BITMAP_BALL_WIDTH, SH110X_WHITE);
  display.display();
  display.clearDisplay();
  // display.clearWriteError();
}

void printAngles()
{
  getAngle();
  display.drawRect(0, 0, 128, 64, SH110X_WHITE); // Table View Start
  display.drawLine(0, 17, 128, 17, SH110X_WHITE);
  display.drawLine(64, 0, 64, 64, SH110X_WHITE); // Table View End
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(7, 5);
  display.print("Parameter");
  display.setCursor(70, 5);
  display.print("Angles");
  display.setCursor(15, 22);
  display.print("Roll ");
  display.setCursor(75, 22);
  display.print(r);
  display.setCursor(15, 37);
  display.print("Pitch ");
  display.setCursor(75, 37);
  display.print(p);
  display.setCursor(15, 52);
  display.print("Yaw ");
  display.setCursor(75, 52);
  display.print(y);
  display.display();
  display.clearDisplay();
  // display.drawLine(0, 0, 128, 64, SH110X_WHITE);
  // display.clearWriteError();
}

void getAngle()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Accelerometer Measurement Register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // 14

  AcX = Wire.read() << 8 | Wire.read(); // 8 bit shift
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();

  xAng = map(AcX, -ACCELEROMETER_SENSITIVITY, ACCELEROMETER_SENSITIVITY, -90, 90);
  yAng = map(AcY, -ACCELEROMETER_SENSITIVITY, ACCELEROMETER_SENSITIVITY, -90, 90);
  zAng = map(AcZ, -ACCELEROMETER_SENSITIVITY, ACCELEROMETER_SENSITIVITY, -90, 90);

  int xAngle = map(AcX, minVal, maxVal, -90, 90);
  int yAngle = map(AcY, minVal, maxVal, -90, 90);
  int zAngle = map(AcZ, minVal, maxVal, -90, 90);

  r = RAD_TO_DEG * (atan2(-yAngle, -zAngle) + PI); // Angular Conversion rad to deg
  p = RAD_TO_DEG * (atan2(-xAngle, -zAngle) + PI);
  y = RAD_TO_DEG * (atan2(-yAngle, -xAngle) + PI);

  Serial.print("Roll= ");
  Serial.print(r);
  Serial.print(" | Pitch= ");
  Serial.print(p);
  Serial.print(" | Yaw= ");
  Serial.println(y);
}

void getIMU()
{
  previousTime = currentTime;                        // Previous time is stored before the actual time read
  currentTime = millis();                            // Current time
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

  //  x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI); //Angular Conversion rad to deg
  //  y = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
  //  z = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);

  // Complementary Filter to add up Gyroscope values and Accelerometer
  float x = (0.96 * GyroX) + (0.04 * AcX);
  float y = (0.96 * GyroY) + (0.04 * AcY);
  float z = (0.96 * GyroZ) + (0.04 * AcZ);

  Serial.print("X: ");
  Serial.print(x);
  Serial.print(" | ");
  Serial.print("Y: ");
  Serial.print(y);
  Serial.print(" | ");
  Serial.print("Z: ");
  Serial.println(z);
  // Serial.print("");
  // return x, y, z;
}

void initialize()
{
  Serial.println("Gathering IMU Data...");
  digitalWrite(BUZZER, HIGH);
  delay(85);
  digitalWrite(BUZZER, LOW);
  tone(BUZZER, 2093);
  delay(200);
  noTone(BUZZER);
  digitalWrite(BUZZER, HIGH);
  delay(100);
  digitalWrite(BUZZER, LOW);
}
