/*

  **************************************************************************************************
  *                                   ESP32 PINOUT & SENSORS                                       *
  **************************************************************************************************

                               3V3 |*|                       |*| 5V
                               GND |*|                       |*| GND
                               IR  |*| GPIO15         GPIO13 |*| C.START
                               CUR |*| GPIO02         GPIO12 |*| C.STOP
                    AMB TEMP / HUM |*| GPIO04         GPIO14 |*| ALARM
                            L.TRIG |*| GPIO16         GPIO27 |*| P.COUNT
                            L.ECHO |*| GPIO17         GPIO26 |*|  -
                              BUZZ |*| GPIO05         GPIO25 |*| RELAY 
                                -  |*| GPIO18         GPIO33 |*| RELAY 
                                -  |*| GPIO19         GPIO32 |*| RELAY 
                               SDA |*| GPIO21         GPIO35 |*| RELAY 
                               RX  |*| GPIO03         GPIO34 |*| PT-100
                               TX  |*| GPIO01         GPIO39 |*| FLOW
                               SCL |*| GPIO22         GPIO36 |*| PRES
                                -  |*| GPIO23           EN   |*|  - 
                                
  ***************************************************************************************************
  

  # Standardised System Variables
  
  MAXCT     Cycle Start                  ON/OFF
  MAXCP     Cycle Stop                   ON/OFF
  MAXA      Alarm                        ON/OFF
  MAXN      Part Count                   N(Discrete Number)
  MAXAT     Ambient Temperature          C(Degree Celcius)
  MAXI      Current (AC/DC) upto 5A      A(Ampere)
  MAXT      Spindle Temperature          C(Degree Celcius)
  MAXL      Coolant Level                CM(Centimetre)
  MAXP      Ambient Pressure             Bar (= 100,000kPA)
  MAXF      Coolant Flow                 CC/s(Cubic CM per Second)
  MAXH      Humidity                     Percentage
  MAXR      Spindle RPM                  RPM(Rotations per Minute)
  MAXVX     Motor X Vibration            mm(millimetres)
  MAXVY     Motor Y Vibration            mm(millimetres)
  MAXVZ     Motor Z Vibration            mm(millimetres)
  
*/


#include <math.h>               // Math Functions
#include <ACS712.h>             // ACS712 Functions                        
#include <dht.h>                // Ambient Temperature/Humidity Functions     
#include <Wire.h>               // For communication with I2C devices
#include <LiquidCrystal_I2C.h>

#define CUR 2
#define TRIG 16
#define ECHO 17
#define DHTPIN 4
#define BUZZ 11
#define dt  2000

float MAXA, MAXAT, MAXT, MAXL, MAXP, MAXF, MAXR, MAXI, MAXN, MAXH, MAXX, MAXY, MAXZ, THLD1, THLD2;
long DUR;

dht DHT;
ACS712 sensor(ACS712_05B, 2);
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  pinMode(CUR, INPUT);                    // ADC Pin for ACS712
  pinMode(TRIG, OUTPUT);                  // TRIGGER Pin of HC-SR05 to Write
  pinMode(ECHO, INPUT);                   // ECHO Pin of HC-SR05 to Read
  pinMode(BUZZ, OUTPUT);                  // Buzzer Pin for Beeps
  pinMode(DHTPIN, INPUT);                 //DHT11 Pin as Input
  Serial.begin(9600);                     // Compensated Baud to 9600 for RS-232
  lcd.init();                             // LCD Initialise
  lcd.backlight();                        // LCD Backlight ON
  initial();                              // Device Start Configuration
  Wire.begin();                           // Initiate wire lib. and I2C
  Wire.beginTransmission(0x68);           // Start transmission to 0x68(MPU)
  Wire.write(0x6B);                       // Power Management Register (PWR_MGMT_1)
  Wire.write(0);                          // Wake up IMU
  Wire.endTransmission(true);             // End transmission to I2C slave
  ACS_Calibrate();                        // Calibrate ACS
}


void loop() {
  LCDPRINT(F_MAXT, F_MAXI);
}
