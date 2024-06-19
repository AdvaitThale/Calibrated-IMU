#include <Arduino.h>

#include <SPI.h>      /* to handle the communication interface with the modem*/
#include <nRF24L01.h> /* to handle this particular modem driver*/
#include <RF24.h>     /* the library which helps us to control the radio modem*/
#define LED 8         /* Connect LED anode to D5(PWM pin) */

// const uint64_t Address = 0xE8E8F0F0E1LL;

// RF24 radio(9, 10); // ( CE , CSN )

// struct MyData
// {
//   byte pot_value;
// };

// MyData data;

// void resetData()
// {
//   data.pot_value = 0;
// }

// void setup()
// {
//   Serial.begin(9600);
//   pinMode(LED, OUTPUT);
//   resetData();
//   radio.begin();
//   radio.setAutoAck(false);
//   radio.setDataRate(RF24_250KBPS);
//   radio.openReadingPipe(1, Address); /* Sets the address of receiver from which program will receive the data*/
//   radio.startListening();
// }

// unsigned long lastRecvTime = 0;

// void recvData()
// {
//   while (radio.available())
//   {
//     radio.read(&data, sizeof(MyData));
//     lastRecvTime = millis(); // here we receive the data
//   }
// }

// void loop()
// {
//   recvData();
//   unsigned long now = millis();

//   if (now - lastRecvTime > 1000)
//   {
//     Serial.println("<!--NO SIGNAL--!>"); // <!--NO SIGNAL--!> if not receiving valid data
//     resetData();
//   }

//   Serial.print("<!--");
//   Serial.print(data.pot_value); /* Print received value on Serial Monitor */
//   Serial.println("--!>");
//   analogWrite(LED, data.pot_value);
// }
void setup()
{
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
}

void loop()
{
  digitalWrite(LED, HIGH);
  Serial.println("<!--HIGH--!>");
  delay(500);
  digitalWrite(LED, LOW);
  Serial.println("<!--LOW--!>");
  delay(500);
}