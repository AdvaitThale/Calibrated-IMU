#include <SPI.h>                  /* to handle the communication interface with the modem*/
#include <nRF24L01.h>             /* to handle this particular modem driver*/
#include <RF24.h>                 /* the library which helps us to control the radio modem*/

#define pot_pin A0                /*Variable pin of POT is to be connected to analog pin 0 i.e.A0*/
RF24 radio(9, 10);                   // CE -> D9 | CSN/SS/CS -> D10
const uint64_t Address = 0xE8E8F0F0E1LL ;     /* Address to which data to be transmitted*/

struct MyData {
  byte pot_value;  
};

MyData data;

void resetData() 
{
  data.pot_value = 0;  
}

void setup() {
  Serial.begin(9600);
  pinMode(pot_pin, INPUT);        /* Setting A0 (POT pin) as input*/
  radio.begin ();                 /* Activate the modem*/
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe (Address); /* Sets the address of transmitter to which program will send the data */
  Serial.print("<!--TRANSMISSION SEQUENCE STARTED--!>");
  resetData();
}
void loop() {
  radio.stopListening ();          /* Setting modem in transmission mode*/
  int value = analogRead(pot_pin);    /*Reading analog value at pin A0 and storing in varible 'value'*/
  data.pot_value = map( value , 0 , 1023 , 0 , 255 );   /* Convering the 10 bit value to 8 bit */
  radio.write(&data, sizeof(MyData));            /* Sending data over NRF 24L01*/
  char buff[20];
  sprintf(buff, "<!--%d--!>", data);
  Serial.println(buff);                           /* Printing POT value on serial monitor*/
}
