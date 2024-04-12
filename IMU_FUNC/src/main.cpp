

#include <Arduino.h>
#include <IRremote.h>

#define LED 2

IRrecv irrecv(kRecvPin);

decode_results results;

void setup()
{
  Serial.begin(115200);
  irrecv.enableIRIn(); // Start the receiver
  while (!Serial)      // Wait for the serial connection to be establised.
    delay(50);
  Serial.println();
  Serial.print("Waiting for IR Signal...");
  Serial.println(kRecvPin);
}

void loop()
{
  if (IrReceiver.decode())
  {
    switch (IrReceiver.decodedIRData.command)
    {
    case 0x84:
      digitalWrite(MOSFET_1, !digitalRead(MOSFET_1));
      Serial.println("<OUTPUT 1 Engaged>");
      break;

    case 0xD9:
      digitalWrite(MOSFET_2, !digitalRead(MOSFET_2));
      Serial.println("<OUTPUT 2 Engaged>");
      break;

    case 0x89:
      digitalWrite(BUZZ, !digitalRead(BUZZ));
      // Serial.println("btn3");
      break;

    default:
      Serial.println(IrReceiver.decodedIRData.command, HEX);
      //        Serial.println("Default");
    }
    IrReceiver.resume(); // Receive the Next Value
  }
}