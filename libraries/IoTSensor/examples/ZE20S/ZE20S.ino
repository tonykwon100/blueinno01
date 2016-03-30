#include <SoftwareSerial.h> 

SoftwareSerial mySerial(1, 2); // RX, TX

void setup(void)
{
  Serial.begin(115200);                                         //send and receive at 9600 baud
  Serial.println("********************* SHINC ******************");

  mySerial.begin(9600);
  Serial.println("************* Zigbee Serial Connect **********");

}

void loop(void)
{
  if (mySerial.available()) {
     Serial.write(mySerial.read());
   }
   if (Serial.available()) {
     mySerial.write(Serial.read());
   }
}