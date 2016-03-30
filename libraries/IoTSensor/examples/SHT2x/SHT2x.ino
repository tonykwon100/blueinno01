#include <Wire.h>
#include <SHT2x.h>

void setup()
{
pinMode(17, OUTPUT);
digitalWrite(17, HIGH);
delay(50);
Serial.begin(115200);

 // Wire.beginOnPins (SCL pin, SDA pin) 
  Wire.beginOnPins(7,6);
   
}

void loop()
{
     Serial.print("Temperature: ");
     Serial.print(SHT2x.readT());
     Serial.print(" C, Humidity: ");
     Serial.print(SHT2x.readRH());
     Serial.println(" %RH");
     delay(1000);
     
}
