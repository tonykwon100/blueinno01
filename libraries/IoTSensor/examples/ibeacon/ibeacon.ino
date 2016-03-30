/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
 
  This example code is in the public domain.
 */
 
// Pin 3 has an green LED connected on the RGB LED shield
// give it a name:

#include <RFduinoBLE.h>


// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
   // do iBeacon advertising
 RFduinoBLE.iBeacon = true;
  
  // start the BLE stack
  RFduinoBLE.begin();
  
 Serial.begin(9600);

}

// the loop routine runs over and over again forever:
void loop() {
                 // wait for a second
  
}
