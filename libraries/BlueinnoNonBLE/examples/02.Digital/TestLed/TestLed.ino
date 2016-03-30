 
// the setup routine runs once when you press reset:
void setup() {                
  pinMode(0, OUTPUT);     
  pinMode(1, OUTPUT);     
  pinMode(2, OUTPUT);     
  pinMode(3, OUTPUT);     
  pinMode(4, OUTPUT);     
  pinMode(5, OUTPUT);     
  pinMode(6, OUTPUT);     
}

// the loop routine runs over and over again forever:
void loop() {
  int cnt;
  
  for(cnt =0; cnt < 7; cnt++)
  {
    digitalWrite(cnt, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(1000);                     // wait for a second
    digitalWrite(cnt, LOW);    // turn the LED off by making the voltage LOW
    delay(1000);                     // wait for a second
  }
}
