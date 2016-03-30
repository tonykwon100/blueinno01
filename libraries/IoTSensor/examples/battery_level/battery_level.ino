
// Battery level  Display  

int Battery_ADC = 3;  //  온도 핀이 연결된 핀을 2번으로 설정
int Battery_ADC_Value = 0;
float Battery_Voltage = 0;

void setup()
{
  analogReference(VBG); 
  pinMode(Battery_ADC, INPUT);
  Serial.begin(9600); 
   
   }     
 
void loop()
{
  // Battery_level 
  Battery_ADC_Value = analogRead(Battery_ADC);
  Battery_Voltage = (((Battery_ADC_Value*2)/1024.0)*3.8);
  delay(100); 
  
Serial.print("Battery_ADC="); 
Serial.println(Battery_ADC_Value); 
Serial.println("------------------");

Serial.print("Battery_Voltage="); 
Serial.print(Battery_Voltage); 
Serial.println("V");
Serial.println("------------------");
  delay(1000); 
}

