
/*********************************************************************************************************
*
* File                : DustSensor
* Hardware Environment: 
* Build Environment   : Arduino
* Version             : V1.0.5-r2
* By                  : SH I&C
*
*                                  (c) Copyright 2015.11.05
*                                                                        
*                                          All Rights Reserved
*
*********************************************************************************************************/

// dust Sensor Define 
#define        COV_RATIO                       0.2            //ug/mmm / mv
#define        NO_DUST_VOLTAGE                 400            //mv
#define        SYS_VOLTAGE                     4500           

/*
 String Test
 */
String strTest = "";
uint16_t value = 0;
uint8_t low=0, high=0;
byte error;
/*
I/O define
*/
const int endc = 17;
const int iled = 12;                                            //drive the led of sensor
const int vout = 4;                                            //analog input

/*
variable
*/
float density, voltage;
float   adcvalue;

/*
private function
*/
float Filter(float m)
{
	int i;
  static int flag_first = 0;
  static float _buff[10], sum;
  const int _buff_max = 10;
  
  if(flag_first == 0)
  {
    flag_first = 1;

    for(i = 0, sum = 0; i < _buff_max; i++)
    {
      _buff[i] = m;
      sum += _buff[i];
    }
    return m;
  }
  else
  {
    sum -= _buff[0];
    for(i = 0; i < (_buff_max - 1); i++)
    {
      _buff[i] = _buff[i + 1];
    }
    _buff[9] = m;
    sum += _buff[9];

    return sum / 10.0;
  }
}


void setup(void)
{
  pinMode(iled, OUTPUT);
  digitalWrite(iled, LOW);                                     //iled default closed

  Serial.begin(115200);                                         //send and receive at 9600 baud
  Serial.println("********************* SHINC ******************");

  pinMode(endc, OUTPUT);
  digitalWrite(endc, HIGH);
}

void loop(void)
{
  /*
  get adcvalue
  */
  digitalWrite(iled, LOW);
  delayMicroseconds(280);
  adcvalue = analogRead(vout);
  delayMicroseconds(40);
  digitalWrite(iled, HIGH);
  delayMicroseconds(9680);
  
  adcvalue = Filter(adcvalue);
  
  /*
  covert voltage (mv)
  */
  voltage = (SYS_VOLTAGE / 1024.0) * adcvalue * 11;
  //voltage_real = (SYS_VOLTAGE / 1024.0)  ; //원래 주석처리되어있었음.
  /*
  voltage to density
  */
  if(voltage >= NO_DUST_VOLTAGE)
  {
    voltage -= NO_DUST_VOLTAGE;
    
    density = voltage * COV_RATIO;
  }
  else
    density = 0;
    
  /*
  display the result
  */
  
  Serial.print("The current dust is: ");
  Serial.print(density);
  Serial.println(" ug/m3");  
  Serial.print(voltage);  //2015-1111 ykk
  Serial.println(" = voltag"); 
  Serial.print(adcvalue);
  Serial.println(" = adcvalue");
  
  delay(1000);
  
}
