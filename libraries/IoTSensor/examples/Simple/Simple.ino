/*
 Copyright (c) 2014 OpenSourceRF.com.  All right reserved.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 See the GNU Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <Wire.h>
#include <SHT2x.h>
#include <EEPROM.h>

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
const int R_LED = 13;   // Red LED�� ����� ���� 4������ ���� 
const int G_LED = 14;   // Green LED�� ����� ���� 5������ ����
const int B_LED = 15;   // Blue LED�� ����� ���� 6������ ���� 
const int endc = 17;
const int iled = 12;                                            //drive the led of sensor
const int vout = 4;                                            //analog input
const int buzzerPin = 16;

/*
variable
*/
float density, voltage;
float   adcvalue;

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

void mySleep()
{
	Serial.println("-- Sleep");
	RFduino_pinWake(0, LOW);
	RFduino_ULPDelay(INFINITE);
}

int powerCallback(uint32_t ulPin)
{

	mySleep();
	
  return 0;
}


void readDustSensor()
{
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
}

void setup() {
	
	Serial.begin(9600);

	// Sensor Power On.
	pinMode(17, OUTPUT);
	digitalWrite(17, HIGH);
	delay(10);
	
	// I2C Init for Temp. & Humidity Sensor
	Wire.beginOnPins(7,6);
	
  // processing handled by exiting RFduino_ULPDelay
  pinMode(0, INPUT);

	// Dust Sensor Init.
  pinMode(iled, OUTPUT);
  digitalWrite(iled, LOW);                                     //iled default closed
  pinMode(endc, OUTPUT);
  digitalWrite(endc, HIGH);

	// Buzzer Init..
	pinMode(buzzerPin, OUTPUT);

	Serial.println("setup");
		
}

void loop() {

	float temp, humidity;
	uint8_t *p;
	int addr;
	int i;
	
	Serial.println("-- Start");
	
	// Read Temp/Humidity
	Serial.println("-- Read Temp/Humidity");
	temp = SHT2x.readT();
	humidity = SHT2x.readRH();
	Serial.print("Temperature: ");
	Serial.print(temp);
	Serial.print(" C, Humidity: ");
	Serial.print(humidity);
	Serial.println(" %RH");	
	
	// Read Dust Sensor
	Serial.println("-- Read Dust Sensor");
	readDustSensor();
	
	// Write Temp,Humidity,Dust Density
	Serial.println("-- Write Temp,Humidity,Dust Density");
	addr = 0;
	p = (uint8_t *)&temp;
	EEPROM.write(addr++, *p++);
	EEPROM.write(addr++, *p++);
	EEPROM.write(addr++, *p++);
	EEPROM.write(addr++, *p++);
	p = (uint8_t *)&humidity;
	EEPROM.write(addr++, *p++);
	EEPROM.write(addr++, *p++);
	EEPROM.write(addr++, *p++);
	EEPROM.write(addr++, *p++);
	p = (uint8_t *)&density;
	EEPROM.write(addr++, *p++);
	EEPROM.write(addr++, *p++);
	EEPROM.write(addr++, *p++);
	EEPROM.write(addr++, *p++);

	// Read Temp,Humidity,Dust Density
	Serial.println("-- Read Temp,Humidity,Dust Density");
	addr = 0;
	p = (uint8_t *)&temp;
	*p++ = EEPROM.read(addr++);
	*p++ = EEPROM.read(addr++);
	*p++ = EEPROM.read(addr++);
	*p++ = EEPROM.read(addr++);
	p = (uint8_t *)&humidity;
	*p++ = EEPROM.read(addr++);
	*p++ = EEPROM.read(addr++);
	*p++ = EEPROM.read(addr++);
	*p++ = EEPROM.read(addr++);
	p = (uint8_t *)&density;
	*p++ = EEPROM.read(addr++);
	*p++ = EEPROM.read(addr++);
	*p++ = EEPROM.read(addr++);
	*p++ = EEPROM.read(addr++);
	Serial.print("(Temp,Humidity,Density)=");
	Serial.print(temp);
	Serial.print(",");
	Serial.print(humidity);
	Serial.print(",");
	Serial.println(density);
	
	// LED & Buzzer Test
	Serial.println("-- LED & Buzzer Test");
  tone(buzzerPin, 1000, 1000);
	for(i = 0 ; i < 256; i++)
	{
		analogWrite(R_LED, i);
		delay(2);
	}
  noTone(buzzerPin);
	delay(1000);
	analogWrite(R_LED, 0);
	tone(buzzerPin, 1000, 1000);
	for(i = 0 ; i < 256; i++)
	{
		analogWrite(G_LED, i);
		delay(2);
	}
	noTone(buzzerPin);
	delay(1000);
	analogWrite(G_LED, 0);
	tone(buzzerPin, 1000, 1000);
	for(i = 0 ; i < 256; i++)
	{
		analogWrite(B_LED, i);
		delay(2);
	}
	noTone(buzzerPin);
	delay(1000);
	analogWrite(B_LED, 0);

	// Set Power Button Callback.
	RFduino_pinWakeCallback(0, LOW, powerCallback);
	while(1)
	{
		delay(1000);
		Serial.print(".");
	}	
}
