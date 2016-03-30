// Humi1 sensor : Humidity & Temperature Sensor
#include "Wire.h"

#define humi_addr		0x28

const int dataLength = 4;
static byte rawData[dataLength];


void setup() 
{
	Serial.begin(9600);
	Wire.begin(); 
}

boolean humi1Read()
{
	int cnt = 0;
	Wire.requestFrom(humi_addr, 4);

	while(Wire.available())
	{
		rawData[cnt] = Wire.read();
		cnt++;
	}	
	
	if(cnt >= dataLength)
		return true;
	else
		return false;
}

float CalHumidity()
{
	long RH_H;
	float result;
	
	RH_H = (long)(rawData[0] & 0x3f);
	result = (((float)(RH_H*256+(long)rawData[1])/16384)*100);
	return result;
}

float CalTemp()
{
        long temp_low;
        float result;
        
        temp_low = long(rawData[3]>>2);
        result = ((float)((long)rawData[2]*64 + temp_low/4)/16384)*165 - 40;
        return result;
}

void loop() 
{
	float humidity, temperature;
		
	Serial.print("humi1:\t");
	
	humi1Read();
	
	humidity = CalHumidity();
  temperature = CalTemp();
        
	Serial.print("  humidity = ");
	Serial.print(humidity);
	Serial.print("  temperature = ");
	Serial.println(temperature);
	
	delay(1000);
}
