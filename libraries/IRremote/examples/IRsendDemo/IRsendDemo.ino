/*
 * IRremote: IRsendDemo - demonstrates sending IR codes with IRsend
 * An IR LED must be connected to Arduino PWM pin 6.
 * Version 0.1 July, 2009
 * Copyright 2009 Ken Shirriff
 * http://arcfn.com
 */

#include <IRremote.h>

IRsend irsend;

unsigned long CODES_LGTV[] =
{
	// 0 ~ 9 ==> 0 ~ 9
	0x20df08f7, 0x20df8877, 0x20df48b7,0x20dfc837,0x20df28d7,0x20dfa857,0x20df6897,0x20dfe817,0x20df18e7,0x20df9867,
	// 10~15 ==> Power On/Off, Volume Up, Volume Down, Channel Up, Channel Down, Mute
	0x20df10ef, 0x20df40bf, 0x20dfc03f, 0x20DF807F, 0x20DF00FF, 0x20df906f
};

unsigned long CODES_SAMSUNGTV[] = 
{
	// 0 ~ 9 ==> 0 ~ 9
	0xE0E08877,0xE0E020DF,0xE0E0A05F,0xE0E0609F,0xE0E010EF,0xE0E0906F,0xE0E050AF,0xE0E030CF,0xE0E0B04F,0xE0E0708F,
	// 10~15 ==> Power On/Off, Volume Up, Volume Down, Channel Up, Channel Down, Mute
	0xE0E040BF,0xE0E0E01F,0xE0E0D02F,0xE0E008F7,0xE0E048B7,0xE0E0F00F
};
	
void printMenu()
{
	Serial.println("-- Remote Control Menu --");
	Serial.println(" 0~9 : Channel Number");
	Serial.println(" p : Power On/Off");
	Serial.println(" u : Volume Up");
	Serial.println(" d : Voulme Down");
	Serial.println(" l : Channel Down");
	Serial.println(" r : Channel Up");
	Serial.println(" m : Mute");
	Serial.print(">");
}

void sendCode(int idx)
{
	//irsend.sendLG(CODES_LGTV[idx], 32);
	irsend.sendSAMSUNG(CODES_SAMSUNGTV[idx], 32);
	delay(100);
}

void setup()
{
  // Sensor Power On.
  pinMode(17, OUTPUT);
  digitalWrite(17, HIGH);
  delay(10);
  
  Serial.begin(115200);
  printMenu();
}

void loop() {
	int c = -1;
	
	c = Serial.read();
  if (c != -1) {
  	Serial.println((char)c);
  	switch(c)
  	{
  		case '0': case '1':case '2':case '3':case '4':case '5':case '6':case '7':case '8':case '9':
  			sendCode(c-'0');
  			break;
  		case 'p': // Power On/Off
  			sendCode(10);
  			break;
  		case 'u': // Volume Up
  			sendCode(11);
  			break;
  		case 'd': // Volume Down
  			sendCode(12);
  			break;
  		case 'l': // Channel Down
  			sendCode(13);
  			break;
  		case 'r': // Channel Up
  			sendCode(14);
  			break;
  		case 'm': // Mute
  			sendCode(15);
  			break;
  	}
  	printMenu();
  }
  
  delay(100);
  
}
