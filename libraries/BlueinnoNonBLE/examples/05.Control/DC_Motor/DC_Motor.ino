// DC-Moter Control

int Motor1speed = 3; // Motor1 속도제어
int Motor2speed = 2; // Motor2 속도제어
int Motor1direction = 5;	// Motor1 방향제어
int Motor2direction = 4;	// Motor2 방향제어

void setup()
{
	pinMode(Motor1speed, OUTPUT);
	pinMode(Motor2speed, OUTPUT);
	pinMode(Motor1direction, OUTPUT);
	pinMode(Motor2direction, OUTPUT);
	delay(5000);
}
void goForward(int duration, int pwm)
{
	digitalWrite(Motor1direction, HIGH); //전진
	digitalWrite(Motor2direction, HIGH); //전진
	analogWrite(Motor1speed, pwm);				// 속도
	analogWrite(Motor2speed, pwm);				// 속도
	delay(duration);
	analogWrite(Motor1speed, 0);				// 속도
	analogWrite(Motor2speed, 0);				// 속도
}

void goBackward(int duration, int pwm)
{
	digitalWrite(Motor1direction, LOW); //후진
	digitalWrite(Motor2direction, LOW); //후진
	analogWrite(Motor1speed, pwm);				// 속도
	analogWrite(Motor2speed, pwm);				// 속도
	delay(duration);
	analogWrite(Motor1speed, 0);				// 속도
	analogWrite(Motor2speed, 0);				// 속도
}

void rotateRight(int duration, int pwm)
{
	digitalWrite(Motor1direction, HIGH); //전진
	digitalWrite(Motor2direction, LOW); //후진
	analogWrite(Motor1speed, pwm);				// 속도
	analogWrite(Motor2speed, pwm);				// 속도
	delay(duration);
	analogWrite(Motor1speed, 0);				// 속도
	analogWrite(Motor2speed, 0);				// 속도
}

void rotateLeft(int duration, int pwm)
{
	digitalWrite(Motor1direction, LOW); //후진
	digitalWrite(Motor2direction, HIGH); //전진
	analogWrite(Motor1speed, pwm);				// 속도
	analogWrite(Motor2speed, pwm);				// 속도
	delay(duration);
	analogWrite(Motor1speed, 0);				// 속도
	analogWrite(Motor2speed, 0);				// 속도
}

void loop()
{
	goForward(1000,255);
	goBackward(1000,255);
	rotateRight(1000,255);
	rotateLeft(1000,255);
	delay(2000);
}
