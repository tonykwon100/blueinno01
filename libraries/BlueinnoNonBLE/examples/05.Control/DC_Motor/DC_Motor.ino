// DC-Moter Control

int Motor1speed = 3; // Motor1 �ӵ�����
int Motor2speed = 2; // Motor2 �ӵ�����
int Motor1direction = 5;	// Motor1 ��������
int Motor2direction = 4;	// Motor2 ��������

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
	digitalWrite(Motor1direction, HIGH); //����
	digitalWrite(Motor2direction, HIGH); //����
	analogWrite(Motor1speed, pwm);				// �ӵ�
	analogWrite(Motor2speed, pwm);				// �ӵ�
	delay(duration);
	analogWrite(Motor1speed, 0);				// �ӵ�
	analogWrite(Motor2speed, 0);				// �ӵ�
}

void goBackward(int duration, int pwm)
{
	digitalWrite(Motor1direction, LOW); //����
	digitalWrite(Motor2direction, LOW); //����
	analogWrite(Motor1speed, pwm);				// �ӵ�
	analogWrite(Motor2speed, pwm);				// �ӵ�
	delay(duration);
	analogWrite(Motor1speed, 0);				// �ӵ�
	analogWrite(Motor2speed, 0);				// �ӵ�
}

void rotateRight(int duration, int pwm)
{
	digitalWrite(Motor1direction, HIGH); //����
	digitalWrite(Motor2direction, LOW); //����
	analogWrite(Motor1speed, pwm);				// �ӵ�
	analogWrite(Motor2speed, pwm);				// �ӵ�
	delay(duration);
	analogWrite(Motor1speed, 0);				// �ӵ�
	analogWrite(Motor2speed, 0);				// �ӵ�
}

void rotateLeft(int duration, int pwm)
{
	digitalWrite(Motor1direction, LOW); //����
	digitalWrite(Motor2direction, HIGH); //����
	analogWrite(Motor1speed, pwm);				// �ӵ�
	analogWrite(Motor2speed, pwm);				// �ӵ�
	delay(duration);
	analogWrite(Motor1speed, 0);				// �ӵ�
	analogWrite(Motor2speed, 0);				// �ӵ�
}

void loop()
{
	goForward(1000,255);
	goBackward(1000,255);
	rotateRight(1000,255);
	rotateLeft(1000,255);
	delay(2000);
}
