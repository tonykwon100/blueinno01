// 스마트폰으로 LED 켜기
#include <RFduinoBLE.h>  // 블루투스 라이브러리를 사용하기 위해 헤더 파일 추가
int led = 13;                   // LED에 연결된 핀을  2번으로 설정 

void setup() {
pinMode(led, OUTPUT);   // LED를 출력으로 설정 
    RFduinoBLE.advertisementData = "ledbtn";      // 블루투스 통신 데이타를  
                                                                 // ledbtn 포맷 적용
 
RFduinoBLE.begin();    // 블루투스 (BLE) 스택의 시작함을 의미하며, 
                               // 이를 통하여 관련 서비스를  할 수 있습니다.
}
 void loop() {
 }

void RFduinoBLE_onReceive(char *data, int len) // 블루투스 수신 데이터를 처리 함수
                          // char data 란에 데이터(0~F), len 란에는 길이 표시 
{                                                               
if (data[0])                      // 수신한 데이터가 참(True, char 인 경우 = 1~ F) 이면   
digitalWrite(led, HIGH);      //  LED를  켜기 (High) 

 else                              // 수신한 데이터가 거짓(False, 0)이면
    digitalWrite(led, LOW);   // LED를 끄기 (Low)
}

