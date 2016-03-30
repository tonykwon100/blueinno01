/*  푸시 버튼에 의한 LED 밝기 바꾸기
  */
int R_LED = 13;   // Red LED가 연결된 핀을 4번으로 설정 
int G_LED = 14;   // Green LED가 연결된 핀을 5번으로 설정
int B_LED = 15;   // Blue LED가 연결된 핀을 6번으로 설정 
int DC_EN = 17; // DC EN 

void setup() {
pinMode(R_LED , OUTPUT);  // Red LED 핀을 출력으로 설정 
 pinMode(G_LED , OUTPUT);  // Green LED 핀을 출력으로 설정 
 pinMode(B_LED , OUTPUT);  // Blue LED 핀을 출력으로 설정 
 pinMode(DC_EN , OUTPUT); // DC EN 
 //digitalWrite ( DC_EN, HIGH); 
}


void loop() {
 
        analogWrite(R_LED, 255);  // Red LED의 밝기를 최대로 합니다. 
       delay(500);   // 500ms 동안 대기합니다. 

        analogWrite(R_LED, 123);   // Red LED의 밝기를 중간으로 합니다. 
        delay(500); // 500ms 동안 대기합니다. 

        analogWrite(R_LED, 0);     // Red LED의 밝기를 최소로 합니다. 
        delay(500); // 500ms 동안 대기합니다.         

       
       
        analogWrite(G_LED, 255);     // Green LED의 밝기를 최대로 합니다. 
         delay(500);   // 500ms 동안 대기합니다. 

        analogWrite(G_LED, 123);     // Green LED의 밝기를 중간으로 합니다. 
        delay(500); // 500ms 동안 대기합니다. 

         analogWrite(G_LED, 0);    // Green LED의 밝기를 최소로 합니다. 
         delay(500); // 500ms 동안 대기합니다.         
       
          analogWrite(B_LED, 255);     // Blue LED의 밝기를 최대로 합니다.     
           delay(500);   // 500ms 동안 대기합니다. 

           analogWrite(B_LED, 123);     // Blue LED의 밝기를 중간으로 합니다. 
           delay(500); // 500ms 동안 대기합니다. 

            analogWrite(B_LED, 0);    // Blue LED의 밝기를 최소로 합니다. 
           delay(500); // 500ms 동안 대기합니다.         
   
}

