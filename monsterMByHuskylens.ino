#include "FaBoPWM_PCA9685.h"
#include "HUSKYLENS.h"
#include "SoftwareSerial.h"

FaBoPWM faboPWM;
HUSKYLENS huskylens;
SoftwareSerial mySerial(8, 9); // RX, TX

int pos = 0;
int MAX_VALUE = 2000;   // 모터 속도 제한
int MIN_VALUE = 300;

char Uart_Date=0;

#define DIRA1 0
#define DIRA2 1
#define DIRB1 2
#define DIRB2 3
#define DIRC1 4
#define DIRC2 5
#define DIRD1 6
#define DIRD2 7

//모터 제어, 전진, 후진, 정지
#define MOTORA_FORWARD(pwm)    do{faboPWM.set_channel_value(DIRA1,pwm);faboPWM.set_channel_value(DIRA2, 0);}while(0)
#define MOTORA_STOP(x)         do{faboPWM.set_channel_value(DIRA1,0);faboPWM.set_channel_value(DIRA2, 0);}while(0)
#define MOTORA_BACKOFF(pwm)    do{faboPWM.set_channel_value(DIRA1,0);faboPWM.set_channel_value(DIRA2, pwm);}while(0)

#define MOTORB_FORWARD(pwm)    do{faboPWM.set_channel_value(DIRB1,pwm);faboPWM.set_channel_value(DIRB2, 0);}while(0)
#define MOTORB_STOP(x)         do{faboPWM.set_channel_value(DIRB1,0);faboPWM.set_channel_value(DIRB2, 0);}while(0)
#define MOTORB_BACKOFF(pwm)    do{faboPWM.set_channel_value(DIRB1,0);faboPWM.set_channel_value(DIRB2, pwm);}while(0)

#define MOTORC_FORWARD(pwm)    do{faboPWM.set_channel_value(DIRC1,pwm);faboPWM.set_channel_value(DIRC2, 0);}while(0)
#define MOTORC_STOP(x)         do{faboPWM.set_channel_value(DIRC1,0);faboPWM.set_channel_value(DIRC2, 0);}while(0)
#define MOTORC_BACKOFF(pwm)    do{faboPWM.set_channel_value(DIRC1,0);faboPWM.set_channel_value(DIRC2, pwm);}while(0)

#define MOTORD_FORWARD(pwm)    do{faboPWM.set_channel_value(DIRD1,pwm);faboPWM.set_channel_value(DIRD2, 0);}while(0)
#define MOTORD_STOP(x)         do{faboPWM.set_channel_value(DIRD1,0);faboPWM.set_channel_value(DIRD2, 0);}while(0)
#define MOTORD_BACKOFF(pwm)    do{faboPWM.set_channel_value(DIRD1,0);faboPWM.set_channel_value(DIRD2, pwm);}while(0)

#define SERIAL  Serial

#define LOG_DEBUG

#ifdef LOG_DEBUG
#define M_LOG SERIAL.print
#else
#define M_LOG 
#endif

//PWM매개변수
#define MAX_PWM   2000
#define MIN_PWM   400

int Motor_PWM = 500;
int LR_PWM = 800;
int rotation_PWM = 500;
 
//모터 움직임 제어 매크로 정의
//    ↑A-----B↑   
//     |  ↑  |
//     |  |  |
//    ↑C-----D↑
void ADVANCE(uint8_t pwm_A,uint8_t pwm_B,uint8_t pwm_C,uint8_t pwm_D)
{
  MOTORA_BACKOFF(Motor_PWM);MOTORB_FORWARD(Motor_PWM);    
  MOTORC_BACKOFF(Motor_PWM);MOTORD_FORWARD(Motor_PWM);    
}

//    ↓A-----B↓ 
//     |  |  |
//     |  ↓  |
//    ↓C-----D↓
void BACK()
{
  MOTORA_FORWARD(Motor_PWM);MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);MOTORD_BACKOFF(Motor_PWM);
}
//    =A-----B↑   
//     |   ↖ |
//     | ↖   |
//    ↑C-----D=
void LEFT_1()
{
  MOTORA_STOP(Motor_PWM);MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM);MOTORD_STOP(Motor_PWM);
}

//    ↓A-----B↑   
//     |  ←  |
//     |  ←  |
//    ↑C-----D↓
void LEFT_2()
{
  MOTORA_FORWARD(LR_PWM);MOTORB_FORWARD(LR_PWM);
  MOTORC_BACKOFF(LR_PWM);MOTORD_BACKOFF(LR_PWM);
}
//    ↓A-----B=   
//     | ↙   |
//     |   ↙ |
//    =C-----D↓
void LEFT_3()
{
  MOTORA_FORWARD(Motor_PWM);MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);MOTORD_BACKOFF(Motor_PWM);
}
//    ↑A-----B=   
//     | ↗   |
//     |   ↗ |
//    =C-----D↑
void RIGHT_1()
{
  MOTORA_BACKOFF(Motor_PWM);MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);MOTORD_FORWARD(Motor_PWM);
}
//    ↑A-----B↓   
//     |  →  |
//     |  →  |
//    ↓C-----D↑
void RIGHT_2()
{
  MOTORA_BACKOFF(LR_PWM);MOTORB_BACKOFF(LR_PWM);
  MOTORC_FORWARD(LR_PWM);MOTORD_FORWARD(LR_PWM);
}
//    =A-----B↓   
//     |   ↘ |
//     | ↘   |
//    ↓C-----D=
void RIGHT_3()
{
  MOTORA_STOP(Motor_PWM);MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);MOTORD_STOP(Motor_PWM);
}

//    ↑A-----B↓   
//     | ↗ ↘ |
//     | ↖ ↙ |
//    ↑C-----D↓
void rotate_1()  //tate_1(uint8_t pwm_A,uint8_t pwm_B,uint8_t pwm_C,uint8_t pwm_D) 
{
  MOTORA_BACKOFF(rotation_PWM);MOTORB_BACKOFF(rotation_PWM);
  MOTORC_BACKOFF(rotation_PWM);MOTORD_BACKOFF(rotation_PWM);
}

//    ↓A-----B↑   
//     | ↙ ↖ |
//     | ↘ ↗ |
//    ↓C-----D↑
void rotate_2()  // rotate_2(uint8_t pwm_A,uint8_t pwm_B,uint8_t pwm_C,uint8_t pwm_D)
{
  MOTORA_FORWARD(rotation_PWM);MOTORB_FORWARD(rotation_PWM);
  MOTORC_FORWARD(rotation_PWM);MOTORD_FORWARD(rotation_PWM);
}
//    =A-----B=  
//     |  =  |
//     |  =  |
//    =C-----D=
void STOP()
{
  MOTORA_STOP(Motor_PWM);MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);MOTORD_STOP(Motor_PWM);
}

//직렬 입력 제어
void UART_Control()
{
  if(SERIAL.available())
  {
    Uart_Date = SERIAL.read();
  }
  switch(Uart_Date)
  {
     case 'w':  ADVANCE(500,500,500,500);  M_LOG("Forward!\r\n");        break;
     case 'e':  RIGHT_1();  M_LOG("Right up!\r\n");     break;
     case 'A':  rotate_2(); M_LOG("CCW!\r\n");          break;      
     case 'c':  RIGHT_3();  M_LOG("Right down!\r\n");   break;
     case 'x':  BACK();     M_LOG("Backward!\r\n");          break;
     case 'z':  LEFT_3();   M_LOG("Left down!\r\n");    break;
     case 'D':  rotate_1(); M_LOG("CW!\r\n");           break;         
     case 'q':  LEFT_1();   M_LOG("Left up!\r\n");     break;
     case 'S':  STOP();     M_LOG("Stop!\r\n");        break;
     case 's':  STOP();     M_LOG("Stop!\r\n");        break;
     case 'a':  LEFT_2();   M_LOG("Left!\r\n");        break;
     case 'd':  RIGHT_2();  M_LOG("Right!\r\n");        break;
     case 'L':  Motor_PWM = 2000;  LR_PWM = 2000; M_LOG("Speed Up!\r\n");    break;
     case 'M':  Motor_PWM = 500;   LR_PWM = 1000; M_LOG("Speed Down!\r\n");  break;
     case 'o':  Motor_PWM -= 100;   LR_PWM -= 100;  M_LOG("Speed Down!\r\n");   
        if(Motor_PWM < 500) {
          Motor_PWM = 500;
        }
        if(LR_PWM < 800) {
          LR_PWM = 800;
        }
        Serial.print(Motor_PWM);  Serial.print("  "); Serial.println(LR_PWM);    break;
     case 'p':  Motor_PWM += 100;   LR_PWM += 100;  M_LOG("Speed Up!\r\n");    
        if(Motor_PWM > 1700) {
          Motor_PWM = 1700;
        }
        if(LR_PWM > 2000) {
          LR_PWM = 2000;
        }
        Serial.print(Motor_PWM);  Serial.print("  "); Serial.println(LR_PWM);    break;
   }
}

void printResult(HUSKYLENSResult result){
    if (result.command == COMMAND_RETURN_BLOCK){
      Serial.println(String()+F("Block:xCenter=")+result.xCenter+F(",yCenter=")+result.yCenter+F(",width=")+result.width+F(",height=")+result.height+F(",ID=")+result.ID);
      
      if(result.xCenter > 180) {
        Uart_Date = 'D';
        Serial.println(String() + Uart_Date + F("  right"));   
      }
      if(result.xCenter < 140) {
        Uart_Date = 'A';
        Serial.println(String() + Uart_Date + F("  left"));   
      }
       if(result.xCenter <= 180 && result.xCenter >=140) {
        Uart_Date = 's';
        Serial.println(String() + Uart_Date + F("  left"));   
      }
      if(result.width < 60 && result.height < 60) {
        Uart_Date = 'w';
        Serial.println(String() + Uart_Date + F("  forward"));      
      } 
      if(result.width > 90 && result.height > 90) {
        Uart_Date = 'x';
        Serial.println(String() + Uart_Date + F("  backward"));  
      }
    }
    else if (result.command == COMMAND_RETURN_ARROW){
      Serial.println(String()+F("Arrow:xOrigin=")+result.xOrigin+F(",yOrigin=")+result.yOrigin+F(",xTarget=")+result.xTarget+F(",yTarget=")+result.yTarget+F(",ID=")+result.ID);
      Uart_Date = 'w';
      
      if(result.xOrigin < result.xTarget && result.xOrigin - result.xTarget < -30) {
        Uart_Date = 'D';
        Serial.println(String() + Uart_Date + F("  right"));  
        if(result.xOrigin < 100) {
          Uart_Date = 'w';
          Serial.println(String() + Uart_Date + F("  forward"));   
        } 
      }
      if(result.xOrigin > result.xTarget && result.xOrigin - result.xTarget > 30) {
        Uart_Date = 'A';
        Serial.println(String() + Uart_Date + F("  left"));   
        if(result.xOrigin > 200) {
          Uart_Date = 'w';
          Serial.println(String() + Uart_Date + F("  forward"));   
        } 
      }
    }
    else{
      Serial.println("Object unknown!");
    }


}

void IO_init()
{
  STOP();
}

void setup()
{
  SERIAL.begin(9600);
  IO_init();
  if(faboPWM.begin()) 
  {
    Serial.println("Find PCA9685");
    faboPWM.init(300);
  }
  
  faboPWM.set_hz(50);
  
  Serial.begin(9600);
  mySerial.begin(9600);
  while (!huskylens.begin(mySerial))
  {
    Serial.println(F("Begin failed!"));
    Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>Serial 9600)"));
    Serial.println(F("2.Please recheck the connection."));
    delay(100);
  }
  SERIAL.print("Start");
}



void loop()
{
  UART_Control();//직렬포트 수신처리, 앱을 이용하여 제어할 때 사용

  // 아래는 허스키 렌즈로 제어할 때 사용
  if (!huskylens.request()) Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
  else if(!huskylens.isLearned()) Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
  else if(!huskylens.available()) Serial.println(F("No block or arrow appears on the screen!"));
  else
  {
    while (huskylens.available())
    {
      HUSKYLENSResult result = huskylens.read();
      printResult(result);
    }    
  }
}
