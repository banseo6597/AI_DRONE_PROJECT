//-----------------------------------------------------
#include <SoftwareSerial.h>

SoftwareSerial bleSerial(A0, A1); // RX, TX (조종기 쉴드 블루투스 핀)

//==== 드론 제어에 필요한 기본 변수 선언 ====
unsigned char startBit_1 = 0x26;
unsigned char startBit_2 = 0xa8;
unsigned char startBit_3 = 0x14;
unsigned char startBit_4 = 0xb1;
unsigned char len = 0x14;
unsigned char checkSum = 0;

int roll = 0;
int pitch = 0;
int yaw = 0;
int throttle = 0;
int option = 0x000f;

int p_vel = 0x0064;
int y_vel = 0x0064;
unsigned char drone_action = 0;
unsigned char payload[14];
unsigned int firstRoll;
unsigned int firstPitch;

// ==========================================
// ★ AI 하이브리드 제어 및 안전망(Watchdog) 변수
unsigned long ai_command_timestamp = 0;
const unsigned long AI_COMMAND_TIMEOUT_MS = 2000; // 2초 타임아웃
int ai_pitch = 0;
int ai_roll = 0;
int ai_throttle = 0;
// ==========================================

//-----------------------------------------------------

void checkThrottle() {
  if(!digitalRead(6)) { if(throttle > 9) throttle -= 10; } // 하강
  else if(!digitalRead(5)) { if(throttle < 141) throttle += 20; } // 상승
}

void checkYaw() {
  if(throttle == 0) yaw = 0;
  if(!digitalRead(7)) { if(yaw > -170) yaw -= 10; } // 좌회전
  else if(!digitalRead(8)) { if(yaw < 170) yaw += 10; } // 우회전
}

void checkEmergency() {
  if(!digitalRead(9)) {
    roll = 0; pitch = 0; yaw = 0; throttle = 0; option = 0x000e;
  } else { option = 0x000f; }
}

void checkRoll() {
  unsigned int secondRoll = analogRead(4);
  if(secondRoll < firstRoll - 450) roll = -200;
  else if(secondRoll < firstRoll - 350) roll = -160;
  else if(secondRoll < firstRoll - 250) roll = -120;
  else if(secondRoll < firstRoll - 150) roll = -80;
  else if(secondRoll < firstRoll - 50) roll = -40;
  else if(secondRoll < firstRoll + 50) roll = 0; // 데드존 (중앙)
  else if(secondRoll < firstRoll + 150) roll = 40;
  else if(secondRoll < firstRoll + 250) roll = 80;
  else if(secondRoll < firstRoll + 350) roll = 120;
  else if(secondRoll < firstRoll + 450) roll = 160;
  else roll = 200;
}

void checkPitch() {
  unsigned int secondPitch = analogRead(5);
  if(secondPitch < firstPitch - 450) pitch = -200;
  else if(secondPitch < firstPitch - 350) pitch = -160;
  else if(secondPitch < firstPitch - 250) pitch = -120;
  else if(secondPitch < firstPitch - 150) pitch = -80;
  else if(secondPitch < firstPitch - 50) pitch = -40;
  else if(secondPitch < firstPitch + 50) pitch = 0; // 데드존 (중앙)
  else if(secondPitch < firstPitch + 150) pitch = 40;
  else if(secondPitch < firstPitch + 250) pitch = 80;
  else if(secondPitch < firstPitch + 350) pitch = 120;
  else if(secondPitch < firstPitch + 450) pitch = 160;
  else pitch = 200;
}

void sendDroneCommand() {
  bleSerial.print("at+writeh000d");
  bleSerial.print(String(startBit_1, HEX)); bleSerial.print(String(startBit_2, HEX));
  bleSerial.print(String(startBit_3, HEX)); bleSerial.print(String(startBit_4, HEX));
  bleSerial.print(String(len, HEX));
  
  if(checkSum < 0x10) bleSerial.print("0" + String(checkSum, HEX));
  else bleSerial.print(String(checkSum, HEX));

  for(int i=0; i<14; i++) {
    if(payload[i] < 0x10) bleSerial.print("0" + String(payload[i], HEX));
    else bleSerial.print(String(payload[i], HEX));
  }
  bleSerial.print("\r");
  delay(50);
}

void checkCRC() {
  memset(payload, 0x00, 14);
  payload[0] = (roll) & 0x00ff; payload[1] = (roll >> 8) & 0x00ff;
  payload[2] = (pitch) & 0x00ff; payload[3] = (pitch >> 8) & 0x00ff;
  payload[4] = (yaw) & 0x00ff; payload[5] = (yaw >> 8) & 0x00ff;
  payload[6] = (throttle) & 0x00ff; payload[7] = (throttle >> 8) & 0x00ff;
  payload[8] = (option) & 0x00ff; payload[9] = (option >> 8) & 0x00ff;
  payload[10] = (p_vel) & 0x00ff; payload[11] = (p_vel >> 8) & 0x00ff;
  payload[12] = (y_vel) & 0x00ff; payload[13] = (y_vel >> 8) & 0x00ff;
  
  checkSum = 0;
  for(int i = 0; i < 14; i++) checkSum += payload[i];
  checkSum = checkSum & 0x00ff;
}

unsigned char startDroneControl() {
   // 비상/시작 버튼(9번 핀)을 눌러야 시작됨
   if(!digitalRead(9)) {
     firstRoll = analogRead(4);
     firstPitch = analogRead(5);
     drone_action = 1;
   }
   return drone_action;
}

void setup() {
  Serial.begin(9600); // 라즈베리파이와 통신할 속도
  bleSerial.begin(9600); // 블루투스 통신 속도

  for(int i = 5; i < 11; i++) {
    pinMode(i, INPUT);
    digitalWrite(i, HIGH); // 내부 풀업 저항 활성화
  }
  delay(500);
}

// ==========================================
// ★★★ 통신 강화 패치가 적용된 무한 루프 ★★★
void loop() { 
  if(startDroneControl()) {
    
    // 1. 조이스틱(수동) 입력값 및 비상버튼 확인
    checkThrottle();
    checkRoll();
    checkPitch();
    checkYaw();
    checkEmergency();
    
    if (option == 0x000e) {
      ai_pitch = 0; ai_roll = 0; ai_throttle = 0; ai_command_timestamp = 0; 
    }
    
    bool isManualOverride = (roll != 0 || pitch != 0 || yaw != 0 || !digitalRead(5) || !digitalRead(6));

    // 2. 🚀 USB 통신 쪼개짐 방지 (강화된 프레이밍 수신부)
    if (Serial.available() > 0) {
      char startChar = Serial.read();
      if (startChar == '<') {
        
        // 나머지 2글자(예: U와 >)가 도착할 때까지 최대 50ms 기다림
        unsigned long waitTime = millis();
        while(Serial.available() < 2 && (millis() - waitTime) < 50) { 
          // 무한 대기 방지
        }

        if (Serial.available() >= 2) {
          char aiCommand = Serial.read();
          char endChar = Serial.read();

          if (endChar == '>') {
            ai_command_timestamp = millis(); 

            if (aiCommand == 'F') { 
              ai_pitch = 80; ai_roll = 0; 
            } 
            else if (aiCommand == 'S') { 
              ai_pitch = 0; ai_roll = 0; 
            }
            else if (aiCommand == 'U') { 
              ai_pitch = 0; ai_roll = 0;
              ai_throttle = 100;  // 고도를 80에서 100으로 올려서 힘을 키움!
            }
          }
        }
      }
    }

    // 3. 권한 중재 및 Watchdog 로직
    if (isManualOverride) {
      ai_pitch = 0; ai_roll = 0; ai_throttle = 0;
    } else {
      if (millis() - ai_command_timestamp > AI_COMMAND_TIMEOUT_MS) {
        ai_pitch = 0;
        ai_roll = 0;
      }
      pitch = ai_pitch;
      roll = ai_roll;
      if (ai_throttle > 0) {
        throttle = ai_throttle; 
      }
    }

    // 4. 무선 전송
    checkCRC();
    sendDroneCommand();
  }
}

