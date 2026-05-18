//-----------------------------------------------------
#include <SoftwareSerial.h>
SoftwareSerial bleSerial(A0, A1); 

unsigned char startBit_1 = 0x26, startBit_2 = 0xa8, startBit_3 = 0x14, startBit_4 = 0xb1;
unsigned char len = 0x14, checkSum = 0, drone_action = 0, payload[14];
int roll = 0, pitch = 0, yaw = 0, throttle = 0, option = 0x000f;
int p_vel = 0x0064, y_vel = 0x0064;
unsigned int firstRoll, firstPitch;

// AI 제어용 변수
unsigned long ai_command_timestamp = 0;
const unsigned long AI_COMMAND_TIMEOUT_MS = 2000;
int ai_pitch = 0, ai_roll = 0, ai_throttle = 0;

void checkThrottle() {
  if(!digitalRead(6)) { if(throttle > 9) throttle -= 10; }
  else if(!digitalRead(5)) { if(throttle < 141) throttle += 20; }
}

void checkYaw() {
  if(throttle == 0) yaw = 0;
  if(!digitalRead(7)) { if(yaw > -170) yaw -= 10; }
  else if(!digitalRead(8)) { if(yaw < 170) yaw += 10; }
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
  else if(secondRoll < firstRoll + 50) roll = 0; 
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
  else if(secondPitch < firstPitch + 50) pitch = 0; 
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
  payload[0] = roll & 0x00ff; payload[1] = (roll >> 8) & 0x00ff;
  payload[2] = pitch & 0x00ff; payload[3] = (pitch >> 8) & 0x00ff;
  payload[4] = yaw & 0x00ff; payload[5] = (yaw >> 8) & 0x00ff;
  payload[6] = throttle & 0x00ff; payload[7] = (throttle >> 8) & 0x00ff;
  payload[8] = option & 0x00ff; payload[9] = (option >> 8) & 0x00ff;
  payload[10] = p_vel & 0x00ff; payload[11] = (p_vel >> 8) & 0x00ff;
  payload[12] = y_vel & 0x00ff; payload[13] = (y_vel >> 8) & 0x00ff;
  checkSum = 0;
  for(int i = 0; i < 14; i++) checkSum += payload[i];
  checkSum = checkSum & 0x00ff;
}

unsigned char startDroneControl() {
   if(!digitalRead(9)) {
     firstRoll = analogRead(4);
     firstPitch = analogRead(5);
     drone_action = 1;
   }
   return drone_action;
}

void setup() {
  Serial.begin(9600); 
  bleSerial.begin(9600); 
  pinMode(13, OUTPUT); // ★ 13번 디버깅 LED 켜기 준비

  for(int i = 5; i < 11; i++) {
    pinMode(i, INPUT);
    digitalWrite(i, HIGH); 
  }
  delay(500);
}

void loop() { 
  if(startDroneControl()) {
    checkThrottle();
    checkRoll();
    checkPitch();
    checkYaw();
    checkEmergency();
    
    if (option == 0x000e) {
      ai_pitch = 0; ai_roll = 0; ai_throttle = 0; ai_command_timestamp = 0; 
    }
    
    // 조이스틱 미세 떨림으로 인한 오작동 방지를 위해 수동 조건 완화
    bool isManualOverride = (!digitalRead(5) || !digitalRead(6) || !digitalRead(7) || !digitalRead(8));
    if (roll > 50 || roll < -50 || pitch > 50 || pitch < -50) isManualOverride = true;

    // 🚀 훨씬 너그러워진 시리얼 수신부 (괄호 상관없이 F, S, U만 찾음)
    if (Serial.available() > 0) {
      char c = Serial.read();
      
      if (c == 'F' || c == 'S' || c == 'U') {
        ai_command_timestamp = millis(); 
        digitalWrite(13, HIGH); // ★ AI 명령 수신 성공 시 13번 LED 불 켬!
      }

      if (c == 'F') { ai_pitch = 80; ai_roll = 0; } 
      else if (c == 'S') { ai_pitch = 0; ai_roll = 0; }
      else if (c == 'U') { ai_pitch = 0; ai_roll = 0; ai_throttle = 100; }
    }

    if (isManualOverride) {
      ai_pitch = 0; ai_roll = 0; ai_throttle = 0;
      digitalWrite(13, LOW); // 수동 조작 시 LED 끄기
    } else {
      if (millis() - ai_command_timestamp > AI_COMMAND_TIMEOUT_MS) {
        ai_pitch = 0; ai_roll = 0;
        digitalWrite(13, LOW); // AI 명령 2초 만료 시 LED 끄기
      }
      pitch = ai_pitch;
      roll = ai_roll;
      
      // ★ 급발진 방지(Ramp-up): 스로틀을 한 번에 100으로 안 쏘고 서서히 올림
      if (ai_throttle > 0) {
        if (throttle < ai_throttle) throttle += 10; 
        else throttle = ai_throttle;
      }
    }

    checkCRC();
    sendDroneCommand();
  }
}
