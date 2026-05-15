// [전역 변수 추가] AI 명령 타임아웃을 위한 타이머 변수
unsigned long ai_command_timestamp = 0;
const unsigned long AI_COMMAND_TIMEOUT_MS = 2000; // 2초간 새 명령 없으면 자동 정지
int ai_throttle = 0; // 이륙(U) 명령용 캐시

// ... (중략: checkThrottle 등 기존 함수 유지) ...

void loop() { 
  if(startDroneControl()) {
    
    // 1. 조이스틱(수동) 입력값 및 비상버튼 확인
    checkThrottle();
    checkRoll();
    checkPitch();
    checkYaw();
    checkEmergency();
    
    // 🟠 High 3.2 패치: 비상정지 활성화 시 AI의 모든 기억을 강제로 날려버림!
    if (option == 0x000e) {
      ai_pitch = 0;
      ai_roll = 0;
      ai_throttle = 0;
      ai_command_timestamp = 0; 
    }

    // 2. 수동 조작 여부 판별
    bool isManualOverride = (roll != 0 || pitch != 0 || yaw != 0);

    // 3. 🔴 Critical 2.3 패치: 파이썬(AI) 프레이밍 명령 수신 (예: <F>)
    if (Serial.available() > 0) {
      char startChar = Serial.read();
      if (startChar == '<') {
        delay(5); // 시리얼 버퍼 대기
        char aiCommand = Serial.read();
        char endChar = Serial.read();

        if (endChar == '>') {
          ai_command_timestamp = millis(); // 정상 수신 시 타이머 리셋

          if (aiCommand == 'F') { 
            ai_pitch = 100; ai_roll = 0; ai_throttle = 0;
          } 
          else if (aiCommand == 'S') { 
            ai_pitch = 0; ai_roll = 0; ai_throttle = 0;
          }
          else if (aiCommand == 'U') { 
            ai_throttle = 80; // (임시) 자율 호버링으로 설정
          }
        }
      }
    }

    // 4. 🔴 Critical 2.1 패치: Watchdog & 권한 중재
    if (isManualOverride) {
      // 수동 조작 시 AI 기억 초기화
      ai_pitch = 0; ai_roll = 0; ai_throttle = 0;
    } else {
      // 자율 모드일 때, 마지막 명령을 받은 지 2초가 지났다면? -> 자동 호버링(안전)
      if (millis() - ai_command_timestamp > AI_COMMAND_TIMEOUT_MS) {
        ai_pitch = 0;
        ai_roll = 0;
        // ai_throttle은 고도 유지를 위해 0으로 깎지 않고 유지할지, 서서히 낮출지 결정 필요
      }
      
      // AI가 기억하는 값을 드론에 최종 반영
      pitch = ai_pitch;
      roll = ai_roll;
      if (ai_throttle > 0) throttle = ai_throttle; // U 명령이 있었을 경우 반영
    }

    checkCRC();
    sendDroneCommand();
  }
}
