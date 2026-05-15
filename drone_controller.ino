// [전역 변수 확인] 맨 위에 이 변수들이 있는지 꼭 확인해 주세요!
// unsigned long ai_command_timestamp = 0;
// const unsigned long AI_COMMAND_TIMEOUT_MS = 2000;
// int ai_pitch = 0;
// int ai_roll = 0;
// int ai_throttle = 0; 

// ... (setup 및 check 함수들 유지) ...

// ==========================================
void loop() { 
  if(startDroneControl()) {
    
    // 1. 조이스틱(수동) 입력값 및 비상버튼 확인
    checkThrottle();
    checkRoll();
    checkPitch();
    checkYaw();
    checkEmergency();
    
    // 2. 비상정지(SW25) 핀 작동 시 AI 캐시 강제 포맷 (안전 최우선)
    if (option == 0x000e) {
      ai_pitch = 0; ai_roll = 0; ai_throttle = 0; ai_command_timestamp = 0; 
    }

    // 3. 수동 오버라이드 조건: 방향키(조이스틱) 뿐만 아니라 '고도 버튼(5,6번)'을 눌러도 발동!
    bool isManualOverride = (roll != 0 || pitch != 0 || yaw != 0 || !digitalRead(5) || !digitalRead(6));

    // 4. 파이썬(AI) 명령 수신 (프레이밍 < > 적용)
    if (Serial.available() > 0) {
      char startChar = Serial.read();
      if (startChar == '<') {
        delay(5); // 시리얼 버퍼 대기
        char aiCommand = Serial.read();
        char endChar = Serial.read();

        if (endChar == '>') {
          ai_command_timestamp = millis(); // 명령 수신 성공 시 타임아웃 타이머 리셋

          if (aiCommand == 'F') { 
            ai_pitch = 80; ai_roll = 0; 
            // 전진 시 고도를 유지하기 위해 ai_throttle은 건드리지 않음
          } 
          else if (aiCommand == 'S') { 
            ai_pitch = 0; ai_roll = 0; 
          }
          else if (aiCommand == 'U') { 
            ai_pitch = 0; ai_roll = 0; // 이륙 시 기체 수평 유지
            ai_throttle = 80;          // ★ 자율 호버링을 위한 기본 스로틀 값 (필요시 70~100 사이로 튜닝)
          }
        }
      }
    }

    // 5. 권한 중재 및 Watchdog (타임아웃) 제어
    if (isManualOverride) {
      // 사람이 조종간이나 고도 버튼을 건드리면 AI의 모든 기억을 지우고 수동 모드 우선!
      ai_pitch = 0; ai_roll = 0; ai_throttle = 0;
    } else {
      // 2초간 파이썬에서 아무 명령이 없었다면? -> 자동 정지(Watchdog)
      if (millis() - ai_command_timestamp > AI_COMMAND_TIMEOUT_MS) {
        ai_pitch = 0;
        ai_roll = 0;
        // ★ 핵심: ai_throttle은 0으로 초기화하지 않음! (통신이 끊겨도 공중에서 제자리 호버링 유지)
      }
      
      // AI의 기억을 실제 드론 변수에 덮어쓰기
      pitch = ai_pitch;
      roll = ai_roll;
      if (ai_throttle > 0) {
        throttle = ai_throttle; 
      }
    }

    // 6. 패킷 검증 및 송신
    checkCRC();
    sendDroneCommand();
  }
}
