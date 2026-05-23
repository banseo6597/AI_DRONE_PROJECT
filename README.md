# 맞춤형 음성 인식 AI 드론 시스템 — 상세 구현 가이드

---

## 📁 전체 프로젝트 구조

```
drone-voice-control/
├── raspberry_pi/
│   ├── requirements.txt
│   ├── voice_controller.py       # 메인 음성 인식 + 시리얼 송신
│   └── models/
│       └── vosk-model-small-ko/  # 한국어 STT 모델 (다운로드 필요)
├── arduino_controller/
│   └── arduino_controller.ino   # 조종기 펌웨어 (조이스틱 + 시리얼 수신 + RF 송신)
└── arduino_drone/
    └── arduino_drone.ino        # 기체 펌웨어 (RF 수신 + 모터 + LED)
```

---

## STEP 1 — 하드웨어 연결

### 1-1. 라즈베리파이 5 ↔ MEMS I2S 마이크 (예: INMP441)

```
INMP441 핀  →  라즈베리파이 5 GPIO 핀
─────────────────────────────────────
VDD        →  3.3V  (Pin 1)
GND        →  GND   (Pin 6)
SD  (Data) →  GPIO20 (Pin 38, PCM_DIN)
SCK (BCLK) →  GPIO18 (Pin 12, PCM_CLK)
WS  (LRCK) →  GPIO19 (Pin 35, PCM_FS)
L/R        →  GND   (왼쪽 채널 선택)
```

> ⚠️ I2S를 활성화하려면 `/boot/config.txt`에 `dtparam=i2s=on` 추가 후 재부팅

### 1-2. 라즈베리파이 5 ↔ 아두이노 조종기

```
USB-A (라즈베리파이)  →  USB-B/마이크로USB (아두이노)
→ /dev/ttyUSB0 또는 /dev/ttyACM0 로 자동 인식됨
```

### 1-3. 아두이노 조종기 — 조이스틱 & RF 모듈 (nRF24L01)

```
조이스틱 모듈
  VRx  →  A0
  VRy  →  A1
  GND  →  GND
  VCC  →  5V

nRF24L01
  VCC  →  3.3V  ← 반드시 3.3V! 5V 연결 시 모듈 파손
  GND  →  GND
  CE   →  D9
  CSN  →  D10
  SCK  →  D13
  MOSI →  D11
  MISO →  D12
```

### 1-4. 아두이노 드론 기체 — RF 수신 & LED & 모터

```
nRF24L01  →  동일하게 연결 (CE=D9, CSN=D10)
LED (+)   →  D7 (330Ω 저항 직렬)
LED (-)   →  GND
모터 ESC 신호선 → D3, D5, D6, D9 (PWM 핀)
```

---

## STEP 2 — 라즈베리파이 환경 설정

### 2-1. OS 및 기본 패키지 설치

```bash
sudo apt update && sudo apt upgrade -y

# Python 및 오디오 라이브러리
sudo apt install -y python3-pip python3-venv portaudio19-dev \
    libatlas-base-dev libasound2-dev git

# 가상환경 생성
python3 -m venv ~/drone-env
source ~/drone-env/bin/activate
```

### 2-2. I2S 마이크 커널 모듈 설정

```bash
# /boot/config.txt 편집
sudo nano /boot/config.txt

# 아래 두 줄 추가
dtparam=i2s=on
dtoverlay=i2s-mmap

# ALSA 설정 파일 생성
sudo nano /etc/asound.conf
```

`/etc/asound.conf` 내용:

```
pcm.i2smic {
    type hw
    card sndrpii2scard
}
pcm.!default {
    type asym
    capture.pcm "i2smic"
}
```

```bash
sudo reboot
# 재부팅 후 마이크 인식 확인
arecord -l
```

### 2-3. Python 패키지 설치

```bash
source ~/drone-env/bin/activate
pip install vosk pyaudio pyserial numpy
```

### 2-4. 한국어 Vosk 모델 다운로드

```bash
mkdir -p ~/drone-voice-control/raspberry_pi/models
cd ~/drone-voice-control/raspberry_pi/models

# 경량 한국어 모델 (약 40MB)
wget https://alphacephei.com/vosk/models/vosk-model-small-ko-0.22.zip
unzip vosk-model-small-ko-0.22.zip
mv vosk-model-small-ko-0.22 vosk-model-small-ko
```

---

## STEP 3 — 라즈베리파이 메인 Python 코드

### `raspberry_pi/voice_controller.py`

```python
#!/usr/bin/env python3
"""
맞춤형 음성 인식 드론 제어 시스템
- Wake Word: "욘두"
- STT: Vosk (오프라인 한국어)
- 통신: USB 시리얼 → 아두이노
"""

import sys
import os
import json
import queue
import threading
import time
import serial
import serial.tools.list_ports
import vosk
import pyaudio

# ─── 설정값 ────────────────────────────────────────────────────────────────
MODEL_PATH   = "./models/vosk-model-small-ko"
SAMPLE_RATE  = 16000          # Vosk 권장 샘플레이트
CHUNK_SIZE   = 4096           # 오디오 청크 크기 (클수록 딜레이 증가)
BAUD_RATE    = 115200         # 시리얼 통신 속도 (아두이노와 동일 설정)
WAKE_WORD    = "욘두"          # 호출어
CMD_TIMEOUT  = 3.0            # 호출어 인식 후 명령 대기 시간(초)

# ─── 명령어 매핑 테이블 ──────────────────────────────────────────────────────
# 텍스트 키워드 → 아두이노로 전송할 1바이트 명령 코드
COMMAND_MAP = {
    # 이동 명령
    "앞으로":  b'F',   # Forward
    "전진":    b'F',
    "뒤로":    b'B',   # Backward
    "후진":    b'B',
    "왼쪽":    b'L',   # Left
    "오른쪽":  b'R',   # Right
    "위로":    b'U',   # Up
    "올라가":  b'U',
    "아래로":  b'D',   # Down
    "내려가":  b'D',

    # 이륙 / 착륙
    "이륙":    b'T',   # Take-off
    "출발":    b'T',
    "착륙":    b'G',   # Ground (landing)
    "내려":    b'G',

    # 정지
    "정지":    b'S',   # Stop / Hover
    "멈춰":    b'S',
    "스탑":    b'S',

    # LED 제어
    "불 켜":   b'O',   # Light On
    "불켜":    b'O',
    "불 꺼":   b'X',   # Light Off  (eXtinguish)
    "불꺼":    b'X',
}

# ─── 전역 상태 ────────────────────────────────────────────────────────────
audio_queue   = queue.Queue()
is_activated  = False          # 호출어 인식 후 활성화 상태
activate_time = 0.0            # 활성화된 시각 (타임아웃 계산용)


# ══════════════════════════════════════════════════════════════════════════════
# 시리얼 포트 자동 탐색
# ══════════════════════════════════════════════════════════════════════════════
def find_arduino_port() -> str:
    """연결된 아두이노의 시리얼 포트를 자동으로 찾습니다."""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        # 아두이노는 보통 'Arduino', 'CH340', 'CP210x' 등의 설명을 가짐
        if any(keyword in (port.description or "") for keyword in
               ["Arduino", "CH340", "CP210", "ttyACM", "ttyUSB"]):
            print(f"[시리얼] 아두이노 감지: {port.device} ({port.description})")
            return port.device

    # 자동 탐지 실패 시 기본값
    fallback = "/dev/ttyACM0"
    print(f"[시리얼] 자동 탐지 실패 → 기본값 사용: {fallback}")
    return fallback


# ══════════════════════════════════════════════════════════════════════════════
# 명령어 파싱
# ══════════════════════════════════════════════════════════════════════════════
def parse_commands(text: str) -> list[bytes]:
    """
    인식된 텍스트에서 호출어와 명령어를 추출합니다.

    반환값:
      - None  : 호출어 없음 (무시)
      - []    : 호출어만 있고 명령어 없음 (활성화만)
      - [cmd] : 명령어 리스트
    """
    global is_activated, activate_time

    print(f"[STT] 인식: '{text}'")

    # ① 호출어 확인
    if WAKE_WORD in text:
        is_activated  = True
        activate_time = time.time()
        print(f"[시스템] 🟢 '{WAKE_WORD}' 호출어 감지 → 활성화 (타임아웃 {CMD_TIMEOUT}초)")
        text = text.replace(WAKE_WORD, "").strip()  # 호출어 제거 후 명령 추출

    elif not is_activated:
        return None  # 호출어 없으면 완전히 무시

    # ② 타임아웃 확인
    if time.time() - activate_time > CMD_TIMEOUT:
        is_activated = False
        print("[시스템] 🔴 타임아웃 → 비활성화")
        return None

    # ③ 명령어 추출 (여러 명령 동시 지원)
    commands = []
    for keyword, code in COMMAND_MAP.items():
        if keyword in text:
            commands.append(code)
            print(f"[파싱] 명령 감지: '{keyword}' → {code}")

    return commands


# ══════════════════════════════════════════════════════════════════════════════
# 오디오 수집 스레드
# ══════════════════════════════════════════════════════════════════════════════
def audio_capture_thread():
    """마이크에서 오디오를 읽어 큐에 넣는 백그라운드 스레드."""
    pa = pyaudio.PyAudio()

    # I2S 마이크 장치 인덱스 찾기
    device_index = None
    for i in range(pa.get_device_count()):
        info = pa.get_device_info_by_index(i)
        if info["maxInputChannels"] > 0:
            print(f"[오디오] 장치 {i}: {info['name']}")
            if "i2s" in info["name"].lower() or "sndrpi" in info["name"].lower():
                device_index = i
                print(f"[오디오] I2S 마이크 선택: {i}")
                break

    stream = pa.open(
        format=pyaudio.paInt16,
        channels=1,
        rate=SAMPLE_RATE,
        input=True,
        input_device_index=device_index,  # None이면 기본 장치 사용
        frames_per_buffer=CHUNK_SIZE,
    )

    print("[오디오] 🎤 마이크 스트리밍 시작")
    while True:
        try:
            data = stream.read(CHUNK_SIZE, exception_on_overflow=False)
            audio_queue.put(data)
        except Exception as e:
            print(f"[오디오] 오류: {e}")
            break


# ══════════════════════════════════════════════════════════════════════════════
# 메인 루프
# ══════════════════════════════════════════════════════════════════════════════
def main():
    # 1) Vosk 모델 로드
    if not os.path.exists(MODEL_PATH):
        print(f"[오류] Vosk 모델을 찾을 수 없습니다: {MODEL_PATH}")
        print("       README의 모델 다운로드 절차를 따라주세요.")
        sys.exit(1)

    print("[Vosk] 한국어 모델 로딩 중...")
    model      = vosk.Model(MODEL_PATH)
    recognizer = vosk.KaldiRecognizer(model, SAMPLE_RATE)
    recognizer.SetWords(True)   # 단어별 신뢰도 점수 포함
    print("[Vosk] ✅ 모델 로드 완료")

    # 2) 시리얼 포트 연결
    port = find_arduino_port()
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
        time.sleep(2)  # 아두이노 리셋 대기
        print(f"[시리얼] ✅ 연결 성공: {port} @ {BAUD_RATE}bps")
    except serial.SerialException as e:
        print(f"[시리얼] ❌ 연결 실패: {e}")
        sys.exit(1)

    # 3) 오디오 캡처 스레드 시작
    t = threading.Thread(target=audio_capture_thread, daemon=True)
    t.start()

    print("\n" + "="*50)
    print(f"  드론 음성 제어 시스템 가동")
    print(f"  호출어: '{WAKE_WORD}'  |  타임아웃: {CMD_TIMEOUT}초")
    print("="*50 + "\n")

    # 4) STT 인식 + 명령 전송 루프
    try:
        while True:
            data = audio_queue.get()

            # Vosk에 오디오 청크 전달
            if recognizer.AcceptWaveform(data):
                # 문장 완성 시
                result = json.loads(recognizer.Result())
                text   = result.get("text", "").strip()
            else:
                # 실시간 부분 인식 (빠른 응답을 위해 partial도 처리)
                partial = json.loads(recognizer.PartialResult())
                text    = partial.get("partial", "").strip()

            if not text:
                continue

            # 명령어 파싱
            commands = parse_commands(text)
            if not commands:
                continue

            # 시리얼로 명령 전송
            for cmd in commands:
                ser.write(cmd)
                print(f"[시리얼] 📤 전송: {cmd}")
                time.sleep(0.05)   # 명령 간 짧은 딜레이

    except KeyboardInterrupt:
        print("\n[시스템] 종료합니다.")
        ser.close()


if __name__ == "__main__":
    main()
```

---

## STEP 4 — 아두이노 조종기 펌웨어

### `arduino_controller/arduino_controller.ino`

```cpp
/**
 * 아두이노 조종기 펌웨어
 * ─ 역할: 시리얼(라즈베리파이) + 조이스틱 입력을 처리하여
 *         nRF24L01을 통해 드론 기체로 패킷 전송
 *
 * 라이브러리 설치 (Arduino IDE → 라이브러리 관리):
 *   - RF24 by TMRh20
 */

#include <SPI.h>
#include <RF24.h>

// ── RF24 핀 설정 ─────────────────────────────────────────────────────────────
#define CE_PIN   9
#define CSN_PIN  10
RF24 radio(CE_PIN, CSN_PIN);
const byte PIPE_ADDRESS[6] = "DRONE";  // 송수신 주소 (드론과 동일해야 함)

// ── 조이스틱 핀 ──────────────────────────────────────────────────────────────
#define JOY_X_PIN  A0   // 좌우 (Roll / Yaw)
#define JOY_Y_PIN  A1   // 앞뒤 (Pitch)

// ── 데드존 범위 ──────────────────────────────────────────────────────────────
// 조이스틱 중립 값(약 512) 근처 ±60을 데드존으로 설정
#define DEADZONE_LOW   450
#define DEADZONE_HIGH  570

// ── 직렬 통신 ────────────────────────────────────────────────────────────────
#define SERIAL_BAUD  115200

// ── 드론 명령 구조체 ─────────────────────────────────────────────────────────
struct DronePacket {
    int16_t  throttle;    // 고도 (-512 ~ 512, 0=호버)
    int16_t  pitch;       // 앞뒤 (-512 ~ 512)
    int16_t  roll;        // 좌우 (-512 ~ 512)
    int16_t  yaw;         // 회전 (-512 ~ 512)
    uint8_t  led;         // LED 상태 (0=끄기, 1=켜기)
    uint8_t  land;        // 착륙 플래그
};

DronePacket packet = {0, 0, 0, 0, 0, 0};

// ── 상태 변수 ────────────────────────────────────────────────────────────────
bool  manualMode     = false;   // true=수동 조종, false=AI 음성 제어
char  voiceCmd       = 0;       // 현재 음성 명령
unsigned long cmdTimestamp = 0; // 마지막 음성 명령 수신 시각
#define CMD_HOLD_MS  500        // 명령 유지 시간 (ms)

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(SERIAL_BAUD);
    Serial.println("[조종기] 초기화 시작");

    // nRF24L01 초기화
    if (!radio.begin()) {
        Serial.println("[오류] nRF24L01 초기화 실패!");
        while (1);
    }
    radio.setPALevel(RF24_PA_LOW);          // 실내 테스트: LOW / 야외: HIGH
    radio.setDataRate(RF24_250KBPS);        // 저속 = 통신 안정성 향상
    radio.setChannel(76);                   // 채널 (드론과 동일)
    radio.openWritingPipe(PIPE_ADDRESS);
    radio.stopListening();                  // 송신 전용 모드

    Serial.println("[조종기] ✅ 준비 완료 — 음성 명령 대기 중");
}

// ─────────────────────────────────────────────────────────────────────────────
// 조이스틱 값이 데드존 안에 있는지 확인
bool isInDeadzone(int val) {
    return (val >= DEADZONE_LOW && val <= DEADZONE_HIGH);
}

// 조이스틱 raw 값 → -512 ~ 512 정규화
int16_t normalizeJoy(int raw) {
    int centered = raw - 512;              // 중앙값(512) 기준으로 이동
    if (centered > -60 && centered < 60)   // 데드존 내부는 0 처리
        return 0;
    return (int16_t)constrain(centered, -512, 512);
}

// ─────────────────────────────────────────────────────────────────────────────
// 음성 명령 코드를 패킷 값으로 변환
void applyVoiceCommand(char cmd) {
    // 이전 이동 명령 초기화 (LED/착륙 명령은 유지)
    packet.pitch    = 0;
    packet.roll     = 0;
    packet.throttle = 0;
    packet.yaw      = 0;

    switch (cmd) {
        case 'F': packet.pitch    =  300;  break;  // 전진
        case 'B': packet.pitch    = -300;  break;  // 후진
        case 'L': packet.roll     = -300;  break;  // 왼쪽
        case 'R': packet.roll     =  300;  break;  // 오른쪽
        case 'U': packet.throttle =  300;  break;  // 상승
        case 'D': packet.throttle = -300;  break;  // 하강
        case 'T': packet.throttle =  400;  break;  // 이륙 (더 강한 상승)
        case 'G': packet.land     =  1;    break;  // 착륙
        case 'S':                          break;  // 정지 (이미 0으로 초기화됨)
        case 'O': packet.led      =  1;    break;  // LED 켜기
        case 'X': packet.led      =  0;    break;  // LED 끄기
    }
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    // ── 1) 시리얼 수신 (라즈베리파이 → 아두이노) ──────────────────────────────
    if (Serial.available() > 0) {
        char incoming = (char)Serial.read();

        // 유효한 명령인지 확인
        const char validCmds[] = "FBLRUDTSGOX";
        bool isValid = false;
        for (int i = 0; validCmds[i] != '\0'; i++) {
            if (incoming == validCmds[i]) { isValid = true; break; }
        }

        if (isValid) {
            voiceCmd      = incoming;
            cmdTimestamp  = millis();
            Serial.print("[수신] 음성 명령: ");
            Serial.println(incoming);
        }
    }

    // ── 2) 조이스틱 읽기 ──────────────────────────────────────────────────────
    int rawX = analogRead(JOY_X_PIN);
    int rawY = analogRead(JOY_Y_PIN);

    // ── 3) 제어권 판별 (핵심 안전 로직) ──────────────────────────────────────
    bool joyMoved = !isInDeadzone(rawX) || !isInDeadzone(rawY);

    if (joyMoved) {
        // 조이스틱 움직임 감지 → 즉시 수동 모드 전환
        if (!manualMode) {
            manualMode = true;
            Serial.println("[안전] ⚠️ 조이스틱 입력 감지 → 수동 모드 전환");
        }
    } else {
        // 조이스틱이 중앙으로 돌아옴 → AI 모드로 복귀
        if (manualMode) {
            manualMode = false;
            Serial.println("[안전] ✅ 조이스틱 중립 → AI 모드 복귀");
        }
    }

    // ── 4) 패킷 구성 ─────────────────────────────────────────────────────────
    if (manualMode) {
        // === 수동 조종 모드: 조이스틱 값 그대로 사용 ===
        packet.pitch    = normalizeJoy(rawY);
        packet.roll     = normalizeJoy(rawX);
        packet.throttle = 0;   // 스로틀은 별도 조이스틱 추가 시 확장
        packet.yaw      = 0;
        // LED / land 상태는 음성 명령 마지막 값 유지
    } else {
        // === AI 음성 제어 모드 ===
        // 명령 유지 시간이 지나면 정지
        if (voiceCmd != 0 && (millis() - cmdTimestamp > CMD_HOLD_MS)) {
            voiceCmd       = 'S';  // 자동 정지
            packet.pitch    = 0;
            packet.roll     = 0;
            packet.throttle = 0;
        } else if (voiceCmd != 0) {
            applyVoiceCommand(voiceCmd);
        }
    }

    // ── 5) RF24 패킷 전송 ────────────────────────────────────────────────────
    bool sent = radio.write(&packet, sizeof(DronePacket));

    if (!sent) {
        Serial.println("[RF] ❌ 전송 실패 (드론 범위 밖?)");
    }

    delay(20);  // 50Hz 전송 주기
}
```

---

## STEP 5 — 아두이노 드론 기체 펌웨어

### `arduino_drone/arduino_drone.ino`

```cpp
/**
 * 아두이노 드론 기체 펌웨어
 * ─ 역할: nRF24L01로 패킷 수신 → ESC/모터 제어 + LED 제어
 *
 * ※ 실제 쿼드콥터 제작 시 PID 제어기 추가 필요 (아래 주석 참고)
 *    이 코드는 모터 출력 직접 제어 기반의 단순화 버전입니다.
 */

#include <SPI.h>
#include <RF24.h>
#include <Servo.h>   // ESC 제어에 Servo 라이브러리 사용 (PWM 신호 출력)

// ── 핀 설정 ──────────────────────────────────────────────────────────────────
#define CE_PIN   9
#define CSN_PIN  10
#define LED_PIN  7

// 모터 ESC 핀 (쿼드콥터: 앞좌, 앞우, 뒤좌, 뒤우)
#define MOTOR_FL 3    // Front-Left
#define MOTOR_FR 5    // Front-Right
#define MOTOR_BL 6    // Back-Left
#define MOTOR_BR 11   // Back-Right

RF24 radio(CE_PIN, CSN_PIN);
const byte PIPE_ADDRESS[6] = "DRONE";

Servo escFL, escFR, escBL, escBR;

// ── 패킷 구조체 (조종기와 동일) ──────────────────────────────────────────────
struct DronePacket {
    int16_t  throttle;
    int16_t  pitch;
    int16_t  roll;
    int16_t  yaw;
    uint8_t  led;
    uint8_t  land;
};

DronePacket packet;

// ── ESC 설정값 ───────────────────────────────────────────────────────────────
#define ESC_MIN       1000   // 최소 PWM (μs) — 모터 정지
#define ESC_MAX       2000   // 최대 PWM (μs) — 최대 출력
#define ESC_HOVER     1350   // 호버링 기준값 (기체 무게에 따라 조정 필요)
#define FAILSAFE_TIME 500    // 이 시간(ms) 동안 패킷 없으면 비상 착륙

unsigned long lastPacketTime = 0;
bool isFlying = false;

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    // LED 초기화
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // ESC 초기화
    escFL.attach(MOTOR_FL, ESC_MIN, ESC_MAX);
    escFR.attach(MOTOR_FR, ESC_MIN, ESC_MAX);
    escBL.attach(MOTOR_BL, ESC_MIN, ESC_MAX);
    escBR.attach(MOTOR_BR, ESC_MIN, ESC_MAX);

    // ESC 캘리브레이션 신호 (최솟값 전송)
    setAllMotors(ESC_MIN);
    delay(2000);
    Serial.println("[드론] ESC 초기화 완료");

    // nRF24L01 수신 설정
    if (!radio.begin()) {
        Serial.println("[오류] RF24 초기화 실패!");
        while (1);
    }
    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(76);
    radio.openReadingPipe(1, PIPE_ADDRESS);
    radio.startListening();   // 수신 모드

    Serial.println("[드론] ✅ 수신 대기 중");
}

// ─────────────────────────────────────────────────────────────────────────────
void setAllMotors(int value) {
    value = constrain(value, ESC_MIN, ESC_MAX);
    escFL.writeMicroseconds(value);
    escFR.writeMicroseconds(value);
    escBL.writeMicroseconds(value);
    escBR.writeMicroseconds(value);
}

// ─────────────────────────────────────────────────────────────────────────────
/**
 * 믹싱 함수: throttle/pitch/roll/yaw → 4개 모터 개별 출력
 *
 *   쿼드콥터 X 배열 기준:
 *     FL(+)  FR(-)      앞
 *     BL(-)  BR(+)      뒤
 *
 *   [실제 적용 시 PID 제어기 삽입 위치]
 *   현재는 단순 피드포워드(명령 → 출력) 구조.
 *   자이로센서(MPU-6050) + PID 루프 추가 시 안정적인 호버링 가능.
 */
void mixMotors(int16_t thr, int16_t pit, int16_t rol, int16_t yaw_) {
    // 입력 스케일: -512~512 → -200~200 μs 범위로 축소 (기체에 맞게 조정)
    float scale = 200.0f / 512.0f;

    int base = ESC_HOVER + (int)(thr * scale);

    int fl = base + (int)(pit * scale) - (int)(rol * scale) + (int)(yaw_ * scale);
    int fr = base + (int)(pit * scale) + (int)(rol * scale) - (int)(yaw_ * scale);
    int bl = base - (int)(pit * scale) - (int)(rol * scale) - (int)(yaw_ * scale);
    int br = base - (int)(pit * scale) + (int)(rol * scale) + (int)(yaw_ * scale);

    escFL.writeMicroseconds(constrain(fl, ESC_MIN, ESC_MAX));
    escFR.writeMicroseconds(constrain(fr, ESC_MIN, ESC_MAX));
    escBL.writeMicroseconds(constrain(bl, ESC_MIN, ESC_MAX));
    escBR.writeMicroseconds(constrain(br, ESC_MIN, ESC_MAX));
}

// ─────────────────────────────────────────────────────────────────────────────
// 점진적 착륙
void performLanding() {
    Serial.println("[드론] 🛬 착륙 시작");
    for (int val = ESC_HOVER; val >= ESC_MIN; val -= 5) {
        setAllMotors(val);
        delay(30);
    }
    isFlying = false;
    Serial.println("[드론] ✅ 착륙 완료");
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    // ── 1) 페일세이프 확인 ──────────────────────────────────────────────────
    if (isFlying && (millis() - lastPacketTime > FAILSAFE_TIME)) {
        Serial.println("[안전] ⚠️ 신호 두절 → 비상 착륙");
        performLanding();
        return;
    }

    // ── 2) RF24 패킷 수신 ───────────────────────────────────────────────────
    if (!radio.available()) return;

    radio.read(&packet, sizeof(DronePacket));
    lastPacketTime = millis();

    // ── 3) LED 제어 ─────────────────────────────────────────────────────────
    digitalWrite(LED_PIN, packet.led ? HIGH : LOW);

    // ── 4) 착륙 명령 ────────────────────────────────────────────────────────
    if (packet.land) {
        performLanding();
        return;
    }

    // ── 5) 이륙 감지 ────────────────────────────────────────────────────────
    if (!isFlying && packet.throttle > 100) {
        isFlying = true;
        Serial.println("[드론] 🚁 이륙!");
    }

    // ── 6) 모터 믹싱 및 출력 ────────────────────────────────────────────────
    if (isFlying) {
        mixMotors(packet.throttle, packet.pitch, packet.roll, packet.yaw);
    } else {
        setAllMotors(ESC_MIN);  // 착지 상태에서는 모터 정지
    }

    // 디버그 출력 (필요 시 주석 해제)
    /*
    Serial.print("THR:"); Serial.print(packet.throttle);
    Serial.print(" PIT:"); Serial.print(packet.pitch);
    Serial.print(" ROL:"); Serial.print(packet.roll);
    Serial.print(" LED:"); Serial.println(packet.led);
    */
}
```

---

## STEP 6 — 시스템 통합 테스트 절차

```
① ESC 캘리브레이션 (최초 1회)
   드론 기체 아두이노 업로드 → USB 연결 상태에서 배터리 OFF로
   시리얼 모니터 열어 "ESC 초기화 완료" 메시지 확인

② RF 통신 테스트
   조종기 아두이노를 PC에 연결, 시리얼 모니터(115200) 열기
   드론 기체에 배터리 연결
   조종기에서 "전송 성공" 로그 확인

③ 음성 인식 단독 테스트 (드론 없이)
   $ source ~/drone-env/bin/activate
   $ cd ~/drone-voice-control/raspberry_pi
   $ python voice_controller.py
   → "욘두, 이륙" 발화 후 시리얼 로그 확인

④ 통합 테스트 (실내, 낮은 고도)
   1. 드론 기체 배터리 연결
   2. 아두이노 조종기 PC/파워뱅크 연결
   3. 라즈베리파이 스크립트 실행
   4. "욘두, 이륙" → LED 점등 + 이륙 확인
   5. "욘두, 착륙" → 점진적 착륙 확인
   6. 조이스틱 건드려 수동 모드 전환 확인
```

---

## STEP 7 — 라즈베리파이 자동 시작 설정 (부팅 시 자동 실행)

```bash
# systemd 서비스 파일 생성
sudo nano /etc/systemd/system/drone-voice.service
```

```ini
[Unit]
Description=Drone Voice Control
After=network.target

[Service]
User=pi
WorkingDirectory=/home/pi/drone-voice-control/raspberry_pi
ExecStart=/home/pi/drone-env/bin/python voice_controller.py
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable drone-voice.service
sudo systemctl start  drone-voice.service
sudo journalctl -u drone-voice.service -f  # 로그 실시간 확인
```

---

## ⚡ 주요 트러블슈팅

| 증상 | 원인 | 해결 |
|------|------|------|
| `No module named 'vosk'` | 가상환경 미활성화 | `source ~/drone-env/bin/activate` |
| I2S 마이크 미인식 | config.txt 미수정 | `dtparam=i2s=on` 추가 후 재부팅 |
| RF24 전송 실패 | 3.3V 미연결 또는 채널 불일치 | 전압 확인, 채널 76 맞추기 |
| 음성 인식 오류 | 모터 소음 간섭 | MEMS 마이크 입 가까이 위치 |
| 모터 한쪽만 돌거나 떨림 | ESC 캘리브레이션 미완료 | ESC_MIN 신호 2초 전송 후 재시도 |
