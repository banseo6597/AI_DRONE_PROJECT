import os
import sys
import time
import select
import tty
import termios
import threading
import json
import pyaudio
import serial
from vosk import Model, KaldiRecognizer

# ==========================================
# ⚙️ [설정 파라미터] 환경에 맞게 수정하세요
# ==========================================
SERIAL_PORT = '/dev/ttyS0'  # 안 될 경우 '/dev/ttyAMA0' 으로 변경
BAUDRATE = 9600
MODEL_PATH = "model"       # Vosk 모델 폴더 경로

WAKE_WORDS = ["욘두", "연두", "용두", "윤두", "년두"]
CMD_FORWARD = ["전진", "앞으로", "앞으로 가", "출발"]
CMD_STOP = ["정지", "멈춰", "브레이크", "스톱"]
CMD_TAKEOFF = ["이륙", "날아라", "떠라", "업"]

# ==========================================
# 📊 [전역 제어 변수] 스레드 간 데이터 공유
# ==========================================
running = True
is_manual_override = False

# 드론 송신용 최종 제어 변수
roll = 0
pitch = 0
yaw = 0
throttle = 0
option = 0x000f  # 0x000f: 정상비행, 0x000e: 비상정지

# AI 명령 기억 캐시 및 Watchdog 타이머
ai_pitch = 0
ai_roll = 0
ai_throttle = 0
ai_command_timestamp = 0
AI_COMMAND_TIMEOUT = 2.0  # 2초 동안 명령 없으면 자동 정지

# 키보드 복원을 위한 세팅 저장 변수
old_settings = None

# 시리얼 포트 초기화
try:
    drone_serial = serial.Serial(SERIAL_PORT, baudrate=BAUDRATE, timeout=1)
    print(f"✅ 펌테크 블루투스 모듈과 다이렉트 연결 성공! ({SERIAL_PORT})")
except Exception as e:
    print(f"❌ 시리얼 포트 오픈 실패: {e}\n회로 배선 및 raspi-config 설정을 확인하세요.")
    sys.exit(1)


# ==========================================
# 📡 [엔진 1] 펌테크 전용 패킷 조립 및 송신
# ==========================================
def send_drone_packet(r, p, y, t, opt):
    """ 아두이노가 하던 역할을 파이썬문자열 헥사 변환으로 완벽 이식 """
    # 14바이트 페이로드 바이트 배열 조립 (Little Endian)
    payload = [
        r & 0xFF, (r >> 8) & 0xFF,
        p & 0xFF, (p >> 8) & 0xFF,
        y & 0xFF, (y >> 8) & 0xFF,
        t & 0xFF, (t >> 8) & 0xFF,
        opt & 0xFF, (opt >> 8) & 0xFF,
        0x64, 0x00,  # p_vel = 100
        0x64, 0x00   # y_vel = 100
    ]
    
    # 체크섬 계산 (페이로드 14바이트의 총합 & 0xFF)
    checksum = sum(payload) & 0xFF
    
    # 펌테크 AT 명령어 프로토콜 텍스트 조립
    packet_str = "at+writeh000d"
    packet_str += f"{0x26:02x}{0xa8:02x}{0x14:02x}{0xb1:02x}{0x14:02x}" # 헤더 및 스타트비트
    packet_str += f"{checksum:02x}"
    for b in payload:
        packet_str += f"{b:02x}"
    packet_str += "\r"
    
    # 드론으로 최종 전송
    drone_serial.write(packet_str.encode('ascii'))
    drone_serial.flush()


# ==========================================
# ⌨️ [스레드 1] 무선 키보드 비비동기 감시 (수동 오버라이드)
# ==========================================
def keyboard_listener():
    global running, is_manual_override, roll, pitch, yaw, throttle, option
    global ai_pitch, ai_roll, ai_throttle, ai_command_timestamp
    
    print("⌨️ [안전장치] 키보드 감시 스레드 가동 (Spacebar: 비상 정지 및 착륙)")
    
    while running:
        # 터미널 입력 버퍼에 데이터가 들어왔는지 확인 (Non-blocking)
        if select.select([sys.stdin], [], [], 0.1)[0]:
            key = sys.stdin.read(1)
            
            # 🚨 스페이스바 감지 시: 하이브리드 수동 오버라이드 무조건 발동
            if key == ' ':
                is_manual_override = True
                ai_pitch = 0; ai_roll = 0; ai_throttle = 0; ai_command_timestamp = 0
                
                # 즉시 드론을 정지 및 강제 셧다운 상태로 변경
                roll = 0; pitch = 0; yaw = 0; throttle = 0
                option = 0x000e  # 비상정지 헥사코드 강제 주입
                print("\n🚨 [MANUAL OVERRIDE] 스페이스바 입력 감지! 비상 정지 패킷을 송신합니다!")


# ==========================================
# 🎙️ [스레드 2] 오프라인 Vosk 음성 인식 처리
# ==========================================
def speech_recognition_thread():
    global running, ai_pitch, ai_roll, ai_throttle, ai_command_timestamp, is_manual_override
    
    if not os.path.exists(MODEL_PATH):
        print(f"❌ Vosk 모델 폴더('{MODEL_PATH}')를 찾을 수 없습니다.")
        running = False
        return

    model = Model(MODEL_PATH)
    rec = KaldiRecognizer(model, 16000)
    p = PyAudio()
    
    # 라즈베리파이 I2S 마이크 설정 입력
    stream = p.open(format=paInt16, channels=1, rate=16000, input=True, frames_per_buffer=8000)
    stream.start_stream()
    
    print("🎙️ [AI 엔진] 오프라인 한국어 음성인식 가동 준비 완료.")
    
    while running:
        if is_manual_override:
            # 수동 오버라이드 상태면 AI는 잠시 대기하며 연산 자원을 아낌
            time.sleep(0.5)
            continue
            
        data = stream.read(4000, exception_on_overflow=False)
        if len(data) == 0:
            continue
            
        if rec.AcceptWaveform(data):
            result = json.loads(rec.Result())
            text = result.get("text", "").replace(" ", "")
            
            if not text:
                continue
                
            print(f"\n📢 인식된 문장: {text}")
            
            # 호출어(Wake-word)가 포함되어 있는지 검사
            if not any(wake in text for wake in WAKE_WORDS):
                continue
                
            # 명령어 판별 및 캐시 갱신
            if any(word in text for word in CMD_FORWARD):
                print(">> 🚀 액션: 전진 명령 수신")
                ai_pitch = 80; ai_roll = 0
                ai_command_timestamp = time.time()
                
            elif any(word in text for word in CMD_STOP):
                print(">> 🛑 액션: 정지 명령 수신")
                ai_pitch = 0; ai_roll = 0
                ai_command_timestamp = time.time()
                
            elif any(word in text for word in CMD_TAKEOFF):
                print(">> 🚁 액션: 자율 이륙 및 호버링 가동")
                ai_pitch = 0; ai_roll = 0
                ai_throttle = 100  # 펌테크 드론 이륙 적정 출력 값
                ai_command_timestamp = time.time()
        else:
            # 주석을 해제하면 실시간 인식 과정을 볼 수 있습니다.
            # partial = json.loads(rec.PartialResult())
            # print(f"   (듣는 중... {partial['partial']:<40})", end='\r')
            pass


# ==========================================
# 🔄 [메인 스레드] 정밀 50ms 주기 드론 송신 루프
# ==========================================
def main():
    global running, old_settings, throttle, pitch, roll, option
    
    # 키보드 입력을 엔터 없이 즉시 받기 위해 터미널 모드 변경 (Raw mode 유사 구현)
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    
    # 백그라운드 보조 스레드들 출발
    t_kb = threading.Thread(target=keyboard_listener, daemon=True)
    t_speech = threading.Thread(target=speech_recognition_thread, daemon=True)
    t_kb.start()
    t_speech.start()
    
    print("\n🚀 [GCS 메인] 다이렉트 자율비행 시스템이 활성화되었습니다. 50ms 송신 시작.")
    print("👉 음성 제어를 하거나, 위급 시 [Spacebar]를 눌러 정지시키세요.\n")
    
    current_throttle = 0  # 부드러운 전압 램프업(Ramp-up)용 변수

    try:
        while running:
            loop_start = time.time()
            
            if is_manual_override:
                # 수동 오버라이드(스페이스바)가 켜지면 무조건 안전 패킷 고정
                pitch = 0; roll = 0; yaw = 0; throttle = 0; option = 0x000e
            else:
                # ⏱️ AI 명령어 Watchdog (타임아웃 백도어)
                if ai_command_timestamp > 0 and (time.time() - ai_command_timestamp > AI_COMMAND_TIMEOUT):
                    ai_pitch = 0
                    ai_roll = 0  # 2초간 통신 없으면 전진을 멈추고 제자리 호버링 유지
                
                # AI 명령 변수를 드론 송신 변수에 매핑
                pitch = ai_pitch
                roll = ai_roll
                option = 0x000f
                
                # 스로틀(고도) 급발진 방지 및 스무스 램프업 알고리즘
                if current_throttle < ai_throttle:
                    current_throttle += 2  # 50ms마다 출력을 2씩 안전하게 올림
                    if current_throttle > ai_throttle: current_throttle = ai_throttle
                elif current_throttle > ai_throttle:
                    current_throttle -= 4
                    if current_throttle < ai_throttle: current_throttle = ai_throttle
                
                throttle = current_throttle

            # 📡 드론으로 14바이트 물리 패킷 직접 방출
            send_drone_packet(roll, pitch, yaw, throttle, option)
            
            # 정확한 50ms(20Hz) 주기를 맞추기 위한 정밀 슬립 계산
            loop_time = time.time() - loop_start
            sleep_time = 0.05 - loop_time
            if sleep_time > 0:
                time.sleep(sleep_time)
                
    except KeyboardInterrupt:
        print("\n👋 사용자가 프로그램을 종료했습니다.")
    finally:
        # 종료 시 터미널 키보드 설정을 반드시 원상복구 (안 하면 리눅스 터미널이 망가짐)
        running = False
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("🔒 터미널 환경이 안전하게 복구되었습니다. 종료합니다.")

if __name__ == "__main__":
    main()
