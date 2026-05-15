# 🚁 Offline Edge AI Hybrid GCS for Drones
> **오프라인 엣지 AI 기반 하이브리드 제어 드론 지상 통제소 시스템** > 2026 대학 캡스톤 디자인 프로젝트

## 💡 Project Overview
본 프로젝트는 **인터넷 연결이 불가능한 오프라인 환경**에서도 음성으로 드론을 제어할 수 있는 지상 통제소(GCS)를 구축한 시스템입니다. 
단순한 음성 인식을 넘어, **AI 자율 비행 중 위급 상황 발생 시 조종자의 수동 조작이 즉각적으로 통제권을 뺏어오는 '하이브리드 제어(Manual Override)'**와 통신 두절 시 드론을 안전하게 호버링 시키는 **'Watchdog 타임아웃'** 안전망을 하드웨어 레벨에서 구현했습니다.

## 🏗️ System Architecture
1. **Edge AI Processing (Raspberry Pi 4)**
   - I2S 마이크(SPH0645)를 통한 디지털 음성 수집
   - Vosk 모델(`vosk-model-small-ko`)을 활용한 오프라인 음성 인식 (STT)
   - 호출어(Wake-word) 감지 및 명령 패킷 시리얼(UART) 송신
2. **Hybrid Controller (Arduino Uno)**
   - 파이썬 AI 명령과 조이스틱(수동) 입력 동시 수신
   - **권한 중재 로직:** 수동 조작 감지 시 AI 명령 즉시 무효화 (Fail-safe)
   - **명령 캐싱 및 Watchdog:** AI 명령 2초 타임아웃 시 자동 호버링 전환
3. **Actuator (Firmtech Drone)**
   - 블루투스(BLE) 14바이트 HEX 패킷 수신 및 모터(Roll, Pitch, Yaw, Throttle) 제어

## ✨ Key Features
- **Zero-Latency Offline AI:** 클라우드를 거치지 않아 지연 시간이 없고 보안이 뛰어남.
- **Robust Keyword Spotting:** 오인식 발음 및 유사어 사전 매핑을 통한 인식률 극대화.
- **State Retention:** 빠른 루프 속도(수백 Hz)로 인해 AI 명령이 증발하는 것을 막는 명령 캐싱 로직.
- **Safety First (Fail-Safe):** 비상정지 버튼 및 조이스틱 데드존 이탈 시 AI 통제권 강제 박탈.

## 🛠️ Hardware & Tech Stack
- **SBC:** Raspberry Pi 4 Model B
- **MCU:** Arduino Uno R3 + Joystick Shield
- **Sensor:** SPH0645LM4H (I2S Digital Microphone)
- **Drone:** Firmtech 아두이노 드론 2호 키트 + FB301 BLE 모듈
- **Software:** Python 3, Vosk, Arduino C++

## 🚀 Quick Start
### 1. Raspberry Pi Setup
```bash
sudo apt-get update
sudo apt-get install python3-pip
pip3 install vosk pyserial
python3 rpi_gcs/main.py
