import pyaudio

p = pyaudio.PyAudio()
print("\n=== 🎙️ 녹음 가능한 진짜 마이크 목록 ===")
for i in range(p.get_device_count()):
    dev = p.get_device_info_by_index(i)
    if dev['maxInputChannels'] > 0:  # 입력 채널이 0보다 큰 기기(마이크)만 출력
        print(f"번호 {i} : {dev['name']} (허용 채널 수: {dev['maxInputChannels']})")
print("======================================\n")
