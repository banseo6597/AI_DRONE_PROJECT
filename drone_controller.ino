import pyaudio

p = pyaudio.PyAudio()
print("\n=== 🕵️‍♂️ 마이크 자동 탐색 및 테스트 시작 ===")

found = False
for i in range(p.get_device_count()):
    dev = p.get_device_info_by_index(i)
    if dev['maxInputChannels'] > 0:
        print(f"▶ 장치 번호 {i}번 ({dev['name']}) 테스트 중...")
        for ch in [1, 2]:
            try:
                # 실제로 스트림을 열어보고 에러가 안 나면 성공!
                stream = p.open(format=pyaudio.paInt16, channels=ch, rate=48000, input=True, input_device_index=i)
                stream.close()
                print(f"  ✅ 빙고! 완벽한 세팅 발견: [ input_device_index={i}, channels={ch} ]")
                found = True
            except:
                pass

if not found:
    print("  ❌ 16-bit 형식으로 열리는 마이크가 없습니다. (32-bit I2S 포맷 충돌 의심)")
print("==========================================\n")
