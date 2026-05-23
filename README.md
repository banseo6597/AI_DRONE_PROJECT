import pyaudio
p = pyaudio.PyAudio()
for i in range(p.get_device_count()):
    print(f"번호 {i} : {p.get_device_info_by_index(i)['name']}")
