import numpy as np
import wave
import serial
import serial.tools.list_ports

devices = serial.tools.list_ports.comports()
for device in devices:
    print(device)

ser = serial.Serial("COM6", 115200, timeout=2)

SAMPLE_RATE = 5000
RECORD_SECONDS = 10
num_samples = SAMPLE_RATE * RECORD_SECONDS

data = []

for i in range(num_samples):
    byte = ser.read(1)

    if byte == b"":
        print("No data received. Stopping early.")
        break

    value = byte[0]
    data.append(value)
    print(value)

ser.close()

data = np.array(data)

if len(data) == 0:
    print("No samples recorded.")
    exit()

data_min = data.min()
data_max = data.max()

if data_max > data_min:
    data = (data - data_min) / (data_max - data_min)
    data = data * 255

data = data.astype(np.uint8)

with wave.open("audio.wav", "wb") as wf:
    wf.setnchannels(1)
    wf.setsampwidth(1)
    wf.setframerate(SAMPLE_RATE)
    wf.writeframes(data.tobytes())

print("audio.wav saved successfully")