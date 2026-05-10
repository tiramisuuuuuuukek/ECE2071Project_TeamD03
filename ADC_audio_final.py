import serial
import serial.tools.list_ports
import numpy as np
import wave
import csv
import matplotlib.pyplot as plt
from datetime import datetime
import time


# =========================
# Settings for current STM setup
# =========================

PORT = "COM6"           # Processing STM COM port
BAUD_RATE = 921600      # Must match Processing STM USART2

# This is only used to decide how many samples to collect.
# Your STM may not actually output exactly this rate.
EXPECTED_SAMPLE_RATE = 22050

RECORD_SECONDS = 20
TEAM_ID = "TeamID"


# =========================
# Helper functions
# =========================

def list_serial_ports():
    print("Available serial ports:")
    ports = serial.tools.list_ports.comports()

    if not ports:
        print("No serial ports found.")

    for port in ports:
        print(f"  {port.device}: {port.description}")


def record_audio(port, baud_rate, expected_sample_rate, duration):
    total_samples = expected_sample_rate * duration
    data = bytearray()

    print(f"\nOpening {port} at {baud_rate} baud...")
    print(f"Target recording: {duration} seconds at expected {expected_sample_rate} samples/second")
    print(f"Expected samples to collect: {total_samples}")

    with serial.Serial(port, baud_rate, timeout=2) as ser:
        ser.reset_input_buffer()

        start_time = time.perf_counter()

        while len(data) < total_samples:
            remaining = total_samples - len(data)

            # Read in chunks for speed
            chunk = ser.read(min(1024, remaining))

            if chunk == b"":
                print("\nTimeout: no data received.")
                print("Check COM port, STM reset order, baud rate, and wiring.")
                break

            data.extend(chunk)

            progress = 100 * len(data) / total_samples
            print(f"\rReceived {len(data)}/{total_samples} samples ({progress:.1f}%)", end="")

        end_time = time.perf_counter()

    actual_duration = end_time - start_time

    if actual_duration > 0:
        actual_rate = len(data) / actual_duration
    else:
        actual_rate = expected_sample_rate

    print("\nFinished reading.")
    print(f"Actual recording time: {actual_duration:.2f} s")
    print(f"Actual received sample rate: {actual_rate:.1f} samples/s")

    return np.frombuffer(data, dtype=np.uint8), actual_duration, actual_rate


def save_wav(filename, data, sample_rate):
    sample_rate = int(round(sample_rate))

    with wave.open(filename, "wb") as wf:
        wf.setnchannels(1)       # mono
        wf.setsampwidth(1)       # 8-bit unsigned PCM
        wf.setframerate(sample_rate)
        wf.writeframes(data.tobytes())

    print(f"Saved WAV: {filename} at {sample_rate} Hz")


def save_csv(filename, data, sample_rate):
    sample_rate = int(round(sample_rate))
    time_axis = np.arange(len(data)) / sample_rate

    with open(filename, "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["sample_rate", sample_rate])
        writer.writerow(["time_s", "amplitude"])

        for t, value in zip(time_axis, data):
            writer.writerow([t, int(value)])

    print(f"Saved CSV: {filename}")


def save_plot(filename, data, sample_rate):
    sample_rate = int(round(sample_rate))
    time_axis = np.arange(len(data)) / sample_rate

    plt.figure(figsize=(12, 4))
    plt.plot(time_axis, data, linewidth=0.5)
    plt.title(f"Audio Waveform, {sample_rate} Hz")
    plt.xlabel("Time (s)")
    plt.ylabel("Amplitude, 8-bit unsigned")
    plt.tight_layout()
    plt.savefig(filename, dpi=300)
    plt.close()

    print(f"Saved PNG: {filename}")


def save_raw(filename, data):
    with open(filename, "wb") as f:
        f.write(data.tobytes())

    print(f"Saved raw data: {filename}")


def save_same_raw_at_multiple_rates(data, rates):
    """
    This is useful for testing playback rate only.
    It saves the exact same recorded bytes at different WAV sample rates.
    """
    for rate in rates:
        filename = f"same_raw_{rate}Hz.wav"
        save_wav(filename, data, rate)


def main():
    list_serial_ports()

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    data, actual_duration, actual_rate = record_audio(
        PORT,
        BAUD_RATE,
        EXPECTED_SAMPLE_RATE,
        RECORD_SECONDS
    )

    if len(data) == 0:
        print("No samples recorded. No files saved.")
        return

    measured_rate = int(round(actual_rate))

    base_name = f"{TEAM_ID}_task3_measured_{measured_rate}Hz_{timestamp}"

    save_raw("raw_ADC_values.data", data)

    print(f"\nActual samples recorded: {len(data)}")
    print(f"Min value: {data.min()}")
    print(f"Max value: {data.max()}")
    print(f"Mean value: {data.mean():.2f}")
    print(f"First 50 values: {data[:50]}")

    # Save using measured sample rate
    save_wav(base_name + ".wav", data, measured_rate)
    save_csv(base_name + ".csv", data, measured_rate)
    save_plot(base_name + ".png", data, measured_rate)

    print("\nDone.")


if __name__ == "__main__":
    main()