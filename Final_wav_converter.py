import csv
import threading
import wave
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np
import serial
import serial.tools.list_ports


# =========================
# STM / Project settings
# =========================

PORT = "COM6"          # Processing STM COM port
BAUD_RATE = 921600     # Must match Processing STM USART2
SAMPLE_RATE = 22050    # Task 3 output rate after downsampling
TEAM_ID = "D03"

RAW_FILENAME = "raw_ADC_values.data"


# =========================
# Output helpers
# =========================

def list_serial_ports():
    print("Available serial ports:")
    ports = serial.tools.list_ports.comports()

    if not ports:
        print("No serial ports found.")
        return

    for port in ports:
        print(f"  {port.device}: {port.description}")


def make_base_name(mode_name):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"{TEAM_ID}_{mode_name}_{SAMPLE_RATE}Hz_{timestamp}"


def save_raw(filename, data):
    with open(filename, "wb") as file:
        file.write(data.tobytes())
    print(f"Saved raw data: {filename}")


def save_wav(filename, data, sample_rate):
    with wave.open(filename, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(1)       # 8-bit unsigned PCM
        wf.setframerate(sample_rate)
        wf.writeframes(data.tobytes())
    print(f"Saved WAV: {filename}")


def save_csv(filename, data, sample_rate):
    time_axis = np.arange(len(data)) / sample_rate

    with open(filename, "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["sample_rate", sample_rate])
        writer.writerow(["time_s", "amplitude"])

        for time_s, amplitude in zip(time_axis, data):
            writer.writerow([time_s, int(amplitude)])

    print(f"Saved CSV: {filename}")


def save_plot(filename, data, sample_rate):
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


def save_outputs(data, mode_name):
    if len(data) == 0:
        print("No samples recorded. No files saved.")
        return

    base_name = make_base_name(mode_name)

    save_raw(RAW_FILENAME, data)

    print(f"\nActual samples recorded: {len(data)}")
    print(f"Min value: {data.min()}")
    print(f"Max value: {data.max()}")
    print(f"Mean value: {data.mean():.2f}")
    print(f"First 50 values: {data[:50]}")

    save_wav(base_name + ".wav", data, SAMPLE_RATE)
    save_csv(base_name + ".csv", data, SAMPLE_RATE)
    save_plot(base_name + ".png", data, SAMPLE_RATE)

    print("\nDone saving outputs.")


# =========================
# Recording modes
# =========================

def manual_recording_mode(ser):
    """
    Manual mode:
    Python sends 'M' to Processing STM.
    User enters duration.
    Python reads SAMPLE_RATE * duration audio bytes.
    """
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    ser.write(b"M")

    duration = int(input("Enter recording duration in seconds: "))
    total_samples = SAMPLE_RATE * duration

    data = bytearray()

    print("\nManual Recording Mode")
    print(f"Recording {duration} seconds at {SAMPLE_RATE} samples/second")
    print(f"Expected samples: {total_samples}")

    while len(data) < total_samples:
        remaining = total_samples - len(data)
        chunk = ser.read(min(2048, remaining))

        if not chunk:
            print("\nTimeout: no data received.")
            print("Check STM mode, COM port, baud rate, reset order, and wiring.")
            break

        data.extend(chunk)

        progress = 100 * len(data) / total_samples
        print(f"\rReceived {len(data)}/{total_samples} samples ({progress:.1f}%)", end="")

    print("\nFinished manual recording.")
    return np.frombuffer(data, dtype=np.uint8)


def distance_recording_mode(ser):
    """
    Distance mode:
    Python sends 'D' to Processing STM.
    Python sends threshold distance as ASCII text with newline.
    Processing STM controls when audio bytes are sent.
    Python records audio bytes until user types 'stop'.
    """
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    ser.write(b"D")

    distance = int(input("Enter trigger distance in cm: "))
    ser.write(f"{distance}\n".encode())

    print("\nDistance Trigger Mode activated.")
    print("STM will only transmit audio when object is within the trigger distance.")
    print("Type 'stop' then press Enter to finish distance recording.")

    stop_event = threading.Event()
    data = bytearray()

    def wait_for_stop():
        while not stop_event.is_set():
            command = input()
            if command.strip().lower() == "stop":
                stop_event.set()

    stop_thread = threading.Thread(target=wait_for_stop, daemon=True)
    stop_thread.start()

    while not stop_event.is_set():
        chunk = ser.read(2048)

        if chunk:
            data.extend(chunk)
            print(f"\rRecorded samples: {len(data)}", end="")

    print("\nStopping distance recording.")

    # Return STM to manual mode after stopping distance mode.
    ser.write(b"M")

    return np.frombuffer(data, dtype=np.uint8)


# =========================
# Main CLI
# =========================

def main():
    list_serial_ports()

    print(f"\nOpening {PORT} at {BAUD_RATE} baud...")

    try:
        with serial.Serial(
            port=PORT,
            baudrate=BAUD_RATE,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=2
        ) as ser:

            print(f"Connected to: {ser.name}")

            while True:
                print("\nSelect mode:")
                print("M - Manual Recording Mode")
                print("D - Distance Trigger Mode")
                print("Q - Quit")

                mode = input("Enter choice: ").strip().upper()

                if mode == "M":
                    data = manual_recording_mode(ser)
                    save_outputs(data, "manual")

                elif mode == "D":
                    data = distance_recording_mode(ser)
                    save_outputs(data, "distance")

                elif mode == "Q":
                    print("Exiting.")
                    break

                else:
                    print("Invalid option. Please enter M, D, or Q.")
                    continue

                again = input("\nDo you want to continue? (Y/N): ").strip().upper()
                if again != "Y":
                    print("Exiting.")
                    break

    except serial.SerialException as error:
        print(f"Serial error: {error}")
        print("Check the COM port and make sure no other program is using the STM serial port.")


if __name__ == "__main__":
    main()