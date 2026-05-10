import threading
import wave
import time
from datetime import datetime

import numpy as np
import serial
import serial.tools.list_ports


# =========================
# STM / Task 4 settings
# =========================

PORT = "COM6"
BAUD_RATE = 921600

SAMPLE_RATE = 44100          # Task 4 target sample rate
TEAM_ID = "D03"

# 12-bit packing:
# 2 samples = 24 bits = 3 bytes
BYTES_PER_2_SAMPLES = 3

RAW_FILENAME = "raw_task4_packed.data"


# =========================
# Serial / output helpers
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
    return f"{TEAM_ID}_{mode_name}_{SAMPLE_RATE}Hz_12bit_{timestamp}"


def unpack_12bit_packed(data):
    """
    Unpack two 12-bit samples packed into three bytes.

    STM packing format:
        byte0 = sampleA[7:0]
        byte1 = sampleA[11:8] | sampleB[3:0] << 4
        byte2 = sampleB[11:4]
    """
    usable_len = (len(data) // 3) * 3
    data = data[:usable_len]

    if usable_len == 0:
        return np.array([], dtype=np.uint16)

    raw = np.frombuffer(data, dtype=np.uint8).reshape(-1, 3)

    b0 = raw[:, 0].astype(np.uint16)
    b1 = raw[:, 1].astype(np.uint16)
    b2 = raw[:, 2].astype(np.uint16)

    sample_a = b0 | ((b1 & 0x0F) << 8)
    sample_b = ((b1 >> 4) & 0x0F) | (b2 << 4)

    samples12 = np.empty(sample_a.size + sample_b.size, dtype=np.uint16)
    samples12[0::2] = sample_a
    samples12[1::2] = sample_b

    return samples12


def save_raw(filename, data):
    with open(filename, "wb") as file:
        file.write(data)
    print(f"Saved raw packed data: {filename}")


def save_wav_16bit(filename, samples12, sample_rate):
    """
    Save unsigned 12-bit ADC samples as signed 16-bit PCM WAV.

    STM data:
        unsigned 12-bit, 0 to 4095, midpoint around 2048

    WAV data:
        signed 16-bit PCM
    """
    centred = samples12.astype(np.int32) - 2048
    audio16 = (centred << 4).astype(np.int16)

    with wave.open(filename, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)      # 16-bit WAV container
        wf.setframerate(int(round(sample_rate)))
        wf.writeframes(audio16.tobytes())

    print(f"Saved WAV: {filename} at {int(round(sample_rate))} Hz")


def print_stats(samples12, actual_sample_rate=None):
    print(f"\nSamples reconstructed: {len(samples12)}")
    print(f"Min 12-bit: {samples12.min()}")
    print(f"Max 12-bit: {samples12.max()}")
    print(f"Mean 12-bit: {samples12.mean():.2f}")
    print(f"First 20 samples: {samples12[:20]}")

    if actual_sample_rate is not None:
        print(f"Measured sample rate: {actual_sample_rate:.1f} samples/s")

    if samples12.min() <= 10 or samples12.max() >= 4085:
        print("\nWARNING: Samples are very close to 0 or 4095.")
        print("This may indicate clipping, byte misalignment, or input level too high.")


def save_outputs(packed_data, mode_name, actual_sample_rate=None):
    samples12 = unpack_12bit_packed(packed_data)

    if len(samples12) == 0:
        print("No usable packed data. No files saved.")
        return

    base_name = make_base_name(mode_name)

    save_raw(base_name + "_packed_raw.data", packed_data)
    print_stats(samples12, actual_sample_rate)

    # Final Task 4 WAV: intended 44.1 kHz output
    save_wav_16bit(base_name + ".wav", samples12, SAMPLE_RATE)

    # Debug WAV using measured rate, useful if playback pitch is wrong
    if actual_sample_rate is not None:
        measured_rate = int(round(actual_sample_rate))
        save_wav_16bit(base_name + f"_measured_{measured_rate}Hz.wav", samples12, measured_rate)

    print("\nDone saving outputs.")


def estimate_rates(byte_count, elapsed_s):
    if elapsed_s <= 0:
        return 0.0, 0.0

    byte_rate = byte_count / elapsed_s
    sample_rate = ((byte_count // 3) * 2) / elapsed_s
    return byte_rate, sample_rate


# =========================
# Recording modes
# =========================

def manual_recording_mode(ser):
    """
    Manual Task 4:
    Python asks for duration first.
    Then sends I to clear STM output.
    Then sends M to start manual recording.
    Python reads packed 12-bit audio.
    """
    duration = int(input("Enter recording duration in seconds: "))

    total_samples = SAMPLE_RATE * duration
    total_groups = total_samples // 2
    total_bytes = total_groups * BYTES_PER_2_SAMPLES

    packed_data = bytearray()

    ser.write(b"I")
    time.sleep(0.2)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    ser.write(b"M")

    print("\nManual Recording Mode, Task 4")
    print(f"Recording {duration} seconds at {SAMPLE_RATE} samples/second")
    print(f"Expected packed bytes: {total_bytes}")

    start_time = time.perf_counter()

    while len(packed_data) < total_bytes:
        remaining = total_bytes - len(packed_data)
        chunk = ser.read(min(4096, remaining))

        if not chunk:
            print("\nTimeout: no data received.")
            print("Check STM mode, COM port, baud rate, reset order, and wiring.")
            break

        packed_data.extend(chunk)

        progress = 100 * len(packed_data) / total_bytes
        print(f"\rReceived {len(packed_data)}/{total_bytes} bytes ({progress:.1f}%)", end="")

    end_time = time.perf_counter()

    ser.write(b"I")
    time.sleep(0.2)
    ser.reset_input_buffer()

    print("\nFinished manual recording.")

    actual_duration = end_time - start_time
    actual_byte_rate, actual_sample_rate = estimate_rates(len(packed_data), actual_duration)

    print(f"Actual recording time: {actual_duration:.3f} s")
    print(f"Actual byte rate: {actual_byte_rate:.1f} bytes/s")
    print(f"Actual sample rate: {actual_sample_rate:.1f} samples/s")

    return bytes(packed_data), actual_sample_rate


def distance_recording_mode(ser):
    """
    Distance Task 4:
    Python sends D and threshold together, e.g. b"D10\n".
    Processing STM only transmits packed 12-bit audio when object is within threshold.
    Python records until user types 'stop'.
    """

    # Ask first, before sending anything to STM
    distance = int(input("Enter trigger distance in cm: "))

    # Put STM into idle and clear old UART data
    # force stm OUT of manual mode
    for _ in range(5):
        ser.write(b"I")
        time.sleep(0.05)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    # Send D and threshold together so STM receives one clean command sequence
    command = f"D{distance}\n".encode()
    for b in command:
        ser.write(bytes([b]))
        time.sleep(0.03)  # Small delay between bytes

    # Give STM time to parse threshold and start TIM1/TIM7 distance mode
    time.sleep(0.3)

    # Clear any old transition bytes from PC buffer.
    # This does not affect the STM command because it was already sent.
    ser.reset_input_buffer()

    print("\nDistance Trigger Mode, Task 4 activated.")
    print("STM will only transmit packed 12-bit audio when object is within trigger distance.")
    print("Type 'stop' then press Enter to finish distance recording.")

    stop_event = threading.Event()
    packed_data = bytearray()

    def wait_for_stop():
        while not stop_event.is_set():
            command = input()
            if command.strip().lower() == "stop":
                stop_event.set()

    stop_thread = threading.Thread(target=wait_for_stop, daemon=True)
    stop_thread.start()

    start_time = time.perf_counter()

    while not stop_event.is_set():
        chunk = ser.read(4096)

        if chunk:
            packed_data.extend(chunk)
            usable_samples = (len(packed_data) // 3) * 2
            print(
                f"\rRecorded packed bytes: {len(packed_data)} | samples: {usable_samples}",
                end=""
            )

    end_time = time.perf_counter()

    print("\nStopping distance recording.")

    ser.write(b"I")
    time.sleep(0.25)
    ser.reset_input_buffer()

    # Keep only full 3-byte packets
    usable_len = (len(packed_data) // 3) * 3
    packed_data = packed_data[:usable_len]

    actual_duration = end_time - start_time
    actual_byte_rate, actual_sample_rate = estimate_rates(len(packed_data), actual_duration)

    print(f"Actual distance-mode elapsed time: {actual_duration:.3f} s")
    print(f"Actual byte rate while script was active: {actual_byte_rate:.1f} bytes/s")
    print(f"Equivalent sample rate while script was active: {actual_sample_rate:.1f} samples/s")
    print("Note: Distance mode rate includes time when object may not be triggering audio.")

    return bytes(packed_data), actual_sample_rate

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
                print("M - Manual Recording Mode, Task 4, 44.1 kHz 12-bit")
                print("D - Distance Trigger Mode, Task 4, 44.1 kHz 12-bit")
                print("Q - Quit")

                mode = input("Enter choice: ").strip().upper()

                if mode == "M":
                    packed_data, actual_rate = manual_recording_mode(ser)
                    save_outputs(packed_data, "manual", actual_rate)

                elif mode == "D":
                    packed_data, actual_rate = distance_recording_mode(ser)
                    save_outputs(packed_data, "distance", actual_rate)

                elif mode == "Q":
                    ser.write(b"I")
                    print("Exiting.")
                    break

                else:
                    print("Invalid option. Please enter M, D, or Q.")
                    continue

                again = input("\nDo you want to continue? (Y/N): ").strip().upper()
                if again != "Y":
                    ser.write(b"I")
                    print("Exiting.")
                    break

    except serial.SerialException as error:
        print(f"Serial error: {error}")
        print("Check the COM port and make sure no other program is using the STM serial port.")


if __name__ == "__main__":
    main()