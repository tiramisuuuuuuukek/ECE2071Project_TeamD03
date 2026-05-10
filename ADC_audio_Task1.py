import numpy as np
import wave
import serial
from serial.tools import list_ports
import time

BAUD = 115200
TIMEOUT = 0.3

def list_stm_ports():
    ports = []
    for p in list_ports.comports(): #returns all COM ports connected to pc
        desc = (p.description or "").lower()
        if "stlink" in desc or "stmicroelectronics" in desc: #keep only STM related ports
            ports.append(p) #append to list
    return ports


#choose usable STM serial port
def choose_stm_port():
    ports = list_stm_ports()


    if not ports: #if no ports are found
        raise RuntimeError("No STM STLink virtual COM ports detected.")


    print("Detected STM ports:")
    for i, p in enumerate(ports, start=1): #loop through and give i (index) p (item)
        print(f"  {i}. {p.device} | {p.description}") #print available options


    if len(ports) == 1: #if only one port available
        print("Only one STM port detected. Selecting it automatically.")
        return ports[0].device


    while True: #ask user to pick if there are multiple choices
        choice = input("Choose port number: ").strip()
        if choice.isdigit():
            idx = int(choice)
            if 1 <= idx <= len(ports):
                return ports[idx - 1].device

#startup sequence to find recording method 
def startup():     
    valid = 0
    method = input("distance or time?")
    method = method.lower()
    if method == "time" or method == "t":
        valid = 1 
        method = 1
    elif method == "distance" or method == "d":
        valid = 1
        method = 0
    else:
        print("invalid input try again")
    while valid == 0:
        method = input("distance or time?")
        method = method.lower()
        if method == "time" or method == "t":
            valid = 1 
            method = 1
        elif method == "distance" or method == "d":
            valid = 1
            method = 0
        else:
            print("invalid input try again")
    if method == 1:
        duration = input('input recording duration')
        try:
            duration = int(duration)
        except ValueError:
            print('duration must be interger')
        while type(duration) != int:
            duration = input('input recording duration')
            try:
                duration = int(duration)
            except ValueError:
                print('duration must be interger')
        return [method,duration,255,254,253]
    if method == 0:
        distance = input('input recording duration')
        try:
            distance = int(distance)
        except ValueError:
            print('distance must be interger')
        while type(distance) != int:
            distance = input('input recording distance')
            try:
                distance = int(distance)
            except ValueError:
                print('distance must be interger')
        return [method,distance,255,254,253]
    

def recive_data(ser):
    data = []
    buffer = []
    END_SEQUENCE = [255, 254, 253]
    while True:
        temp = ser.read(1)[0]
        buffer.append(temp)
        if buffer[-3:] == END_SEQUENCE:
            break
        if len(buffer) > 3:
            buffer.pop(0)
        data.append(temp) 
    data = data[:-2]
    # convert list to numpy array
    data = np.array(data)
    # normalise to 0 to 255 range:
    data = (data - data.min()) / data.max() # scale to 0-1
    data = data * 255 # scale to 0-255
    data = data.astype(np.uint8) # convert to uint8 type

            
    with wave.open("audio.wav", 'wb') as wf:
        wf.setnchannels(1) # mono audio (single channel)
        wf.setsampwidth(1) # 8 bits (1 byte ) per sample
        wf.setframerate(SAMPLE_RATE) # set the sample rate that the data wasrecorded at
        wf.writeframes(data.tobytes()) # write the audio data to the file

    print("audio.wav saved successfully")
def start_protocall(ser):
    buffer = []
    START_SEQUENCE = [123,124,125]
    SAMPLE_RATE= 11000
    while True:
        temp = ser.read(1)[0]
        buffer.append(temp)
        if buffer[-3:] == START_SEQUENCE:
            print('start up successful')
            break
        if len(buffer) > 3:
            buffer.pop(0)
    recive_data(ser)



def main():
    head_port = choose_stm_port()
    print(f"\nUsing port: {head_port}")
    ser = serial.Serial(head_port, BAUD, timeout=TIMEOUT) #open COM port
    time.sleep(2.0) #wait for board startup
    startUp = startup()
    ser.write(startUp)
    start_protocall(ser)
    
if __name__ == "__main__":
    main()
