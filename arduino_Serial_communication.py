import serial
import time

# Replace with your Arduino's serial port
arduino = serial.Serial(port='COM4', baudrate=115200, timeout=.1)

def write_warning():
    # Write low priority warning to Arduino
    arduino.write(b'1')  # Sending '1' to indicate low-priority warning

def read_arduino():
    data = arduino.readline().decode('utf-8').strip()  # Read Arduino output
    if data:
        print(f"Arduino says: {data}")
        if data == "LOW_PRIORITY":
            # Do something in response to low priority warning
            print("Low priority warning received.")
        elif data == "LAST_MESSAGE":
            # Do something in response to last message
            print("Last message warning received.")
            
while True:
    read_arduino()
    time.sleep(1)
