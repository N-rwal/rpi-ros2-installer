#!/usr/bin/env python3

import serial
import time

PORT = "/dev/ttyUSB1"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

print("Starting hardware test... (Ctrl+C to stop)")

patterns = [
    0x04,  # alarmPin
    0x08,  # flash + red
]

try:
    while True:
        for p in patterns:
            ser.write(bytes([p]))
            print(f"Sent: {p} (0b{p:03b})")
            time.sleep(1)

except KeyboardInterrupt:
    pass

finally:
    ser.write(bytes([0x00]))  # turn everything off
    ser.close()
    print("Test stopped")
