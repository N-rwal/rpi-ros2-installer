#!/usr/bin/env python3

import serial
import time

PORT = "/dev/ttyUSB0"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)
patterns = [
    0x00,  # all OFF
    0x01,  # 
    0x02,  # redPin
    0x04,  # alarmPin
    0x08,  # flashPin
    0x10,  # flash + alarm
    0x06,
]

try:
    while True:
        ser.write(bytes([3]))
        time.sleep(1)

except KeyboardInterrupt:
    pass

finally:
    ser.write(bytes([0x00]))  # turn everything off
    ser.close()
