#!/usr/bin/env python3

import serial
import socket
import threading
import time
from enum import IntFlag


class Outputs(IntFlag):
    MAIN_LED = 0x08
    STATUS_LED = 0x02
    ALARM = 0x04

alarm_stop_event = threading.Event()

UDP_IP = "0.0.0.0"
UDP_PORT = 5005

def send_command(ser, cmd_byte):
    ser.write(bytes([cmd_byte]))

def flash_led_alarm(ser, duration=5):
    end_time = time.time() + duration
    while time.time() < end_time:
        if alarm_stop_event.is_set():  # Stop immediately if event is set
            break
        send_command(ser, Outputs.MAIN_LED | Outputs.ALARM)
        time.sleep(0.5)
        if alarm_stop_event.is_set():
            break
        send_command(ser, 0x00)
        time.sleep(0.5)

    send_command(ser, 0x00)
    alarm_stop_event.clear()

    send_command(ser, 0x00)

def handle_command(ser, command):
    cmd_num = int(command)

    if cmd_num == 0:
        alarm_stop_event.set()  # Stop any running alarm
        send_command(ser, 0x00)

    elif cmd_num == 1:
        threading.Thread(
            target=flash_led_alarm,
            args=(ser, 5),
            daemon=True
        ).start()

    elif cmd_num == 2:
        send_command(ser, Outputs.MAIN_LED)

def main():

    ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    print(f"Listening for UDP commands on port {UDP_PORT}")

    try:

        while True:

            data, addr = sock.recvfrom(1024)
            message = data.decode().strip()
            send_command(ser, 0x02)

            print(f"Received UDP from {addr}: {message}")

            if message.startswith("cmd:"):
                cmd_value = message.split("cmd:", 1)[1]
                handle_command(ser, cmd_value)
            else:
                print("Ignored non-command message")

    except KeyboardInterrupt:
        pass

    finally:
        ser.close()
        sock.close()

if __name__ == "__main__":
    main()

