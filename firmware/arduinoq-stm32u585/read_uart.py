#!/usr/bin/env python3
"""Read STM32 uptime messages from /dev/ttyHS1 and print them."""

import serial
import sys

PORT = '/dev/ttyHS1'
BAUD = 209700

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=5)
    except serial.SerialException as e:
        print(f"Failed to open {PORT}: {e}", file=sys.stderr)
        sys.exit(1)

    print(f"Listening on {PORT} at {BAUD} baud...")
    try:
        while True:
            line = ser.readline()
            if line:
                print(line.decode('utf-8', errors='replace').strip())
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

if __name__ == '__main__':
    main()
