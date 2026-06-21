#!/usr/bin/env python3
"""
A Unit Test TCP Serverr to send Value to Labview via TCP connection.
This will simulate Arduino Q sending data over Wifi

Protocol
--------
On connect, the server sends one line every UPDATE_INTERVAL seconds:

    <value>\r\n        e.g.  "42.73\r\n"

That is exactly what LabVIEW's TCP Read in CRLF mode expects. Next is to send more complex data

Run:
    python server.py
Then point LabVIEW at  127.0.0.1 : 9000
"""

import math
import socket
import threading
import time

HOST = "0.0.0.0"        # all interfaces; use "127.0.0.1" for local-only
PORT = 9000
UPDATE_INTERVAL = 0.25  # seconds between samples (4 Hz)


def read_value(t: float) -> float:
    """Return the current value to display on the gauge.

    Replace the body of this function with a real measurement
    (DAQ read, serial port, sensor, etc.). Here we simulate a
    smooth 5..95 signal so the gauge looks alive.
    """
    return round(50 + 45 * math.sin(t / 3.0), 2)


def handle_client(conn: socket.socket, addr) -> None:
    print(f"[+] client connected: {addr}")
    start = time.monotonic()
    try:
        while True:
            value = read_value(time.monotonic() - start)
            conn.sendall(f"{value}\r\n".encode("ascii"))
            time.sleep(UPDATE_INTERVAL)
    except (ConnectionResetError, BrokenPipeError, OSError):
        pass
    finally:
        conn.close()
        print(f"[-] client disconnected: {addr}")


def main() -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((HOST, PORT))
        server.listen()
        print(f"Gauge TCP server listening on {HOST}:{PORT}  (Ctrl+C to stop)")
        try:
            while True:
                conn, addr = server.accept()
                threading.Thread(
                    target=handle_client, args=(conn, addr), daemon=True
                ).start()
        except KeyboardInterrupt:
            print("\nShutting down.")


if __name__ == "__main__":
    main()
