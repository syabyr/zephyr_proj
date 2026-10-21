#!/usr/bin/env python3
"""Read a serial console and prefix every line with the host elapsed time.

Usage:
    python console.py /dev/cu.usbmodemXXXX [seconds] [baud]

Line timestamps make it possible to compare the uptime the target reports
against the wall clock the host sees, which is how a changed kernel tick
source is validated.

The line settings are applied with termios on the already open descriptor:
configuring the port with stty before opening it does not stick, because
the AppleUSBACM driver reprograms the port on open.
"""
import os
import select
import sys
import termios
import time

BAUDS = {
    "9600": termios.B9600,
    "19200": termios.B19200,
    "38400": termios.B38400,
    "57600": termios.B57600,
    "115200": termios.B115200,
    "230400": termios.B230400,
}

dev = sys.argv[1]
seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
baud = BAUDS[sys.argv[3] if len(sys.argv) > 3 else "115200"]

fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
attrs = termios.tcgetattr(fd)
attrs[0] = 0
attrs[1] = 0
attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
attrs[3] = 0
attrs[4] = baud
attrs[5] = baud
attrs[6][termios.VMIN] = 0
attrs[6][termios.VTIME] = 0
termios.tcsetattr(fd, termios.TCSANOW, attrs)

start = time.monotonic()
end = start + seconds
buf = ""

while time.monotonic() < end:
    ready, _, _ = select.select([fd], [], [], 0.2)
    if not ready:
        continue
    try:
        chunk = os.read(fd, 8192)
    except BlockingIOError:
        continue
    buf += chunk.decode("utf-8", "replace")
    while "\n" in buf:
        line, buf = buf.split("\n", 1)
        print(f"[{time.monotonic() - start:6.2f}] {line.rstrip()}", flush=True)

os.close(fd)