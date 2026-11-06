import serial
import time

ser = serial.Serial('COM3', 115200)

buf = b''
count = 0
start = time.perf_counter()

while True:
    buf += ser.read(ser.in_waiting or 1)
    while b'\n' in buf:
        line, buf = buf.split(b'\n', 1)
        line = line.decode(errors='ignore').strip()
        try:
            parts = line.split()
            f1 = int(parts[1], 16)
            f2 = int(parts[2], 16)
            f3 = int(parts[3], 16)
            f4 = int(parts[4], 16)
            count += 1
        except (IndexError, ValueError):
            continue

    now = time.perf_counter()
    if now - start >= 1.0:
        print(f"Rate: {count / (now - start):.0f}/s")
        count = 0
        start = now