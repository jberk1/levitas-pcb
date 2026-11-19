"""Send commands to the levitas firmware and log everything it prints.

usage: python levitas_tool.py LOGFILE  "cmd[@delay_s]" ...  [--tail SECONDS]
e.g.   python levitas_tool.py run1.log "stream 1" "cal@5" "start@3" "stop@0.5"
Each command is sent, then the given delay (default 0.3 s) is waited while logging.
'#' message lines are echoed to the terminal; data lines only go to the log file.
"""
import sys
import time

import serial
from serial.tools import list_ports


def find_port():
    for p in list_ports.comports():
        if p.vid == 0x2E8A and p.pid != 0x000F:
            return p.device
    return None


def main():
    args = sys.argv[1:]
    tail = 0.5
    if '--tail' in args:
        i = args.index('--tail')
        tail = float(args[i + 1])
        del args[i:i + 2]
    logfile, steps = args[0], args[1:]
    port = find_port()
    if port is None:
        print('no Pico serial port found')
        return 1
    ser = serial.Serial(port, 115200, timeout=0.05)
    n_data = 0
    with open(logfile, 'w', newline='\n') as log:
        buf = b''

        def pump(seconds):
            nonlocal buf, n_data
            end = time.perf_counter() + seconds
            while time.perf_counter() < end:
                try:
                    buf += ser.read(ser.in_waiting or 1)
                except serial.SerialException:
                    return  # port went away (e.g. after the bootsel command)
                *lines, buf = buf.split(b'\n')
                for raw in lines:
                    line = raw.decode(errors='ignore').rstrip('\r')
                    log.write(line + '\n')
                    if line.startswith('#'):
                        print(line)
                    else:
                        n_data += 1

        pump(0.3)
        for step in steps:
            cmd, _, delay = step.partition('@')
            print('>>', cmd)
            log.write('>> ' + cmd + '\n')
            try:
                ser.write((cmd + '\n').encode())
            except serial.SerialException:
                print('serial port closed')
                break
            pump(float(delay) if delay else 0.3)
        pump(tail)
    try:
        ser.close()
    except serial.SerialException:
        pass
    print('port', port, 'data lines', n_data)
    return 0


if __name__ == '__main__':
    sys.exit(main())
