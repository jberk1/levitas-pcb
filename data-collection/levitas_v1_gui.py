#!/usr/bin/env python3
"""
LeviTAS v1 monitor and manual high voltage control.

Works with the firmware for the first version of the PCB (firmware/src/main.c:
four FDC2112 capacitance sensors, four high voltage outputs). It shows:

  - the current reading of each capacitance sensor (raw value and estimated
    capacitance)
  - a plot of the height (z position) of the levitating mass versus time,
    together with the applied voltage
  - a slider to set the high voltage on the mass by hand (the same voltage
    on all four electrodes), and a box to type any other firmware command

Run with:

    python levitas_v1_gui.py                 (choose the serial port in the window)
    python levitas_v1_gui.py --port COM3     (connect to COM3 right away)
    python levitas_v1_gui.py --simulate      (no hardware: made-up data, to try the program)

Needs the "pyserial" module (pip install pyserial) to talk to the Pico.
Everything else is from the Python standard library (tkinter).

------------------------------------------------------------------------------
How readings are turned into capacitance and height
------------------------------------------------------------------------------
Capacitance: each FDC2112 measures the oscillation frequency of an LC tank
(18 uH inductor, 33 pF capacitor, plus the sense electrodes). The capacitance
shown is the total capacitance of that tank worked out from the frequency.
Its absolute value is only approximate (the chip's internal oscillator and the
inductor both vary by several percent from part to part); the CHANGE in
capacitance is much more trustworthy. Higher reading = lower capacitance =
mass further from the electrodes.

Height: each sensor's reading is compared with two reference readings, one
taken with the mass resting on its bottom stop and one with the mass against
its top stop:

    fraction of travel = (bottom reading - reading) / (bottom reading - top reading)

The height plotted is the average of the four sensors, multiplied by the
travel between the stops in um (set in the window). The four sensors are
also plotted separately as thin lines: if they disagree, the mass is tilted.
NOTE: this assumes the reading changes in proportion to height, which is only
roughly true (capacitance goes as 1/gap), so heights between the stops are
approximate.

The reference readings drift, so set them again when the plot at rest is not
at zero: with high voltage off and the mass at rest press "Set BOTTOM"; with
the mass held against the top stop press "Set TOP". They are saved in
levitas_v1_calibration.json next to this file.

------------------------------------------------------------------------------
Safety
------------------------------------------------------------------------------
The "HV OFF" button and the Esc key send the firmware's "stop" command, which
switches the high voltage off whatever the firmware is doing. The firmware
also switches it off by itself when this program closes the serial port.
"""

import argparse
import json
import math
import os
import queue
import random
import threading
import time
from collections import deque

import tkinter as tk
from tkinter import ttk

try:
    import serial
    import serial.tools.list_ports
except ImportError:  # the program still runs with simulated data
    serial = None

# ===========================================================================
# Settings
# ===========================================================================

BAUD = 115200
NUM_CH = 4

# ----- FDC2112 settings: these must match firmware/src/main.c -----
FDC_OFFSET = 0x1800        # offset register (printed value = 16-bit data - offset)
FDC_FIN_SEL = 2            # from clock dividers register = 0x2002
FDC_FREF_DIVIDER = 2       # from clock dividers register = 0x2002
FDC_F_CLK_HZ = 43.4e6      # internal oscillator (nominal; 35-55 MHz chip to chip)
TANK_L_H = 18e-6           # inductor across each sensor (18 uH)
READ_MAX = 0x0FFE          # largest real reading (0x0fff = read failed)

# ----- high voltage -----
HV_MAX_V = 240             # top of the slider (the firmware also limits to this)
HV_SEND_INTERVAL_S = 0.05  # while dragging the slider, send at most this often
HV_PRESETS = (50, 100, 150, 200, 240)

# ----- height -----
# Reference readings used until you press "Set BOTTOM" / "Set TOP"
# (measured on one board on 2026-11-18; they drift, so set your own).
DEFAULT_BOTTOM = (1443.0, 1770.0, 1775.0, 1639.0)  # mass resting on the bottom stop
DEFAULT_TOP = (1289.0, 1610.0, 1628.0, 1470.0)     # mass against the top stop
DEFAULT_TRAVEL_UM = 80.0   # distance between the two stops

# ----- display -----
STREAM_EVERY = 8           # ask the firmware for every 8th data line (plenty for a display)
AVERAGE_MS = 10.0          # readings are averaged over this long before use
DEFAULT_WINDOW_S = 10.0    # length of time shown in the plot
MAX_HISTORY_S = 120.0
CAPTURE_S = 0.5            # "Set BOTTOM/TOP" average the last 0.5 s
READOUT_S = 0.2            # the numbers shown are averaged over the last 0.2 s
REFRESH_MS = 40            # redraw interval (25 frames per second)
MAX_PLOT_POINTS = 500      # points drawn per line (more than this is slow to paint)

MODE_NAMES = {0: 'idle (HV off)', 1: 'fixed voltage', 2: 'calibrating', 3: 'PD controller'}
SENSOR_COLORS = ('#2ca02c', '#9467bd', '#8c564b', '#e377c2')
HEIGHT_COLOR = '#1f77b4'
VOLTAGE_COLOR = '#ff7f0e'


# ===========================================================================
# Readings -> capacitance
# ===========================================================================

def counts_to_pf(counts):
    """Total capacitance (pF) of a sensor's LC tank from its printed reading.

    The FDC2112 measures the oscillation frequency of the tank:
        f = FIN_SEL * (f_CLK / FREF_DIVIDER) * (reading + offset) / 2^16
    and the tank capacitance follows from f = 1 / (2 pi sqrt(L C)).
    (Higher reading = higher frequency = lower capacitance.)
    """
    f = FDC_FIN_SEL * (FDC_F_CLK_HZ / FDC_FREF_DIVIDER) * (counts + FDC_OFFSET) / 65536.0
    return 1e12 / ((2.0 * math.pi * f) ** 2 * TANK_L_H)


# ===========================================================================
# Height calibration (the two reference readings per sensor)
# ===========================================================================

class Calibration:
    def __init__(self, path):
        self.path = path
        self.bottom = list(DEFAULT_BOTTOM)
        self.top = list(DEFAULT_TOP)
        self.travel_um = DEFAULT_TRAVEL_UM
        self.bottom_set = False   # False = still the built-in default
        self.top_set = False
        self.load()

    def load(self):
        try:
            with open(self.path) as f:
                d = json.load(f)
            bottom, top = [float(v) for v in d['bottom']], [float(v) for v in d['top']]
            if len(bottom) == NUM_CH and len(top) == NUM_CH:
                self.bottom, self.top = bottom, top
                self.travel_um = float(d.get('travel_um', DEFAULT_TRAVEL_UM))
                self.bottom_set = bool(d.get('bottom_set', True))
                self.top_set = bool(d.get('top_set', True))
        except (OSError, ValueError, KeyError, TypeError):
            pass  # no file yet (or unreadable): keep the defaults

    def save(self):
        try:
            with open(self.path, 'w') as f:
                json.dump({'bottom': self.bottom, 'top': self.top, 'travel_um': self.travel_um,
                           'bottom_set': self.bottom_set, 'top_set': self.top_set}, f, indent=2)
        except OSError:
            pass

    def reset(self):
        self.bottom, self.top = list(DEFAULT_BOTTOM), list(DEFAULT_TOP)
        self.bottom_set = self.top_set = False
        self.save()

    def fractions(self, counts):
        """Fraction of travel (0 = bottom stop, 1 = top stop) seen by each
        sensor; None for a sensor with no reading or no usable calibration."""
        out = []
        for c, b, t in zip(counts, self.bottom, self.top):
            span = b - t
            out.append(None if c is None or abs(span) < 1.0 else (b - c) / span)
        return out

    def height_um(self, counts):
        """(average height in um or None, [height of each sensor in um or None])"""
        fr = self.fractions(counts)
        each = [None if f is None else f * self.travel_um for f in fr]
        valid = [h for h in each if h is not None]
        return (sum(valid) / len(valid) if valid else None), each


# ===========================================================================
# Where the data comes from: the serial port, or a simulator
# ===========================================================================

class LineProcessor:
    """Turns the bytes from the Pico into averaged readings.

    Each data line printed by the firmware is
        <microseconds> 0x.... 0x.... 0x.... 0x.... <V> <V> <V> <V> <height %> <target %> <mode>
    (older firmware stops after the four voltages). Readings are averaged over
    AVERAGE_MS and put on a queue as
        ('sample', (time in seconds, [4 average readings or None], voltage, mode, firmware height % or None))
    Lines starting with '#' are messages from the firmware: ('info', text).
    """

    def __init__(self, out_queue, average_ms=AVERAGE_MS):
        self.q = out_queue
        self.average_us = average_ms * 1000.0
        self.buf = b''
        self.n_lines = 0   # good data lines so far
        self.n_bad = 0     # malformed lines so far
        self.last_t = None # timestamp of the previous line
        self._start_block(None)

    def _start_block(self, t):
        self.block_t0 = t
        self.sums = [0] * NUM_CH
        self.counts = [0] * NUM_CH
        self.hv_sum = 0.0
        self.hv_n = 0

    def feed(self, data):
        self.buf += data
        *lines, self.buf = self.buf.split(b'\n')   # keep any partial last line
        if len(self.buf) > 100000:                 # garbage without newlines
            self.buf = b''
        for line in lines:
            self._line(line.strip())

    def _line(self, line):
        if not line:
            return
        if line.startswith(b'#'):
            self.q.put(('info', line[1:].decode(errors='replace').strip()))
            return
        parts = line.split()
        if len(parts) < 9:
            self.n_bad += 1
            return
        try:
            t = int(parts[0])
            vals = [int(p, 16) for p in parts[1:5]]
            hv = max(float(p) for p in parts[5:9])
            fw_height = float(parts[9]) if len(parts) > 9 else None
            mode = int(parts[11]) if len(parts) > 11 else None
        except ValueError:
            self.n_bad += 1
            return
        self.n_lines += 1

        if self.last_t is not None and t < self.last_t:
            # time went backwards: the Pico was restarted
            self.q.put(('restart', None))
            self._start_block(t)
        elif self.block_t0 is None:
            self._start_block(t)
        self.last_t = t
        for i, v in enumerate(vals):
            if v <= READ_MAX:          # (0x0fff = read failed: leave it out)
                self.sums[i] += v
                self.counts[i] += 1
        self.hv_sum += hv
        self.hv_n += 1
        if t - self.block_t0 >= self.average_us:
            avg = [self.sums[i] / self.counts[i] if self.counts[i] else None
                   for i in range(NUM_CH)]
            self.q.put(('sample', ((self.block_t0 + t) / 2e6, avg, self.hv_sum / self.hv_n, mode, fw_height)))
            self._start_block(None)


class SerialSource(threading.Thread):
    """Reads from the serial port in the background, and writes the commands
    queued with send() (only this thread touches the port)."""

    def __init__(self, port, processor):
        super().__init__(daemon=True)
        self.port = port
        self.processor = processor
        self.stop_event = threading.Event()
        self.outgoing = queue.Queue()

    def send(self, command):
        self.outgoing.put(command)

    def _write_pending(self, ser):
        while True:
            try:
                command = self.outgoing.get_nowait()
            except queue.Empty:
                return
            ser.write((command + '\n').encode())

    def run(self):
        try:
            with serial.Serial(self.port, BAUD, timeout=0.05) as ser:
                while not self.stop_event.is_set():
                    self._write_pending(ser)
                    data = ser.read(ser.in_waiting or 1)
                    if data:
                        self.processor.feed(data)
                self._write_pending(ser)   # e.g. the "stop" sent while closing
                ser.flush()
        except Exception as e:  # port unplugged, access denied, ...
            self.processor.q.put(('error', f'{self.port}: {e}'))

    def stop(self):
        self.stop_event.set()


class SimSource(threading.Thread):
    """Made-up data in the same format as the firmware, to try the program
    without hardware. The pretend mass behaves roughly like the real one: it
    leaves the bottom stop at about 200 V, then snaps to the top stop, and
    only drops again when the voltage is almost zero."""

    RATE_HZ = 500
    NOISE_COUNTS = 1.0
    LIFT_V = 200.0     # voltage that just lifts the mass off the bottom stop
    RELEASE_V = 15.0   # voltage that just holds it against the top stop

    def __init__(self, processor):
        super().__init__(daemon=True)
        self.processor = processor
        self.stop_event = threading.Event()
        self.voltage = 0.0
        self.z = 0.0   # fraction of travel

    def send(self, command):
        parts = command.split()
        if not parts:
            return
        if parts[0] == 'stop':
            self.voltage = 0.0
            self.processor.feed(b'# stopped, high voltage off\n')
        elif parts[0] == 'hv' and len(parts) > 1:
            try:
                self.voltage = min(max(float(parts[1]), 0.0), HV_MAX_V)
            except ValueError:
                pass
        elif parts[0] != 'stream':
            self.processor.feed(('# (simulator) ignoring command "%s"\n' % command).encode())

    def line(self, t):
        # voltage needed to balance the weight shrinks as the gap closes
        balance_v = self.LIFT_V + (self.RELEASE_V - self.LIFT_V) * self.z
        speed = ((self.voltage / balance_v) ** 2 - 1.0) * 8.0   # fraction of travel per second
        self.z = min(max(self.z + min(max(speed, -30.0), 30.0) / self.RATE_HZ, 0.0), 1.0)
        vals = []
        for b, top in zip(DEFAULT_BOTTOM, DEFAULT_TOP):
            v = int(round(b - self.z * (b - top) + random.gauss(0.0, self.NOISE_COUNTS)))
            vals.append(min(max(v, 0), READ_MAX))
        v = self.voltage
        return ('%d %s %.2f %.2f %.2f %.2f -1.00 50.00 %d\n' % (
            int(t * 1e6), ' '.join('0x%04x' % x for x in vals), v, v, v, v, 1 if v > 0 else 0)).encode()

    def run(self):
        self.processor.feed(b'# simulated data (no hardware connected)\n')
        t0 = time.perf_counter()
        n = 0
        while not self.stop_event.is_set():
            target = int((time.perf_counter() - t0) * self.RATE_HZ)
            chunk = []
            while n < target:
                chunk.append(self.line(n / self.RATE_HZ))
                n += 1
            if chunk:
                self.processor.feed(b''.join(chunk))
            time.sleep(0.01)

    def stop(self):
        self.stop_event.set()


# ===========================================================================
# Drawing: height and voltage versus time
# ===========================================================================

def nice_ticks(lo, hi, target=5):
    """Round tick positions covering lo..hi, and the number of decimals to
    print them with."""
    span = hi - lo
    if span <= 0 or not math.isfinite(span):
        return [lo], 2
    raw = span / target
    mag = 10.0 ** math.floor(math.log10(raw))
    step = 10.0 * mag
    for m in (1.0, 2.0, 5.0):
        if raw <= m * mag:
            step = m * mag
            break
    ticks = []
    v = math.ceil(lo / step - 1e-9) * step
    while v <= hi + step * 1e-9:
        ticks.append(0.0 if abs(v) < step * 1e-9 else v)
        v += step
    decimals = max(0, int(math.ceil(-math.log10(step) - 1e-9)))
    return ticks, decimals


class HeightPlot:
    """Height of the mass (left axis, um) and applied voltage (right axis, V)
    versus time, drawn on a tkinter canvas."""

    LEFT, RIGHT, TOP, BOTTOM = 62, 56, 30, 26

    def __init__(self, parent):
        self.canvas = tk.Canvas(parent, bg='white', highlightthickness=0, width=640, height=360)

    def draw(self, ts, height, each, volts, window_s, travel_um, show_each):
        """ts: times in seconds relative to now (<= 0). height: average height
        (None = no data). each: four lists, the height from each sensor.
        volts: applied voltage."""
        c = self.canvas
        c.delete('all')
        w, h = c.winfo_width(), c.winfo_height()
        if w < 200 or h < 120:
            return
        x0, x1 = self.LEFT, w - self.RIGHT
        y0, y1 = self.TOP, h - self.BOTTOM
        font = ('Segoe UI', 8)

        # vertical range of the height axis: always shows both stops, and
        # grows if the data goes beyond them
        lo, hi = -0.1 * travel_um, 1.1 * travel_um
        series = [height] + (list(each) if show_each else [])
        for ys in series:
            valid = [y for y in ys if y is not None]
            if valid:
                lo, hi = min(lo, min(valid)), max(hi, max(valid))
        if hi - lo < 1e-9:
            lo, hi = lo - 1.0, hi + 1.0

        def px(t):
            return x0 + (t + window_s) / window_s * (x1 - x0)

        def py(v):
            return y1 - (v - lo) / (hi - lo) * (y1 - y0)

        def pv(v):   # voltage axis: 0 .. HV_MAX_V over the full height
            return y1 - v / float(HV_MAX_V) * (y1 - y0)

        # grid and tick labels
        yticks, ydec = nice_ticks(lo, hi, max(3, (y1 - y0) // 32))
        for v in yticks:
            y = py(v)
            c.create_line(x0, y, x1, y, fill='#e6e6e6')
            c.create_text(x0 - 6, y, text=f'{v:.{ydec}f}', anchor='e', fill=HEIGHT_COLOR, font=font)
        for v in range(0, HV_MAX_V + 1, 40):
            c.create_text(x1 + 6, pv(v), text=str(v), anchor='w', fill=VOLTAGE_COLOR, font=font)
        xticks, xdec = nice_ticks(-window_s, 0.0, max(2, (x1 - x0) // 90))
        for t in xticks:
            x = px(t)
            c.create_line(x, y0, x, y1, fill='#e6e6e6')
            c.create_text(x, y1 + 4, text=f'{t:.{xdec}f}', anchor='n', fill='#444', font=font)
        c.create_rectangle(x0, y0, x1, y1, outline='#888')
        c.create_text(x0 - 10, y1 + 4, text='time (s)', anchor='ne', fill='#666', font=font)
        c.create_text(4, y0 - 16, text='height z (µm)', anchor='nw', fill=HEIGHT_COLOR, font=('Segoe UI', 9, 'bold'))
        c.create_text(w - 4, y0 - 16, text='voltage (V)', anchor='ne', fill=VOLTAGE_COLOR, font=('Segoe UI', 9, 'bold'))

        # the two stops
        for v, label in ((0.0, 'bottom stop'), (travel_um, 'top stop')):
            y = py(v)
            c.create_line(x0, y, x1, y, fill='#999', dash=(4, 3))
            c.create_text(x0 + 4, y - 1, text=label, anchor='sw', fill='#777', font=font)

        def polyline(ys, to_y, color, width):
            seg = []   # (a gap wherever there is no value)
            for t, v in zip(ts, ys):
                if v is None or t < -window_s:
                    if len(seg) >= 4:
                        c.create_line(*seg, fill=color, width=width)
                    seg = []
                else:
                    seg.extend((px(t), min(max(to_y(v), y0), y1)))
            if len(seg) >= 4:
                c.create_line(*seg, fill=color, width=width)

        polyline(volts, pv, VOLTAGE_COLOR, 1)
        if show_each:
            for ys, color in zip(each, SENSOR_COLORS):
                polyline(ys, py, color, 1)
        polyline(height, py, HEIGHT_COLOR, 2)

        # legend (the voltage line is identified by its orange axis title)
        x = x0 + 110
        items = [('height (average)', HEIGHT_COLOR)]
        if show_each:
            items += [(f'sensor {i + 1}', col) for i, col in enumerate(SENSOR_COLORS)]
        for label, color in items:
            c.create_line(x, y0 - 9, x + 14, y0 - 9, fill=color, width=2)
            item = c.create_text(x + 18, y0 - 9, text=label, anchor='w', fill='#333', font=font)
            x = c.bbox(item)[2] + 12


# ===========================================================================
# The window
# ===========================================================================

class App:
    def __init__(self, root, args):
        self.root = root
        self.args = args
        self.cal = Calibration(args.cal)
        self.q = queue.Queue()
        self.source = None
        self.processor = None
        # one entry per averaged sample: (time s, [4 readings], voltage, mode, firmware height %)
        self.history = deque(maxlen=int(MAX_HISTORY_S * 1000.0 / AVERAGE_MS))
        self.pending_v = None       # voltage waiting to be sent to the firmware
        self.last_send_time = 0.0
        self.last_rate_time = time.perf_counter()
        self.last_rate_lines = 0

        root.title('LeviTAS v1 - capacitance, height and high voltage')
        self._build(root)
        root.protocol('WM_DELETE_WINDOW', self.close)
        root.bind('<Escape>', lambda e: self.hv_off())
        self.refresh_ports()
        if args.simulate:
            self.connect()
        elif args.port:
            self.port_var.set(args.port)
            self.connect()
        root.after(REFRESH_MS, self.tick)

    # ----- building the window -----

    def _build(self, root):
        mono = ('Consolas', 10)
        panel = ttk.Frame(root, padding=8)
        panel.pack(side='left', fill='y')
        right = ttk.Frame(root, padding=(0, 8, 8, 8))
        right.pack(side='left', fill='both', expand=True)

        # connection
        f = ttk.LabelFrame(panel, text='Connection', padding=6)
        f.pack(fill='x')
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(f, textvariable=self.port_var, width=30)
        self.port_box.grid(row=0, column=0, columnspan=2, sticky='we')
        ttk.Button(f, text='Refresh', command=self.refresh_ports, width=8).grid(row=0, column=2, padx=(4, 0))
        self.connect_btn = ttk.Button(f, text='Connect', command=self.toggle_connection)
        self.connect_btn.grid(row=1, column=0, sticky='w', pady=(4, 0))
        self.status_var = tk.StringVar(value='not connected')
        ttk.Label(f, textvariable=self.status_var, wraplength=320, justify='left').grid(
            row=2, column=0, columnspan=3, sticky='w', pady=(4, 0))

        # high voltage
        f = ttk.LabelFrame(panel, text='High voltage on the mass (all four electrodes)', padding=6)
        f.pack(fill='x', pady=(8, 0))
        self.applied_var = tk.StringVar(value='applied:   --- V')
        ttk.Label(f, textvariable=self.applied_var, font=('Consolas', 16, 'bold'), foreground=VOLTAGE_COLOR).grid(
            row=0, column=0, columnspan=6, sticky='w')
        self.mode_var = tk.StringVar(value='firmware: ---')
        ttk.Label(f, textvariable=self.mode_var, foreground='#555').grid(row=1, column=0, columnspan=6, sticky='w')
        self.slider_var = tk.DoubleVar(value=0.0)
        self.slider = tk.Scale(f, from_=0, to=HV_MAX_V, orient='horizontal', resolution=1, length=320,
                               variable=self.slider_var, command=self.on_slider, tickinterval=40)
        self.slider.grid(row=2, column=0, columnspan=6, sticky='we', pady=(4, 0))
        ttk.Label(f, text='Set (V):').grid(row=3, column=0, sticky='w')
        self.entry_var = tk.StringVar(value='0')
        e = ttk.Entry(f, textvariable=self.entry_var, width=7)
        e.grid(row=3, column=1, sticky='w')
        e.bind('<Return>', lambda ev: self.set_from_entry())
        ttk.Button(f, text='Apply', width=6, command=self.set_from_entry).grid(row=3, column=2, sticky='w', padx=(4, 0))
        presets = ttk.Frame(f)
        presets.grid(row=4, column=0, columnspan=6, sticky='w', pady=(4, 0))
        for v in HV_PRESETS:
            ttk.Button(presets, text=str(v), width=5, command=lambda v=v: self.set_voltage(v)).pack(side='left', padx=(0, 2))
        tk.Button(f, text='HV OFF  (Esc)', command=self.hv_off, bg='#c62828', fg='white',
                  activebackground='#e53935', activeforeground='white',
                  font=('Segoe UI', 11, 'bold')).grid(row=5, column=0, columnspan=6, sticky='we', pady=(8, 0))
        ttk.Label(f, wraplength=320, justify='left', foreground='#555', text=(
            'The slider only works while the firmware is idle or in fixed voltage mode. '
            'HV OFF also stops the calibration and the PD controller.')).grid(
            row=6, column=0, columnspan=6, sticky='w', pady=(6, 0))

        # height calibration
        f = ttk.LabelFrame(panel, text='Height calibration', padding=6)
        f.pack(fill='x', pady=(8, 0))
        ttk.Button(f, text='Set BOTTOM', width=12, command=lambda: self.capture('bottom')).grid(row=0, column=0, sticky='w')
        ttk.Label(f, text='mass at rest, HV off', foreground='#555').grid(row=0, column=1, sticky='w', padx=(6, 0))
        ttk.Button(f, text='Set TOP', width=12, command=lambda: self.capture('top')).grid(row=1, column=0, sticky='w', pady=(2, 0))
        ttk.Label(f, text='mass against the top stop', foreground='#555').grid(row=1, column=1, sticky='w', padx=(6, 0))
        self.cal_var = tk.StringVar()
        ttk.Label(f, textvariable=self.cal_var, font=('Consolas', 9), foreground='#555', justify='left').grid(
            row=2, column=0, columnspan=3, sticky='w', pady=(4, 0))
        ttk.Label(f, text='Travel between stops (µm):').grid(row=3, column=0, columnspan=2, sticky='w', pady=(4, 0))
        self.travel_var = tk.StringVar(value=f'{self.cal.travel_um:g}')
        e = ttk.Entry(f, textvariable=self.travel_var, width=7)
        e.grid(row=3, column=2, sticky='w', pady=(4, 0))
        e.bind('<Return>', lambda ev: self.set_travel())
        e.bind('<FocusOut>', lambda ev: self.set_travel())
        ttk.Button(f, text='Back to defaults', command=self.reset_cal).grid(row=4, column=0, columnspan=2, sticky='w', pady=(4, 0))
        self.update_cal_label()

        # display
        f = ttk.LabelFrame(panel, text='Display', padding=6)
        f.pack(fill='x', pady=(8, 0))
        ttk.Label(f, text='Time shown (s)').grid(row=0, column=0, sticky='w')
        self.window_var = tk.DoubleVar(value=self.args.window)
        ttk.Spinbox(f, from_=1, to=MAX_HISTORY_S, increment=1, textvariable=self.window_var, width=6).grid(
            row=0, column=1, sticky='w', padx=(8, 0))
        self.show_each_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(f, text='show each sensor (tilt)', variable=self.show_each_var).grid(
            row=1, column=0, columnspan=2, sticky='w', pady=(4, 0))

        # other commands
        f = ttk.LabelFrame(panel, text='Other firmware command (e.g. cal, start, status, help)', padding=6)
        f.pack(fill='x', pady=(8, 0))
        self.command_var = tk.StringVar()
        e = ttk.Entry(f, textvariable=self.command_var, width=30)
        e.pack(side='left', fill='x', expand=True)
        e.bind('<Return>', lambda ev: self.send_typed_command())
        ttk.Button(f, text='Send', width=6, command=self.send_typed_command).pack(side='left', padx=(4, 0))

        # right side: plot, then sensors and messages
        self.plot = HeightPlot(right)
        self.plot.canvas.pack(fill='both', expand=True)
        below = ttk.Frame(right)
        below.pack(fill='x', pady=(8, 0))
        f = ttk.LabelFrame(below, text='Capacitance sensors (now)', padding=6)
        f.pack(side='left', fill='y')
        ttk.Label(f, font=mono, text=f"{'':9s}{'reading':>9s}{'capacitance':>13s}{'change':>11s}{'height':>10s}").pack(anchor='w')
        self.sensor_vars = []
        for i in range(NUM_CH):
            var = tk.StringVar()
            ttk.Label(f, textvariable=var, font=mono, foreground=SENSOR_COLORS[i]).pack(anchor='w')
            self.sensor_vars.append(var)
        self.height_var = tk.StringVar()
        ttk.Label(f, textvariable=self.height_var, font=('Consolas', 11, 'bold'), foreground=HEIGHT_COLOR).pack(anchor='w', pady=(4, 0))
        ttk.Label(f, foreground='#555', font=('Segoe UI', 8), text='change = capacitance minus its value at the bottom stop').pack(anchor='w')
        f = ttk.LabelFrame(below, text='Messages from the Pico', padding=6)
        f.pack(side='left', fill='both', expand=True, padx=(8, 0))
        self.msg = tk.Text(f, height=8, width=40, font=('Consolas', 9), state='disabled', wrap='word')
        self.msg.pack(fill='both', expand=True)

    # ----- connection -----

    def refresh_ports(self):
        ports = []
        if serial is not None:
            found = list(serial.tools.list_ports.comports())
            # Raspberry Pi Pico first (USB vendor id 0x2e8a)
            found.sort(key=lambda p: (p.vid != 0x2E8A, p.device))
            ports = [f'{p.device} - {p.description}' for p in found]
        self.port_box['values'] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def toggle_connection(self):
        if self.source is None:
            self.connect()
        else:
            self.disconnect()

    def connect(self):
        self.disconnect()
        self.processor = LineProcessor(self.q)
        if self.args.simulate:
            self.source = SimSource(self.processor)
            self.status_var.set('simulated data (no hardware)')
        else:
            if serial is None:
                self.status_var.set('the pyserial module is not installed (pip install pyserial)')
                return
            port = self.port_var.get().split(' - ')[0].strip()
            if not port:
                self.status_var.set('choose a serial port first')
                return
            self.source = SerialSource(port, self.processor)
            self.status_var.set(f'connected to {port}')
        self.history.clear()
        self.source.start()
        self.source.send(f'stream {STREAM_EVERY}')
        self.connect_btn.config(text='Disconnect')
        self.last_rate_time, self.last_rate_lines = time.perf_counter(), 0

    def disconnect(self):
        if self.source is None:
            return
        # leave the firmware safe and as we found it
        self.source.send('stop')
        self.source.send('stream 1')
        self.source.stop()
        self.source.join(timeout=1.0)
        self.source = None
        self.pending_v = None
        self.slider_var.set(0.0)
        self.entry_var.set('0')
        self.connect_btn.config(text='Connect')
        self.status_var.set('not connected')

    def close(self):
        self.disconnect()
        self.root.destroy()

    # ----- high voltage -----

    def on_slider(self, value):
        self.entry_var.set(f'{float(value):.0f}')
        self.pending_v = float(value)

    def set_voltage(self, v):
        v = min(max(float(v), 0.0), float(HV_MAX_V))
        self.slider_var.set(v)
        self.entry_var.set(f'{v:.0f}')
        self.pending_v = v

    def set_from_entry(self):
        try:
            self.set_voltage(float(self.entry_var.get()))
        except ValueError:
            self.entry_var.set(f'{self.slider_var.get():.0f}')

    def hv_off(self):
        self.pending_v = None
        self.slider_var.set(0.0)
        self.entry_var.set('0')
        if self.source is not None:
            self.source.send('stop')

    def send_typed_command(self):
        command = self.command_var.get().strip()
        if command and self.source is not None:
            self.source.send(command)
            self.add_message('> ' + command)
        self.command_var.set('')

    # ----- height calibration -----

    def recent_average(self, seconds):
        """Average reading of each sensor over the last `seconds` (None = no data)."""
        if not self.history:
            return [None] * NUM_CH
        t_end = self.history[-1][0]
        sums, counts = [0.0] * NUM_CH, [0] * NUM_CH
        for t, vals, _, _, _ in reversed(self.history):
            if t < t_end - seconds:
                break
            for i, v in enumerate(vals):
                if v is not None:
                    sums[i] += v
                    counts[i] += 1
        return [sums[i] / counts[i] if counts[i] else None for i in range(NUM_CH)]

    def capture(self, which):
        avg = self.recent_average(CAPTURE_S)
        if any(v is None for v in avg):
            self.add_message('cannot set the reference: no readings from all four sensors')
            return
        if which == 'bottom':
            self.cal.bottom, self.cal.bottom_set = avg, True
        else:
            self.cal.top, self.cal.top_set = avg, True
        self.cal.save()
        self.update_cal_label()

    def reset_cal(self):
        self.cal.reset()
        self.update_cal_label()

    def set_travel(self):
        try:
            v = float(self.travel_var.get())
            if v > 0:
                self.cal.travel_um = v
                self.cal.save()
        except ValueError:
            pass
        self.travel_var.set(f'{self.cal.travel_um:g}')

    def update_cal_label(self):
        def row(name, vals, is_set):
            return f"{name:7s}" + ' '.join(f'{v:6.0f}' for v in vals) + ('' if is_set else '  (default)')
        self.cal_var.set(row('bottom', self.cal.bottom, self.cal.bottom_set) + '\n' +
                         row('top', self.cal.top, self.cal.top_set))

    # ----- the regular update -----

    def add_message(self, text):
        self.msg.config(state='normal')
        self.msg.insert('end', text + '\n')
        if float(self.msg.index('end-1c').split('.')[0]) > 300:   # keep the last 300 lines
            self.msg.delete('1.0', '50.0')
        self.msg.see('end')
        self.msg.config(state='disabled')

    def tick(self):
        started = time.perf_counter()

        # take in everything that has arrived
        while True:
            try:
                kind, payload = self.q.get_nowait()
            except queue.Empty:
                break
            if kind == 'sample':
                self.history.append(payload)
            elif kind == 'info':
                if not payload.startswith('fixed voltage'):   # (echo of every slider move)
                    self.add_message(payload)
            elif kind == 'restart':
                self.history.clear()
                self.add_message('(the Pico was restarted)')
            elif kind == 'error':
                self.add_message('ERROR ' + payload)
                self.disconnect()
                self.status_var.set('connection lost: ' + payload)

        # send the newest slider position (not more often than HV_SEND_INTERVAL_S)
        if self.pending_v is not None and self.source is not None and started - self.last_send_time >= HV_SEND_INTERVAL_S:
            self.source.send(f'hv {self.pending_v:.0f}')
            self.pending_v = None
            self.last_send_time = started

        self.update_readouts()
        self.draw_plot()

        # data rate, once per second
        if self.processor is not None and started - self.last_rate_time >= 1.0 and self.source is not None:
            rate = (self.processor.n_lines - self.last_rate_lines) / (started - self.last_rate_time)
            self.last_rate_time, self.last_rate_lines = started, self.processor.n_lines
            where = 'simulated data' if self.args.simulate else f'connected to {self.source.port}'
            self.status_var.set(f'{where}: {rate:.0f} lines/s' + ('' if rate > 0 else '  (NO DATA)'))

        spent_ms = (time.perf_counter() - started) * 1000.0
        self.root.after(max(5, int(REFRESH_MS - spent_ms)), self.tick)

    def update_readouts(self):
        if not self.history:
            for i, var in enumerate(self.sensor_vars):
                var.set(f'sensor {i + 1}      ---')
            self.height_var.set('height z:    --- µm')
            self.applied_var.set('applied:   --- V')
            self.mode_var.set('firmware: ---')
            return
        avg = self.recent_average(READOUT_S)
        height, each = self.cal.height_um(avg)
        for i, var in enumerate(self.sensor_vars):
            if avg[i] is None:
                var.set(f'sensor {i + 1}   no reading')
                continue
            pf = counts_to_pf(avg[i])
            change_ff = (pf - counts_to_pf(self.cal.bottom[i])) * 1000.0
            h = '      ---' if each[i] is None else f'{each[i]:7.1f} µm'
            var.set(f'sensor {i + 1} {avg[i]:8.1f} {pf:9.3f} pF {change_ff:+8.0f} fF {h}')
        if height is None:
            self.height_var.set('height z:    --- µm')
        else:
            percent = 100.0 * height / self.cal.travel_um
            self.height_var.set(f'height z: {height:6.1f} µm  ({percent:5.1f} % of travel)')
        _, _, hv, mode, fw_height = self.history[-1]
        self.applied_var.set(f'applied: {hv:5.1f} V')
        text = 'firmware: ' + MODE_NAMES.get(mode, '---' if mode is None else f'mode {mode}')
        if fw_height is not None and fw_height >= 0:
            text += f'   (its own height: {fw_height:.1f} %)'
        self.mode_var.set(text)

    def draw_plot(self):
        try:
            window_s = min(max(float(self.window_var.get()), 1.0), MAX_HISTORY_S)
        except (tk.TclError, ValueError):
            window_s = DEFAULT_WINDOW_S
        ts, height, volts = [], [], []
        each = [[] for _ in range(NUM_CH)]
        if self.history:
            t_end = self.history[-1][0]
            n = min(len(self.history), int(window_s * 1000.0 / AVERAGE_MS) + 1)
            stride = max(1, n // MAX_PLOT_POINTS)
            for k in range(len(self.history) - n, len(self.history), stride):
                t, vals, hv, _, _ = self.history[k]
                h, e = self.cal.height_um(vals)
                ts.append(t - t_end)
                height.append(h)
                volts.append(hv)
                for i in range(NUM_CH):
                    each[i].append(e[i])
        self.plot.draw(ts, height, each, volts, window_s, self.cal.travel_um, self.show_each_var.get())


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description='LeviTAS v1 monitor and manual high voltage control')
    ap.add_argument('--port', help='serial port to connect to at startup (e.g. COM3)')
    ap.add_argument('--simulate', action='store_true', help='use made-up data instead of a serial port')
    ap.add_argument('--cal', default=os.path.join(here, 'levitas_v1_calibration.json'),
                    help='file the height calibration is saved in')
    ap.add_argument('--window', type=float, default=DEFAULT_WINDOW_S,
                    help='length of time shown in the plot, in seconds')
    args = ap.parse_args()

    root = tk.Tk()
    App(root, args)
    root.mainloop()


if __name__ == '__main__':
    main()
