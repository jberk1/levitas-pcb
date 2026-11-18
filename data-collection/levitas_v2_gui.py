#!/usr/bin/env python3
"""
LeviTAS v2 position monitor.

Reads the eight FDC2112 capacitance readings that the v2 firmware
(firmware_v2/src/main_v2.c) prints over USB serial, converts them to the
position of the levitating mass, and shows three windows:

  1. a 3D view of the mass (plus the controls and calibration buttons)
  2. translation along x, y, z versus time
  3. rotation about x, y, z versus time

Run with:

    python levitas_v2_gui.py                 (choose the serial port in the window)
    python levitas_v2_gui.py --port COM5     (connect to COM5 right away)
    python levitas_v2_gui.py --simulate      (no hardware: made-up data, to try the program)

Needs the "pyserial" module (pip install pyserial) to talk to the Pico.
Everything else is from the Python standard library (tkinter).

------------------------------------------------------------------------------
How readings are turned into position
------------------------------------------------------------------------------
(Following the "Position detection of the proof mass" section of the paper.)

Each FDC2112 measures the capacitance between two sense electrodes, coupled
through the floating mass. Sensor n in the firmware output is channel CHn in
the paper (wiring from the KiCad netlist levitas_v2.net):

    CH1  U2  XS_Q4    - YS_Q1         CH5  U6  ZS_XP (+x arm)
    CH2  U3  YS_Q4    - XS_Q1         CH6  U7  ZS_XM (-x arm)
    CH3  U4  YS_Q2    - XS_Q3         CH7  U8  ZS_YP (+y arm)
    CH4  U5  XS_Q2    - YS_Q3         CH8  U9  ZS_YM (-y arm)

The six motions are proportional to these combinations (paper eq. 4-9):

    x       ~  C1 + C2 - C3 - C4          z       ~  C5 + C6 - C7 - C8
    y       ~  C1 - C2 + C3 - C4          theta_x ~  C8 - C7
    theta_z ~  C1 - C2 - C3 + C4          theta_y ~  C6 - C5

The scale and zero of x, y and z come from a two-point calibration: tilt the
device so the mass rests against the stop at one end of its travel, press
"Set MIN", tilt it the other way, press "Set MAX". Until an axis is
calibrated, the sensitivity from Table 1 of the paper is used instead (this
is only approximate because it depends on the exact oscillator frequency and
inductor value). The rotations use the same sensitivities together with the
lever arms below.
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
NUM_CH = 8

# ----- FDC2112 settings: these must match firmware_v2/src/main_v2.c -----
FDC_OFFSET = 0x1000        # FDC2112_OFFSET (printed value = 16-bit data - offset)
FDC_FIN_SEL = 2            # from FDC2112_CLOCK_DIVIDERS = 0x2002
FDC_FREF_DIVIDER = 2       # from FDC2112_CLOCK_DIVIDERS = 0x2002
FDC_F_CLK_HZ = 43.4e6      # internal oscillator (nominal; 35-55 MHz chip to chip)
TANK_L_H = 18e-6           # inductor across each sensor (L1..L8 = 18 uH)
READ_MAX = 0x0FFF          # largest real reading (0xffff = read failed)

# ----- device numbers from Table 1 of the paper -----
SENS_XY_PF_PER_UM = 0.583  # one comb electrode: 0.583 fF/nm
SENS_Z_PF_PER_UM = 15.0    # one parallel plate electrode: 15.0 fF/nm
# travel of the mass between its stops (um): (minimum, maximum)
# These are the positions assigned to "Set MIN" and "Set MAX". They can be
# changed in the window.
TRAVEL_UM = {'x': (-3.5, 3.5), 'y': (-3.5, 3.5), 'z': (-1.0, 1.0)}

# ----- lever arms (mm), only used for the three rotations -----
# Distance from the centre of the mass to the centre of each electrode,
# measured across the direction the electrode senses.
# NOTE: THESE ARE ESTIMATES. Replace them with the values from the layout
# to get correct rotation angles (translations do not depend on them).
ARM_XCOMB_MM = 2.5         # x-sensing combs: distance from the x axis
ARM_YCOMB_MM = 2.5         # y-sensing combs: distance from the y axis
ARM_ZPLATE_X_MM = 4.0      # z plates on the +x / -x arms: distance from centre
ARM_ZPLATE_Y_MM = 4.0      # z plates on the +y / -y arms: distance from centre

# ----- display -----
AVERAGE_MS = 5.0           # readings are averaged over this long before use
DEFAULT_WINDOW_S = 10.0    # length of time shown in the plots
MAX_HISTORY_S = 60.0
CAPTURE_S = 0.5            # calibration buttons average the last 0.5 s
REFRESH_MS = 40            # redraw interval (25 frames per second)
MAX_PLOT_POINTS = 400      # points drawn per plot (more than this is slow to paint)

CHANNELS = [  # (name, chip, electrodes)
    ('CH1', 'U2', 'XS_Q4 - YS_Q1'),
    ('CH2', 'U3', 'YS_Q4 - XS_Q1'),
    ('CH3', 'U4', 'YS_Q2 - XS_Q3'),
    ('CH4', 'U5', 'XS_Q2 - YS_Q3'),
    ('CH5', 'U6', 'ZS_XP (+x arm)'),
    ('CH6', 'U7', 'ZS_XM (-x arm)'),
    ('CH7', 'U8', 'ZS_YP (+y arm)'),
    ('CH8', 'U9', 'ZS_YM (-y arm)'),
]

# Order of the six motions everywhere in this program.
# (tx, ty, tz = rotation about x, y, z)
AXES = ('x', 'y', 'z', 'tx', 'ty', 'tz')
AXIS_INDEX = {a: i for i, a in enumerate(AXES)}

# Channel combinations, paper eq. 4-9 (weights for CH1..CH8).
WEIGHTS = {
    'x':  (+1, +1, -1, -1, 0, 0, 0, 0),    # eq. 4
    'y':  (+1, -1, +1, -1, 0, 0, 0, 0),    # eq. 5
    'tz': (+1, -1, -1, +1, 0, 0, 0, 0),    # eq. 6
    'z':  (0, 0, 0, 0, +1, +1, -1, -1),    # eq. 7
    'tx': (0, 0, 0, 0, 0, 0, -1, +1),      # eq. 8
    'ty': (0, 0, 0, 0, -1, +1, 0, 0),      # eq. 9
}
SIGNAL_FORMULA = {
    'x': 'C1+C2-C3-C4', 'y': 'C1-C2+C3-C4', 'z': 'C5+C6-C7-C8',
    'tx': 'C8-C7', 'ty': 'C6-C5', 'tz': 'C1-C2-C3+C4',
}
AXIS_LABEL = {
    'x': 'x', 'y': 'y', 'z': 'z',
    'tx': 'θx (about x)', 'ty': 'θy (about y)', 'tz': 'θz (about z)',
}
AXIS_UNIT = {'x': 'µm', 'y': 'µm', 'z': 'µm', 'tx': 'mrad', 'ty': 'mrad', 'tz': 'mrad'}
AXIS_COLOR = {'x': '#d62728', 'y': '#2ca02c', 'z': '#1f77b4',
              'tx': '#d62728', 'ty': '#2ca02c', 'tz': '#1f77b4'}

# Sensitivity used for an axis that has not been calibrated (um per pF of the
# channel combination). A channel sees a quarter of the change of each of its
# two electrodes (paper eq. 3), so:
#   x: four channels, one x comb each     -> combination = S_xy * x
#   z: four channels, two z plates each   -> combination = 2 * S_z * z
THEORY_K = {
    'x': 1.0 / SENS_XY_PF_PER_UM,
    'y': 1.0 / SENS_XY_PF_PER_UM,
    'z': 1.0 / (2.0 * SENS_Z_PF_PER_UM),
}


# ===========================================================================
# Readings -> capacitance -> channel combinations
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


def pf_to_counts(c_pf):
    """Inverse of counts_to_pf (used by the simulator)."""
    f = 1.0 / (2.0 * math.pi * math.sqrt(TANK_L_H * c_pf * 1e-12))
    return f * 65536.0 / (FDC_FIN_SEL * (FDC_F_CLK_HZ / FDC_FREF_DIVIDER)) - FDC_OFFSET


def axis_signals(caps_pf):
    """The six channel combinations (pF), in the order of AXES.

    caps_pf: list of 8 capacitances (None for a sensor without data).
    An entry of the result is None if one of the channels it needs is None.
    """
    out = []
    for a in AXES:
        total = 0.0
        for w, c in zip(WEIGHTS[a], caps_pf):
            if w:
                if c is None:
                    total = None
                    break
                total += w * c
        out.append(total)
    return tuple(out)


# ===========================================================================
# Calibration
# ===========================================================================

class Calibration:
    """Two-point calibration of x, y, z (saved in a JSON file)."""

    def __init__(self, path):
        self.path = path
        self.axes = {a: {'min_sig': None, 'max_sig': None,
                         'min_um': TRAVEL_UM[a][0], 'max_um': TRAVEL_UM[a][1]}
                     for a in ('x', 'y', 'z')}
        # channel combinations (pF) at the pose that counts as zero. Used for
        # the rotations and for translation axes that are not calibrated.
        self.zero = None
        self.load()

    def load(self):
        if not self.path or not os.path.exists(self.path):
            return
        try:
            with open(self.path, 'r', encoding='utf-8') as f:
                d = json.load(f)
            for a in ('x', 'y', 'z'):
                for key in ('min_sig', 'max_sig', 'min_um', 'max_um'):
                    v = d.get('axes', {}).get(a, {}).get(key, self.axes[a][key])
                    self.axes[a][key] = None if v is None else float(v)
            z = d.get('zero')
            if isinstance(z, list) and len(z) == len(AXES):
                self.zero = [None if v is None else float(v) for v in z]
        except (OSError, ValueError, TypeError) as e:
            print(f'could not read calibration file {self.path}: {e}')

    def save(self):
        if not self.path:
            return
        try:
            with open(self.path, 'w', encoding='utf-8') as f:
                json.dump({'version': 1, 'axes': self.axes, 'zero': self.zero}, f, indent=2)
        except OSError as e:
            print(f'could not write calibration file {self.path}: {e}')

    def is_calibrated(self, axis):
        c = self.axes[axis]
        return (c['min_sig'] is not None and c['max_sig'] is not None
                and abs(c['max_sig'] - c['min_sig']) > 1e-6
                and abs(c['max_um'] - c['min_um']) > 1e-9)

    def gain(self, axis):
        """um per pF of the channel combination, for x, y or z."""
        c = self.axes[axis]
        if self.is_calibrated(axis):
            return (c['max_um'] - c['min_um']) / (c['max_sig'] - c['min_sig'])
        return THEORY_K[axis]

    def solver(self):
        """Return a function: six channel combinations (pF) -> six positions.

        Positions are (x, y, z) in um and (theta_x, theta_y, theta_z) in mrad,
        None where the data is missing.
        """
        zero = [0.0 if v is None else v for v in (self.zero or [None] * 6)]
        k = {a: self.gain(a) for a in ('x', 'y', 'z')}
        offset = {}
        for a in ('x', 'y', 'z'):
            c = self.axes[a]
            if self.is_calibrated(a):
                # straight line through (min_sig, min_um) and (max_sig, max_um)
                offset[a] = c['min_um'] - c['min_sig'] * k[a]
            else:
                offset[a] = -zero[AXIS_INDEX[a]] * k[a]

        # Rotations (small angles; um / mm = mrad).
        #
        # theta_z: every in-plane channel has an x comb and a y comb. A
        # rotation moves the x combs by theta*ARM_XCOMB and the y combs by
        # theta*ARM_YCOMB, so  C1-C2-C3+C4 = theta * (ARM_XCOMB/kx + ARM_YCOMB/ky)
        kx, ky = k['x'], k['y']
        if kx * ky < 0:
            # x and y calibrated with opposite signs: follow x
            ky = math.copysign(ky, kx)
        g_tz = 1.0 / (ARM_XCOMB_MM / kx + ARM_YCOMB_MM / ky)
        # theta_y: the +x and -x arm ends move by -/+ theta*ARM_ZPLATE_X in z,
        # so  C6-C5 = theta * ARM_ZPLATE_X / (2 kz).  Same for theta_x.
        g_ty = 2.0 * k['z'] / ARM_ZPLATE_X_MM
        g_tx = 2.0 * k['z'] / ARM_ZPLATE_Y_MM

        gains = (k['x'], k['y'], k['z'], g_tx, g_ty, g_tz)
        offsets = (offset['x'], offset['y'], offset['z'],
                   -zero[3] * g_tx, -zero[4] * g_ty, -zero[5] * g_tz)

        def pose(sig):
            return tuple(None if s is None else s * g + o
                         for s, g, o in zip(sig, gains, offsets))
        return pose


# ===========================================================================
# Serial data: parsing and averaging
# ===========================================================================

class LineProcessor:
    """Turns the bytes from the Pico into averaged readings.

    The firmware prints about 6000 lines per second, each one
        <microseconds> 0x.... 0x.... 0x.... 0x.... 0x.... 0x.... 0x.... 0x....
    Readings are averaged over AVERAGE_MS and put on a queue as
        ('sample', (time in seconds, [8 average readings or None]))
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
        if len(parts) != 1 + NUM_CH:
            self.n_bad += 1
            return
        try:
            t = int(parts[0])
            vals = [int(p, 16) for p in parts[1:]]
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
            if v <= READ_MAX:          # (0xffff = read failed: leave it out)
                self.sums[i] += v
                self.counts[i] += 1
        if t - self.block_t0 >= self.average_us:
            avg = [self.sums[i] / self.counts[i] if self.counts[i] else None
                   for i in range(NUM_CH)]
            self.q.put(('sample', ((self.block_t0 + t) / 2e6, avg)))
            self._start_block(None)


class SerialSource(threading.Thread):
    """Reads from the serial port in the background."""

    def __init__(self, port, processor):
        super().__init__(daemon=True)
        self.port = port
        self.processor = processor
        self.stop_event = threading.Event()

    def run(self):
        try:
            with serial.Serial(self.port, BAUD, timeout=0.1) as ser:
                while not self.stop_event.is_set():
                    data = ser.read(ser.in_waiting or 1)
                    if data:
                        self.processor.feed(data)
        except Exception as e:  # port unplugged, access denied, ...
            self.processor.q.put(('error', f'{self.port}: {e}'))

    def stop(self):
        self.stop_event.set()


class SimSource(threading.Thread):
    """Made-up data in the same format as the firmware, to try the program
    without hardware. The mass wanders around slowly; set `pose_override` to
    a tuple (x, y, z in um, theta_x, theta_y, theta_z in mrad) to hold it
    somewhere instead."""

    # readings measured on the real PCB (16-bit data), used as the baseline
    BASE_DATA16 = (0x1468, 0x152f, 0x1469, 0x1563, 0x1a5d, 0x1af2, 0x1534, 0x1473)
    RATE_HZ = 1000
    NOISE_COUNTS = 0.7

    def __init__(self, processor):
        super().__init__(daemon=True)
        self.processor = processor
        self.stop_event = threading.Event()
        self.pose_override = None
        self.base_pf = [counts_to_pf(d - FDC_OFFSET) for d in self.BASE_DATA16]

    @staticmethod
    def wander(t):
        return (3.0 * math.sin(2 * math.pi * 0.20 * t),
                2.5 * math.sin(2 * math.pi * 0.13 * t + 1.0),
                0.7 * math.sin(2 * math.pi * 0.31 * t + 2.0),
                0.10 * math.sin(2 * math.pi * 0.23 * t),
                0.12 * math.sin(2 * math.pi * 0.17 * t + 0.5),
                0.40 * math.sin(2 * math.pi * 0.11 * t + 1.5))

    def caps(self, pose):
        """Channel capacitances (pF) for a pose, from the same linear model
        the paper uses (each channel = 1/4 of the change of its electrodes)."""
        x, y, z, tx, ty, tz = pose
        s = SENS_XY_PF_PER_UM / 4.0
        rot = (ARM_XCOMB_MM + ARM_YCOMB_MM) * tz
        sz = SENS_Z_PF_PER_UM / 2.0
        delta = (
            s * (+x + y + rot),
            s * (+x - y - rot),
            s * (-x + y - rot),
            s * (-x - y + rot),
            +sz * (z - ARM_ZPLATE_X_MM * ty),
            +sz * (z + ARM_ZPLATE_X_MM * ty),
            -sz * (z + ARM_ZPLATE_Y_MM * tx),
            -sz * (z - ARM_ZPLATE_Y_MM * tx),
        )
        return [b + d for b, d in zip(self.base_pf, delta)]

    def line(self, t):
        pose = self.pose_override or self.wander(t)
        vals = []
        for c in self.caps(pose):
            v = int(round(pf_to_counts(c) + random.gauss(0.0, self.NOISE_COUNTS)))
            vals.append(min(max(v, 0), READ_MAX))
        return ('%d ' % int(t * 1e6) + ' '.join('0x%04x' % v for v in vals) + '\n').encode()

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
# Drawing: plots versus time
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


class StripChart:
    """One plot of a value versus time, drawn on a tkinter canvas."""

    LEFT, RIGHT, TOP, BOTTOM = 66, 14, 24, 24

    def __init__(self, parent, color):
        self.color = color
        # (small requested size: the plots in a window then share its height equally)
        self.canvas = tk.Canvas(parent, bg='white', highlightthickness=0, width=420, height=110)

    def draw(self, title, unit, ts, ys, window_s, min_span, hlines=()):
        """ts: times in seconds relative to now (<= 0). ys: values (None =
        no data). hlines: [(value, label)] dashed lines, e.g. travel limits."""
        c = self.canvas
        c.delete('all')
        w, h = c.winfo_width(), c.winfo_height()
        if w < 120 or h < 80:
            return
        x0, x1 = self.LEFT, w - self.RIGHT
        y0, y1 = self.TOP, h - self.BOTTOM

        # vertical range: the data (and limit lines), with a little margin
        valid = [y for y in ys if y is not None]
        if valid:
            lo, hi = min(valid), max(valid)
        else:
            lo, hi = -min_span / 2, min_span / 2
        for v, _ in hlines:
            lo, hi = min(lo, v), max(hi, v)
        if hi - lo < min_span:
            mid = (hi + lo) / 2
            lo, hi = mid - min_span / 2, mid + min_span / 2
        pad = 0.08 * (hi - lo)
        lo, hi = lo - pad, hi + pad

        def px(t):
            return x0 + (t + window_s) / window_s * (x1 - x0)

        def py(v):
            return y1 - (v - lo) / (hi - lo) * (y1 - y0)

        # grid and tick labels
        yticks, ydec = nice_ticks(lo, hi, max(3, (y1 - y0) // 28))
        for v in yticks:
            y = py(v)
            c.create_line(x0, y, x1, y, fill='#e6e6e6')
            c.create_text(x0 - 6, y, text=f'{v:.{ydec}f}', anchor='e', fill='#444', font=('Segoe UI', 8))
        xticks, xdec = nice_ticks(-window_s, 0.0, max(2, (x1 - x0) // 90))
        for t in xticks:
            x = px(t)
            c.create_line(x, y0, x, y1, fill='#e6e6e6')
            c.create_text(x, y1 + 4, text=f'{t:.{xdec}f}', anchor='n', fill='#444', font=('Segoe UI', 8))
        c.create_rectangle(x0, y0, x1, y1, outline='#888')
        c.create_text(x0 - 10, y1 + 4, text='time (s)', anchor='ne', fill='#666', font=('Segoe UI', 8))

        # dashed limit lines
        for v, label in hlines:
            y = py(v)
            c.create_line(x0, y, x1, y, fill='#999', dash=(4, 3))
            c.create_text(x1 - 4, y - 1, text=label, anchor='se', fill='#777', font=('Segoe UI', 8))

        # the data (a gap wherever there is no value)
        seg = []
        for t, v in zip(ts, ys):
            if v is None or t < -window_s:
                if len(seg) >= 4:
                    c.create_line(*seg, fill=self.color)
                seg = []
            else:
                seg.extend((px(t), py(v)))
        if len(seg) >= 4:
            c.create_line(*seg, fill=self.color)

        # title and current value
        c.create_text(x0, 4, text=f'{title} ({unit})', anchor='nw', fill='#111', font=('Segoe UI', 10, 'bold'))
        last = next((v for v in reversed(ys) if v is not None), None)
        text = 'no data' if last is None else f'{last:+.4f} {unit}'
        c.create_text(x1, 4, text=text, anchor='ne', fill=self.color, font=('Consolas', 10, 'bold'))


# ===========================================================================
# Drawing: 3D view of the mass
# ===========================================================================

def rotation_matrix(rx, ry, rz):
    """Rotation by rx about x, then ry about y, then rz about z (radians)."""
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    return (
        (cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx),
        (sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx),
        (-sy, cy * sx, cy * cx),
    )


def mat_vec(m, v):
    return (m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
            m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
            m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2])


def shade(rgb, brightness):
    return '#%02x%02x%02x' % tuple(min(255, max(0, int(v * brightness))) for v in rgb)


class View3D:
    """Draws the two-layer mass at its measured position.

    The real movement (a few um on a 10 mm part) is far too small to see, so
    the movement is drawn larger by the factor `exaggeration`. The cavity
    outlines are drawn larger by the same factor, so the mass touches an
    outline when it reaches the end of its x/y travel.
    """

    # drawing dimensions in mm (thickness is not to scale: the real silicon
    # layers are 0.04 mm thick)
    HALF_LONG, HALF_SHORT, THICK = 5.0, 2.5, 0.45
    FACES = (  # (corner indices, outward normal)
        ((0, 3, 2, 1), (0, 0, -1)), ((4, 5, 6, 7), (0, 0, 1)),
        ((0, 1, 5, 4), (0, -1, 0)), ((2, 3, 7, 6), (0, 1, 0)),
        ((0, 4, 7, 3), (-1, 0, 0)), ((1, 2, 6, 5), (1, 0, 0)),
    )
    LIGHT = (-0.37, -0.56, 0.74)

    def __init__(self, parent):
        self.canvas = tk.Canvas(parent, bg='#f6f7f9', highlightthickness=0, width=600, height=560)
        self.az = math.radians(-35.0)
        self.el = math.radians(30.0)
        self.exaggeration = 300.0
        self._drag = None
        self.canvas.bind('<ButtonPress-1>', self._press)
        self.canvas.bind('<B1-Motion>', self._motion)

    def _press(self, e):
        self._drag = (e.x, e.y)

    def _motion(self, e):
        if self._drag is None:
            return
        dx, dy = e.x - self._drag[0], e.y - self._drag[1]
        self._drag = (e.x, e.y)
        self.az += math.radians(dx * 0.5)
        self.el = min(math.radians(89.0), max(math.radians(3.0), self.el + math.radians(dy * 0.5)))

    @staticmethod
    def box(xh, yh, z0, z1):
        return [(-xh, -yh, z0), (xh, -yh, z0), (xh, yh, z0), (-xh, yh, z0),
                (-xh, -yh, z1), (xh, -yh, z1), (xh, yh, z1), (-xh, yh, z1)]

    def draw(self, pose, travel_xy_um, status):
        """pose: (x, y, z um, theta_x, theta_y, theta_z mrad), entries may be
        None. travel_xy_um: (x travel, y travel) from centre to stop."""
        c = self.canvas
        c.delete('all')
        w, h = c.winfo_width(), c.winfo_height()
        if w < 120 or h < 120:
            return
        cx, cy = w / 2, h / 2 + 10
        scale = min(w, h) / 19.0   # pixels per mm
        ca, sa = math.cos(self.az), math.sin(self.az)
        ce, se = math.cos(self.el), math.sin(self.el)

        def project(p):
            xr = ca * p[0] - sa * p[1]
            yr = sa * p[0] + ca * p[1]
            # (screen x, screen y, depth: larger = further from the viewer)
            return (cx + scale * xr, cy - scale * (p[2] * ce + yr * se), yr * ce - p[2] * se)

        def line(p, q, **kw):
            a, b = project(p), project(q)
            c.create_line(a[0], a[1], b[0], b[1], **kw)

        ex = self.exaggeration
        T = self.THICK

        # ----- fixed parts: chip outline, cavity outlines, axes -----
        o = 7.4
        outline = [(-o, -o, -T), (o, -o, -T), (o, o, -T), (-o, o, -T)]
        for i in range(4):
            line(outline[i], outline[(i + 1) % 4], fill='#b9bec6')
        gx = travel_xy_um[0] * 1e-3 * ex   # drawn size of the x/y travel (mm)
        gy = travel_xy_um[1] * 1e-3 * ex
        for xh, yh, z, col in ((self.HALF_LONG + gx, self.HALF_SHORT + gy, -T / 2, '#5b7fa6'),
                               (self.HALF_SHORT + gx, self.HALF_LONG + gy, +T / 2, '#b9772e')):
            pts = [(-xh, -yh, z), (xh, -yh, z), (xh, yh, z), (-xh, yh, z)]
            for i in range(4):
                line(pts[i], pts[(i + 1) % 4], fill=col, dash=(5, 3))
        axis_len = 8.6
        for vec, col, name in (((axis_len, 0, 0), '#d62728', '+x'),
                               ((0, axis_len, 0), '#2ca02c', '+y'),
                               ((0, 0, axis_len * 0.55), '#1f77b4', '+z')):
            line((0, 0, 0), vec, fill=col, width=2, arrow='last')
            tip = project(tuple(v * 1.07 for v in vec))
            c.create_text(tip[0], tip[1], text=name, fill=col, font=('Segoe UI', 10, 'bold'))

        # ----- the mass -----
        have = [v is not None for v in pose]
        p = [0.0 if v is None else v for v in pose]
        clipped = False
        d = []
        for v in p[:3]:                       # um -> drawn mm
            dv = v * 1e-3 * ex
            if abs(dv) > 4.0:
                dv, clipped = math.copysign(4.0, dv), True
            d.append(dv)
        ang = []
        for v in p[3:]:                       # mrad -> drawn radians
            av = v * 1e-3 * ex
            if abs(av) > 0.6:
                av, clipped = math.copysign(0.6, av), True
            ang.append(av)
        R = rotation_matrix(*ang)

        # lower layer (SOI1): long along x. upper layer (SOI2): long along y.
        layers = [(self.box(self.HALF_LONG, self.HALF_SHORT, -T, 0.0), (108, 150, 205)),
                  (self.box(self.HALF_SHORT, self.HALF_LONG, 0.0, T), (236, 160, 80))]
        # Draw the layer that is further from the viewer first, and of each
        # layer only the faces that point toward the viewer.
        toward_viewer = (-sa * ce, -ca * ce, se)

        def facing(n):
            return n[0] * toward_viewer[0] + n[1] * toward_viewer[1] + n[2] * toward_viewer[2]

        if facing(mat_vec(R, (0, 0, 1))) < 0:   # seen from underneath
            layers.reverse()
        for corners, rgb in layers:
            moved = []
            for v in corners:
                r = mat_vec(R, v)
                moved.append(project((r[0] + d[0], r[1] + d[1], r[2] + d[2])))
            for idx, normal in self.FACES:
                n = mat_vec(R, normal)
                if facing(n) <= 0:
                    continue
                light = max(0.0, n[0] * self.LIGHT[0] + n[1] * self.LIGHT[1] + n[2] * self.LIGHT[2])
                pts = [v for i in idx for v in moved[i][:2]]
                c.create_polygon(*pts, fill=shade(rgb, 0.62 + 0.42 * light), outline='#3a3f47')

        # ----- text -----
        names = ('x ', 'y ', 'z ', 'θx', 'θy', 'θz')
        units = ('µm', 'µm', 'µm', 'mrad', 'mrad', 'mrad')
        for i in range(6):
            txt = f'{names[i]} = {p[i]:+9.4f} {units[i]}' if have[i] else f'{names[i]} =   no data'
            c.create_text(12, 10 + 17 * i, text=txt, anchor='nw', fill='#222', font=('Consolas', 10))
        note = f'movement drawn ×{ex:.0f} (thickness not to scale)'
        if clipped:
            note += '  -  TOO LARGE TO DRAW, CLIPPED'
        c.create_text(12, h - 8, text=note, anchor='sw', fill='#c0392b' if clipped else '#666', font=('Segoe UI', 9))
        c.create_text(w - 12, h - 8, text='drag to rotate the view', anchor='se', fill='#888', font=('Segoe UI', 9))
        c.create_text(w - 12, 10, text=status, anchor='ne', fill='#666', font=('Segoe UI', 9))


# ===========================================================================
# The program
# ===========================================================================

SIMULATED = 'Simulated data (no hardware)'


class App:
    def __init__(self, root, args):
        self.root = root
        self.args = args
        self.cal = Calibration(args.cal)
        self.q = queue.SimpleQueue()
        self.processor = None
        self.source = None
        self.history = deque()        # (time s, six channel combinations in pF)
        self.t_latest = None
        self.last_wall = 0.0          # when the last sample arrived
        self.latest_counts = [None] * NUM_CH
        self.rate_mark = (time.monotonic(), 0)
        self.tick_count = 0

        root.title('LeviTAS v2 - 3D view and controls')
        panel = ttk.Frame(root, padding=8)
        panel.pack(side='right', fill='y')
        left = ttk.Frame(root)
        left.pack(side='left', fill='both', expand=True)
        below = ttk.Frame(left, padding=(8, 0, 0, 8))
        below.pack(side='bottom', fill='x')
        self.view = View3D(left)
        self.view.canvas.pack(side='top', fill='both', expand=True)
        self._build_panel(panel, below)

        self.win_trans, self.charts_trans = self._plot_window('LeviTAS v2 - translation', ('x', 'y', 'z'))
        self.win_rot, self.charts_rot = self._plot_window('LeviTAS v2 - rotation', ('tx', 'ty', 'tz'))
        root.update_idletasks()
        self._place_windows()

        root.protocol('WM_DELETE_WINDOW', self.quit)
        self.refresh_ports()
        if args.simulate:
            self.port_var.set(SIMULATED)
            self.connect()
        elif args.port:
            self.port_var.set(args.port)
            self.connect()
        self.update_cal_labels()
        root.after(REFRESH_MS, self.tick)

    # ----- windows -----

    def _plot_window(self, title, axes):
        win = tk.Toplevel(self.root)
        win.title(title)
        win.protocol('WM_DELETE_WINDOW', win.withdraw)   # closing only hides it
        charts = {}
        for a in axes:
            ch = StripChart(win, AXIS_COLOR[a])
            ch.canvas.pack(fill='both', expand=True)
            charts[a] = ch
        return win, charts

    def _place_windows(self):
        # main window on the left, the two plot windows stacked on the right
        sw, sh = self.root.winfo_screenwidth(), self.root.winfo_screenheight()
        rw = int(min(1100, max(860, 0.58 * sw), sw - 40))
        rh = min(840, sh - 110)
        self.root.geometry(f'{rw}x{rh}+10+10')
        pw = min(760, max(420, sw - rw - 40))
        ph = min(470, (sh - 140) // 2)
        px = max(10, min(rw + 20, sw - pw - 10))
        self.win_trans.geometry(f'{pw}x{ph}+{px}+10')
        self.win_rot.geometry(f'{pw}x{ph}+{px}+{ph + 50}')

    def show_plot_windows(self):
        for win in (self.win_trans, self.win_rot):
            win.deiconify()
            win.lift()

    # ----- control panel -----

    def _build_panel(self, panel, below):
        """panel: column to the right of the 3D view (connection, display,
        calibration). below: strip under the 3D view (sensors, messages)."""
        mono = ('Consolas', 9)

        # connection
        f = ttk.LabelFrame(panel, text='Connection', padding=6)
        f.pack(fill='x')
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(f, textvariable=self.port_var, width=34)
        self.port_box.grid(row=0, column=0, columnspan=2, sticky='ew')
        ttk.Button(f, text='Refresh', command=self.refresh_ports, width=8).grid(row=0, column=2, padx=(4, 0))
        self.connect_btn = ttk.Button(f, text='Connect', command=self.toggle_connection)
        self.connect_btn.grid(row=1, column=0, sticky='w', pady=(4, 0))
        self.status_var = tk.StringVar(value='not connected')
        ttk.Label(f, textvariable=self.status_var, wraplength=330).grid(row=2, column=0, columnspan=3, sticky='w', pady=(4, 0))

        # display
        f = ttk.LabelFrame(panel, text='Display', padding=6)
        f.pack(fill='x', pady=(8, 0))
        self.mode_var = tk.StringVar(value='position')
        ttk.Radiobutton(f, text='Position (µm, mrad)', variable=self.mode_var, value='position').grid(row=0, column=0, sticky='w')
        ttk.Radiobutton(f, text='Raw capacitance (pF)', variable=self.mode_var, value='raw').grid(row=0, column=1, sticky='w', padx=(8, 0))
        ttk.Label(f, text='Time shown (s)').grid(row=1, column=0, sticky='w', pady=(4, 0))
        self.window_var = tk.DoubleVar(value=self.args.window)
        ttk.Spinbox(f, from_=1, to=MAX_HISTORY_S, increment=1, textvariable=self.window_var, width=6).grid(row=1, column=1, sticky='w', padx=(8, 0), pady=(4, 0))
        ttk.Label(f, text='3D movement drawn ×').grid(row=2, column=0, sticky='w', pady=(4, 0))
        self.exag_var = tk.DoubleVar(value=300)
        ttk.Spinbox(f, from_=1, to=5000, increment=50, textvariable=self.exag_var, width=6).grid(row=2, column=1, sticky='w', padx=(8, 0), pady=(4, 0))
        ttk.Button(f, text='Show plot windows', command=self.show_plot_windows).grid(row=3, column=0, columnspan=2, sticky='w', pady=(6, 0))

        # calibration
        f = ttk.LabelFrame(panel, text='Calibration', padding=6)
        f.pack(fill='x', pady=(8, 0))
        ttk.Label(f, wraplength=330, justify='left', text=(
            'For each axis: tilt the device until the mass rests against the '
            'stop at the low end, hold still, press Set MIN. Tilt it the other '
            'way and press Set MAX. Those two positions are given the values '
            'in the min / max boxes.'
        )).grid(row=0, column=0, columnspan=6, sticky='w')
        for col, text in enumerate(('', '', '', 'min µm', 'max µm', '')):
            ttk.Label(f, text=text).grid(row=1, column=col, sticky='w')
        self.cal_state = {}
        self.travel_vars = {}
        for row, a in enumerate(('x', 'y', 'z'), start=2):
            ttk.Label(f, text=a.upper(), font=('Segoe UI', 10, 'bold')).grid(row=2 * row, column=0, sticky='w', padx=(0, 4))
            ttk.Button(f, text='Set MIN', width=8, command=lambda a=a: self.capture(a, 'min')).grid(row=2 * row, column=1, padx=1, pady=(4, 0))
            ttk.Button(f, text='Set MAX', width=8, command=lambda a=a: self.capture(a, 'max')).grid(row=2 * row, column=2, padx=1, pady=(4, 0))
            vmin = tk.StringVar(value=f"{self.cal.axes[a]['min_um']:g}")
            vmax = tk.StringVar(value=f"{self.cal.axes[a]['max_um']:g}")
            self.travel_vars[a] = (vmin, vmax)
            for col, var in ((3, vmin), (4, vmax)):
                e = ttk.Entry(f, textvariable=var, width=6)
                e.grid(row=2 * row, column=col, padx=1, pady=(4, 0))
                e.bind('<Return>', lambda _e, a=a: self.travel_changed(a))
                e.bind('<FocusOut>', lambda _e, a=a: self.travel_changed(a))
            ttk.Button(f, text='Clear', width=6, command=lambda a=a: self.clear_axis(a)).grid(row=2 * row, column=5, padx=(4, 0), pady=(4, 0))
            self.cal_state[a] = tk.StringVar()
            ttk.Label(f, textvariable=self.cal_state[a], font=mono, foreground='#555').grid(row=2 * row + 1, column=1, columnspan=5, sticky='w')
        ttk.Button(f, text='Set zero here', command=self.set_zero).grid(row=20, column=0, columnspan=3, sticky='w', pady=(8, 0))
        ttk.Label(f, wraplength=330, justify='left', foreground='#555', text=(
            'Zero: makes the present position count as zero for the three '
            'rotations (and for any axis above that is not calibrated yet). '
            'Do this with the device level and the mass at rest.'
        )).grid(row=21, column=0, columnspan=6, sticky='w')

        # channel table
        f = ttk.LabelFrame(below, text='Sensors', padding=6)
        f.pack(side='left', fill='y')
        ttk.Label(f, text=f"{'':5s}{'chip':5s}{'electrodes':16s}{'reading':>8s}{'capacitance':>13s}", font=mono).pack(anchor='w')
        self.ch_vars = []
        for _ in CHANNELS:
            var = tk.StringVar()
            ttk.Label(f, textvariable=var, font=mono).pack(anchor='w')
            self.ch_vars.append(var)

        # firmware messages
        f = ttk.LabelFrame(below, text='Messages from the Pico', padding=6)
        f.pack(side='left', fill='both', expand=True, padx=(8, 0))
        self.msg = tk.Text(f, height=8, width=24, font=mono, state='disabled', wrap='word')
        self.msg.pack(fill='both', expand=True)

    def add_message(self, text):
        self.msg.configure(state='normal')
        self.msg.insert('end', text + '\n')
        if int(self.msg.index('end-1c').split('.')[0]) > 60:
            self.msg.delete('1.0', '10.0')
        self.msg.see('end')
        self.msg.configure(state='disabled')

    # ----- connection -----

    def refresh_ports(self):
        names = []
        preferred = None
        if serial is not None:
            for p in serial.tools.list_ports.comports():
                names.append(p.device)
                if p.vid == 0x2E8A and preferred is None:   # Raspberry Pi
                    preferred = p.device
        names.append(SIMULATED)
        self.port_box['values'] = names
        if not self.port_var.get():
            self.port_var.set(preferred or names[0])
        if serial is None:
            self.status_var.set('pyserial is not installed (pip install pyserial): only simulated data is available')

    def toggle_connection(self):
        if self.source is None:
            self.connect()
        else:
            self.disconnect()

    def connect(self):
        self.disconnect()
        port = self.port_var.get().strip()
        if not port:
            return
        self.history.clear()
        self.t_latest = None
        self.processor = LineProcessor(self.q, self.args.average_ms)
        if port == SIMULATED:
            self.source = SimSource(self.processor)
        elif serial is None:
            self.status_var.set('pyserial is not installed (pip install pyserial)')
            return
        else:
            self.source = SerialSource(port, self.processor)
        self.source.start()
        self.rate_mark = (time.monotonic(), 0)
        self.connect_btn.configure(text='Disconnect')
        self.status_var.set(f'connected to {port}')

    def disconnect(self):
        if self.source is not None:
            self.source.stop()
            self.source = None
            self.connect_btn.configure(text='Connect')
            self.status_var.set('not connected')

    def quit(self):
        self.disconnect()
        self.root.destroy()

    # ----- calibration -----

    def recent_signals(self, duration=CAPTURE_S):
        """Average of the channel combinations over the last `duration`
        seconds, or None if there is no fresh data."""
        if self.t_latest is None or time.monotonic() - self.last_wall > 1.0:
            return None
        sums = [0.0] * 6
        counts = [0] * 6
        for t, sig in reversed(self.history):
            if t < self.t_latest - duration:
                break
            for i, s in enumerate(sig):
                if s is not None:
                    sums[i] += s
                    counts[i] += 1
        return [sums[i] / counts[i] if counts[i] >= 3 else None for i in range(6)]

    def capture(self, axis, which):
        sig = self.recent_signals()
        value = None if sig is None else sig[AXIS_INDEX[axis]]
        if value is None:
            self.add_message(f'cannot set {axis} {which}: no data from the sensors for that axis')
            return
        self.cal.axes[axis][which + '_sig'] = value
        self.cal.save()
        self.update_cal_labels()

    def clear_axis(self, axis):
        self.cal.axes[axis]['min_sig'] = None
        self.cal.axes[axis]['max_sig'] = None
        self.cal.save()
        self.update_cal_labels()

    def travel_changed(self, axis):
        vmin, vmax = self.travel_vars[axis]
        try:
            lo, hi = float(vmin.get()), float(vmax.get())
        except ValueError:
            lo, hi = self.cal.axes[axis]['min_um'], self.cal.axes[axis]['max_um']
        vmin.set(f'{lo:g}')
        vmax.set(f'{hi:g}')
        if (lo, hi) != (self.cal.axes[axis]['min_um'], self.cal.axes[axis]['max_um']):
            self.cal.axes[axis]['min_um'], self.cal.axes[axis]['max_um'] = lo, hi
            self.cal.save()
            self.update_cal_labels()

    def set_zero(self):
        sig = self.recent_signals()
        if sig is None:
            self.add_message('cannot set zero: no data from the sensors')
            return
        self.cal.zero = sig
        self.cal.save()
        self.update_cal_labels()

    def update_cal_labels(self):
        for a in ('x', 'y', 'z'):
            c = self.cal.axes[a]
            lo = '   ---  ' if c['min_sig'] is None else f"{c['min_sig']:8.3f}"
            hi = '   ---  ' if c['max_sig'] is None else f"{c['max_sig']:8.3f}"
            if self.cal.is_calibrated(a):
                state = f'calibrated: {self.cal.gain(a):+.4g} µm/pF'
            elif c['min_sig'] is not None and c['max_sig'] is not None:
                state = 'MIN and MAX are the same: redo'
            else:
                state = 'not calibrated (paper value)'
            self.cal_state[a].set(f'min {lo}  max {hi} pF\n{state}')
        self.solve = self.cal.solver()

    # ----- main loop -----

    def tick(self):
        tick_start = time.perf_counter()
        self.tick_count += 1
        # take in everything the reader thread produced
        while True:
            try:
                kind, payload = self.q.get_nowait()
            except queue.Empty:
                break
            if kind == 'sample':
                t, counts = payload
                caps = [None if v is None else counts_to_pf(v) for v in counts]
                self.history.append((t, axis_signals(caps)))
                self.t_latest = t
                self.last_wall = time.monotonic()
                self.latest_counts = counts
            elif kind == 'info':
                self.add_message(payload)
            elif kind == 'restart':
                self.history.clear()
                self.add_message('(Pico restarted)')
            elif kind == 'error':
                self.add_message('ERROR ' + payload)
                self.disconnect()
                self.status_var.set('connection lost: ' + payload)
        while self.history and self.history[0][0] < self.t_latest - MAX_HISTORY_S:
            self.history.popleft()

        # until "Set zero here" is used, the first reading counts as zero
        if self.cal.zero is None and len(self.history) >= 20:
            sig = self.recent_signals(0.2)
            if sig is not None:
                self.cal.zero = sig
                self.solve = self.cal.solver()

        self.redraw()
        if self.tick_count % 6 == 0:
            self.update_table()
        # aim for one refresh every REFRESH_MS, counting the time this one took
        # (but always leave the window a few ms to react to the mouse)
        spent_ms = (time.perf_counter() - tick_start) * 1000.0
        self.root.after(max(5, int(REFRESH_MS - spent_ms)), self.tick)

    def redraw(self):
        try:
            window_s = min(MAX_HISTORY_S, max(1.0, float(self.window_var.get())))
        except (tk.TclError, ValueError):
            window_s = DEFAULT_WINDOW_S
        try:
            self.view.exaggeration = min(5000.0, max(1.0, float(self.exag_var.get())))
        except (tk.TclError, ValueError):
            pass
        raw = self.mode_var.get() == 'raw'
        solve = self.solve

        # the part of the history that is on screen, thinned out for drawing
        ts, sigs = [], []
        if self.t_latest is not None:
            start = self.t_latest - window_s
            n_visible = sum(1 for t, _ in self.history if t >= start)
            stride = max(1, n_visible // MAX_PLOT_POINTS)
            for i, (t, sig) in enumerate(self.history):
                if t >= start and i % stride == 0:
                    ts.append(t - self.t_latest)
                    sigs.append(sig)
        values = sigs if raw else [solve(s) for s in sigs]

        for charts, win in ((self.charts_trans, self.win_trans), (self.charts_rot, self.win_rot)):
            if not win.winfo_viewable():
                continue
            for a, chart in charts.items():
                i = AXIS_INDEX[a]
                ys = [v[i] for v in values]
                if raw:
                    chart.draw(f'{AXIS_LABEL[a]}:  {SIGNAL_FORMULA[a]}', 'pF', ts, ys, window_s, min_span=0.02)
                else:
                    hlines = []
                    if a in self.cal.axes and self.cal.is_calibrated(a):
                        hlines = [(self.cal.axes[a]['min_um'], 'min'), (self.cal.axes[a]['max_um'], 'max')]
                    chart.draw(AXIS_LABEL[a], AXIS_UNIT[a], ts, ys, window_s,
                               min_span=0.05 if a in self.cal.axes else 0.005, hlines=hlines)

        # 3D view: the average of the last few readings
        recent = self.recent_signals(0.05)
        pose = solve(recent) if recent is not None else (None,) * 6
        travel = (max(abs(self.cal.axes['x']['min_um']), abs(self.cal.axes['x']['max_um'])),
                  max(abs(self.cal.axes['y']['min_um']), abs(self.cal.axes['y']['max_um'])))
        n_cal = sum(self.cal.is_calibrated(a) for a in ('x', 'y', 'z'))
        self.view.draw(pose, travel, f'{n_cal} of 3 axes calibrated')

    def update_table(self):
        for i, (name, chip, electrodes) in enumerate(CHANNELS):
            v = self.latest_counts[i]
            if v is None:
                text = f"{name:5s}{chip:5s}{electrodes:16s}{'----':>8s}{'----':>13s}"
            else:
                text = f'{name:5s}{chip:5s}{electrodes:16s}{"0x%04x" % int(round(v)):>8s}{counts_to_pf(v):10.3f} pF'
            self.ch_vars[i].set(text)
        if self.source is not None and self.processor is not None:
            now = time.monotonic()
            t0, n0 = self.rate_mark
            if now - t0 >= 1.0:
                rate = (self.processor.n_lines - n0) / (now - t0)
                self.rate_mark = (now, self.processor.n_lines)
                self.status_var.set(f'connected to {self.port_var.get()}:  {rate:.0f} readings/s,  '
                                    f'{self.processor.n_bad} bad lines')


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description='LeviTAS v2 position monitor')
    ap.add_argument('--port', help='serial port to connect to at startup (e.g. COM5)')
    ap.add_argument('--simulate', action='store_true', help='use made-up data instead of a serial port')
    ap.add_argument('--cal', default=os.path.join(here, 'levitas_v2_calibration.json'),
                    help='calibration file (default: next to this script)')
    ap.add_argument('--average-ms', type=float, default=AVERAGE_MS,
                    help='average readings over this many milliseconds (default %(default)s)')
    ap.add_argument('--window', type=float, default=DEFAULT_WINDOW_S,
                    help='seconds of data shown in the plots (default %(default)s)')
    args = ap.parse_args()

    root = tk.Tk()
    App(root, args)
    root.mainloop()


if __name__ == '__main__':
    main()
