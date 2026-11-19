import csv
import queue
import threading
import time
from collections import deque
from statistics import median

import serial;

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button

# ---------------- settings ----------------
PORT, BAUD = 'COM3', 115200
BASE  = [1416, 1762, 1757, 1621]  # raw count at 0 µm, per sensor
SLOPE = [62.5, 67.5, 76, 87.5]    # raw counts per µm, per sensor

SPIKE_COUNTS = 40     # reject a reading this many raw counts from its recent median
MEDIAN_LEN = 5        # how many recent readings the median uses
SCROLL_WINDOW = 20000 # samples shown in moving-window view (~3 s, covers a full HV cycle)
CAPTURE_SIZE = 20000  # samples in the frozen "fit" capture
LOG_CSV = True        # save data to capture_<time>.csv
LOG_AVG = 50          # average this many samples into one CSV row (1 = log every sample)
LOG_SECONDS = 0       # stop logging after this many seconds (0 = never stop)

ser = serial.Serial(PORT, BAUD)
q = queue.Queue()
bad_lines = 0         # running count of malformed / out-of-range / out-of-order lines


# ---------------- parsing ----------------
def parse(line):
    """Return (t_us, [4 raw], [4 hv]) or None if the line is malformed."""
    parts = line.split()
    if len(parts) < 9:                  # timestamp + 4 hex + 4 floats (+ height %, target %, mode: not used here)
        return None
    try:
        t = int(parts[0])
        raw = [int(p, 16) for p in parts[1:5]]
        hv = [float(p) for p in parts[5:9]]
    except ValueError:
        return None
    if any(r > 0x0FFF for r in raw):    # FDC2112 data is 12 bits
        return None
    return t, raw, hv


# ---------------- reader thread ----------------
def reader():
    global bad_lines
    buf, last_t, n = b'', -1, 0
    history = [deque(maxlen=MEDIAN_LEN) for _ in range(4)]

    f = open('square.csv', 'w', newline='') if LOG_CSV else None
    log = csv.writer(f) if f else None
    if log:
        log.writerow(['t_us', 'c1', 'c2', 'c3', 'c4', 'spikes', 'height_um', 'hv_applied'])
    block = []        # samples waiting to be averaged into one CSV row
    t_start = None

    while True:
        buf += ser.read(ser.in_waiting or 1)
        *lines, buf = buf.split(b'\n')   # keep any partial last line in buf
        for line in lines:
            p = parse(line.decode(errors='ignore'))
            if p is None or p[0] <= last_t:
                bad_lines += 1           # torn, corrupted, out-of-range or out-of-order line
                continue
            t, raw, hv = p
            last_t = t

            # per-sensor spike rejection: replace outliers with the recent median
            clean, spike = [], False
            for hist, r in zip(history, raw):
                if len(hist) == MEDIAN_LEN and abs(r - median(hist)) > SPIKE_COUNTS:
                    clean.append(median(hist))
                    spike = True
                else:
                    clean.append(r)
                hist.append(r)           # raw value, so real steps get accepted

            height = sum((b - c) / s for b, c, s in zip(BASE, clean, SLOPE)) / 4
            hv_applied = max(hv)
            q.put((t / 1e6, height, hv_applied, spike))

            if log:
                if t_start is None:
                    t_start = t
                block.append((t, raw, spike, height, hv_applied))
                if len(block) == LOG_AVG:
                    k = len(block)
                    log.writerow([
                        block[k // 2][0],                                         # middle timestamp
                        *[f'{sum(b[1][i] for b in block) / k:.1f}' for i in range(4)],  # avg raw counts
                        sum(b[2] for b in block),                                 # spikes in block
                        f'{sum(b[3] for b in block) / k:.4f}',                    # avg height
                        f'{sum(b[4] for b in block) / k:.2f}',                    # avg voltage
                    ])
                    block.clear()
                    n += 1
                    if n % 100 == 0:
                        f.flush()        # so little is lost when the window closes
                if LOG_SECONDS and t - t_start >= LOG_SECONDS * 1_000_000:
                    f.close()
                    log = None
                    print('CSV logging finished.')


threading.Thread(target=reader, daemon=True).start()


# ---------------- plot ----------------
fig, ax = plt.subplots()
fig.subplots_adjust(bottom=0.22, right=0.86)
ax_v = ax.twinx()

h_line, = ax.plot([], [], lw=0.8, label='Height')
spike_pts, = ax.plot([], [], 'rx', ms=4, label='Rejected spike')
v_line, = ax_v.plot([], [], color='tab:orange', lw=0.8, label='HV')

ax.set_xlabel('Time (s)')
ax.set_ylabel('Height (µm)')
ax_v.set_ylabel('Voltage (V)')
ax.set_ylim(-0.25, 3)
ax_v.set_ylim(0, 260)
ax.legend(handles=[h_line, spike_pts, v_line], loc='upper left', fontsize=8)

live = deque(maxlen=SCROLL_WINDOW)   # moving-window buffer
capture = []                         # frozen fit-view capture
moving = False

button = Button(fig.add_axes([0.4, 0.03, 0.2, 0.06]), 'Moving window: OFF')
button.label.set_fontsize(8)

def toggle(_):
    global moving
    moving = not moving
    button.label.set_text(f"Moving window: {'ON' if moving else 'OFF'}")
    if not moving:
        capture.clear()              # start a fresh capture
        print(f'Starting new {CAPTURE_SIZE}-sample capture...')

button.on_clicked(toggle)

stats = {'n': 0, 'spikes': 0, 'bad_prev': 0, 't0': time.perf_counter()}

def update(_):
    # pull everything the reader produced since the last frame
    while True:
        try:
            s = q.get_nowait()
        except queue.Empty:
            break
        live.append(s)
        if not moving and len(capture) < CAPTURE_SIZE:
            capture.append(s)
            if len(capture) == CAPTURE_SIZE:
                print('Capture complete (frozen). Toggle twice for a new one.')
        stats['n'] += 1
        stats['spikes'] += s[3]

    # once-a-second diagnostics
    now = time.perf_counter()
    if now - stats['t0'] >= 1:
        bad_now = bad_lines
        print(f"{stats['n'] / (now - stats['t0']):.0f} samples/s, "
              f"{stats['spikes']} spikes rejected, "
              f"{bad_now - stats['bad_prev']} bad lines dropped")
        stats.update(n=0, spikes=0, bad_prev=bad_now, t0=now)

    shown = live if moving else capture
    if len(shown) < 2:
        return h_line, v_line, spike_pts

    t, h, v, sp = zip(*shown)
    h_line.set_data(t, h)
    v_line.set_data(t, v)
    spike_pts.set_data([ti for ti, s in zip(t, sp) if s],
                       [hi for hi, s in zip(h, sp) if s])
    if t[-1] > t[0]:
        ax.set_xlim(t[0], t[-1])
    return h_line, v_line, spike_pts

ani = FuncAnimation(fig, update, interval=50, cache_frame_data=False)
plt.show()