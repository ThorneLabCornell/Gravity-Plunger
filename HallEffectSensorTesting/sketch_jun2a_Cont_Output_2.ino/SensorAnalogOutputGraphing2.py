#DMW - 2025/06/02
#!/usr/bin/env python3
"""
Live plot of two Hall-sensor channels (raw + 16-sample avg).

Arduino sends:  count,raw0,raw1
    ⇢ count : uint32, increments every sample
    ⇢ raw0/1: 0–1023 (10-bit ADC counts)

Python:
    • Converts counts ➜ seconds (period = 0.0005 s)
    • Converts raw counts ➜ volts (VREF = 5.0 V)
    • Computes 16-sample rolling mean
    • Displays a scrolling 2-s window
"""

import collections, serial, matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ───── user settings ────────────────────────────────────────────────
PORT, BAUD = 'COM8', 115200          # match Serial.begin()
SAMPLE_US  = 500                     # must match Arduino
PERIOD_S   = SAMPLE_US * 1e-6
VREF       = 5.0
AVG_WIN    = 16
VIEW_SEC   = 0.2
MAX_SAMP   = 10000
REFRESH_MS = 80
MAX_LINES  = 2000

# ── globals ──────────────────────────────
base_cnt = None       # first counter value → t = 0
synced   = False      # have we already flushed once?
# ────────────────────────────────────────────────────────────────────


def rolling_mean(buf, n):
    if len(buf) < n: return []
    csum = [0]
    for x in buf: csum.append(csum[-1] + x)
    return [(csum[i+n] - csum[i]) / n for i in range(len(buf) - n + 1)]


# ───── serial port ──────────────────────────────────────────────────
ser = serial.Serial(PORT, BAUD, timeout=1)
ser.reset_input_buffer()
print(f"✓ Serial {PORT} opened @ {BAUD}")

base_cnt = None                      # first counter kept → t = 0

t_buf    = collections.deque(maxlen=MAX_SAMP)
raw0_buf = collections.deque(maxlen=MAX_SAMP)
raw1_buf = collections.deque(maxlen=MAX_SAMP)

# ───── figure ───────────────────────────────────────────────────────
plt.style.use('ggplot')
fig, ax = plt.subplots()
ln_r0, = ax.plot([], [], lw=1, label='A0 raw')
ln_a0, = ax.plot([], [], lw=2, label=f'A0 {AVG_WIN}-avg')
ln_r1, = ax.plot([], [], lw=1, label='A1 raw')
ln_a1, = ax.plot([], [], lw=2, label=f'A1 {AVG_WIN}-avg')
ax.set_xlabel('Time [s]'); ax.set_ylabel('Voltage [V]')
ax.set_title('Hall-sensor live stream')
ax.legend(loc='upper left')
ax.set_xlim(0, VIEW_SEC)


def update(_):
    global base_cnt, synced
    parsed = 0

    while ser.in_waiting and parsed < MAX_LINES:
        parts = ser.readline().decode(errors='ignore').strip().split(',')
        if len(parts) != 3:
            continue
        try:
            cnt = int(parts[0]); r0 = int(parts[1]); r1 = int(parts[2])
        except ValueError:
            continue

        # -------- one-time flush right after we connect ----------
        if not synced:
            ser.reset_input_buffer()   # toss backlog *once*
            synced = True              # don't do it again
            continue                   # skip this line, wait for fresh one

        # -------- establish t = 0 on first fresh sample ----------
        if base_cnt is None:
            base_cnt = cnt             # this packet defines zero
            t = 0.0
        else:
            t = (cnt - base_cnt) * PERIOD_S

        # -------- store data ----------
        v0 = r0 * VREF / 1023.0
        v1 = r1 * VREF / 1023.0
        t_buf.append(t); raw0_buf.append(v0); raw1_buf.append(v1)
        parsed += 1

    # -------- drawing ----------
    ln_r0.set_data(t_buf, raw0_buf)
    ln_r1.set_data(t_buf, raw1_buf)
    avg0 = rolling_mean(raw0_buf, AVG_WIN)
    avg1 = rolling_mean(raw1_buf, AVG_WIN)
    if avg0:
        ln_a0.set_data(list(t_buf)[AVG_WIN-1:], avg0)
        ln_a1.set_data(list(t_buf)[AVG_WIN-1:], avg1)

    if t_buf:
        t_end   = t_buf[-1]
        t_start = max(0.0, t_end - VIEW_SEC)   # never invert limits
        ax.set_xlim(t_start, t_end)
    ax.relim(); ax.autoscale_view(scalex=False)
    return ln_r0, ln_r1, ln_a0, ln_a1

ani = FuncAnimation(fig, update,
                    interval=REFRESH_MS,
                    blit=False,
                    cache_frame_data=False)
plt.show()
ser.close()
