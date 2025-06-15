#DMW - 2025/06/02
#!/usr/bin/env python3
"""
Hall-sensor live viewer – DEBUG build
-------------------------------------
* Prints a short message every time it keeps or drops a line.
* Flushes the USB backlog after the first good sample so
  the plot appears within a second.
* Limits parsing to 2 000 lines per animation frame so the UI
  never stalls even if the PC falls behind.

Comment out the lines marked  ### DEBUG  once things are working.
"""

import collections, serial, matplotlib, time
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ───── user settings ────────────────────────────────────────────────
PORT, BAUD_RATE = 'COM8', 115200
SAMPLE_US       = 500                 # 0.5 ms  → 2 kSa s⁻¹
PERIOD_S        = SAMPLE_US * 1e-6
ROLL_WINDOW     = 200
VIEW_SEC        = 2.0
MAX_SAMPLES     = 10000
REFRESH_MS      = 80
MAX_LINES_FRAME = 2000
# ────────────────────────────────────────────────────────────────────

def rolling_mean(buf, n):
    if len(buf) < n: return []
    csum = [0.0]
    for x in buf: csum.append(csum[-1] + x)
    return [(csum[i+n] - csum[i]) / n for i in range(len(buf)-n+1)]

def open_port():
    try:
        s = serial.Serial(PORT, BAUD_RATE, timeout=1)
        s.reset_input_buffer()
        print(f"✓ Opened {PORT} @ {BAUD_RATE} baud")
        return s
    except serial.SerialException as e:
        print(f"⚠  {e}"); return None

ser = open_port()
base_cnt = None  # first counter kept  → t = 0

t_buf, raw0_buf, raw1_buf = (collections.deque(maxlen=MAX_SAMPLES) for _ in range(3))
min0 = float('inf'); max0 = float('-inf')
min1 = float('inf'); max1 = float('-inf')

plt.style.use('ggplot')
fig, ax = plt.subplots()
ln_r0, = ax.plot([], [], lw=1, label='A0 raw')
ln_a0, = ax.plot([], [], lw=2, label=f'A0 {ROLL_WINDOW}-avg')
ln_r1, = ax.plot([], [], lw=1, label='A1 raw')
ln_a1, = ax.plot([], [], lw=2, label=f'A1 {ROLL_WINDOW}-avg')
ax.set_xlabel('Time [s]'); ax.set_ylabel('Voltage [V]')
ax.set_title('Hall-sensor live stream'); ax.legend(loc='upper left')
ax.set_xlim(0, VIEW_SEC)

def update(_):
    global base_cnt, min0, max0, min1, max1
    if not ser or not ser.is_open: return

    lines   = 0
    dropped = 0
    try:
        while ser.in_waiting and lines < MAX_LINES_FRAME:
            raw = ser.readline().decode(errors='ignore').strip()
            parts = raw.split(',')
            if len(parts) != 5:
                dropped += 1; continue
            try:
                cnt = int(parts[0]); r0 = float(parts[1]); r1 = float(parts[2])
            except ValueError:
                dropped += 1; continue

            if base_cnt is None:
                base_cnt = cnt
                ser.reset_input_buffer()           # flush backlog
                #print(f"First good line: {raw}")    ### DEBUG
            t = (cnt - base_cnt) * PERIOD_S
            t_buf.append(t); raw0_buf.append(r0); raw1_buf.append(r1)
            min0, max0 = min(min0, r0), max(max0, r0)
            min1, max1 = min(min1, r1), max(max1, r1)
            lines += 1
    except serial.SerialException:
        print("⚠  Serial lost"); return

    #if dropped:   print(f"dropped {dropped} malformed lines")           ### DEBUG
    #if not t_buf: print("no valid samples yet")                         ### DEBUG

    ln_r0.set_data(t_buf, raw0_buf); ln_r1.set_data(t_buf, raw1_buf)
    avg0 = rolling_mean(raw0_buf, ROLL_WINDOW)
    avg1 = rolling_mean(raw1_buf, ROLL_WINDOW)
    if avg0:
        ln_a0.set_data(list(t_buf)[ROLL_WINDOW-1:], avg0)
        ln_a1.set_data(list(t_buf)[ROLL_WINDOW-1:], avg1)

    ax.relim(); ax.autoscale_view(scalex=False)
    if t_buf:
        t_end = t_buf[-1]
        ax.set_xlim(t_end - VIEW_SEC, t_end) if t_end >= VIEW_SEC else ax.set_xlim(0, VIEW_SEC)

    return ln_r0, ln_r1, ln_a0, ln_a1

ani = FuncAnimation(fig, update, interval=REFRESH_MS,
                    blit=False, cache_frame_data=False)
plt.show()
if ser and ser.is_open: ser.close()
