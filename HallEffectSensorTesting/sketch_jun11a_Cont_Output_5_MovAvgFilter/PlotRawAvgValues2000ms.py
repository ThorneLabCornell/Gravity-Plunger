#!/usr/bin/env python3
"""
Collect one burst of sensor data from an Arduino.

Arduino behaviour expected
--------------------------
* Waits for an 's' byte before transmitting.
* Streams ASCII lines in the form
      <raw1>,<raw2>,<avg1>,<avg2>\r\n
  (volts or ADC counts – it doesn’t matter; all four are numeric.)
* Prints the single line   DONE\r\n   to mark the end of the burst.

The script stops at DONE and immediately shows a plot.
"""

import time
import serial, sys
from pathlib import Path
from datetime import datetime
import matplotlib.pyplot as plt
import pandas as pd  # <---- NEW IMPORT

# ───── user settings ────────────────────────────────────────────────
PORT          = 'COM9'          # ← adjust to your system
BAUD          = 115200         # or 500_000 if you changed Arduino
TIMEOUT_S     = 0.5             # read timeout (s)
START_CHAR    = 's'            # what we send to begin a run
DONE_TOKEN    = 'DONE'          # exact text ending a run
# ────────────────────────────────────────────────────────────────────

# ───── capture parameters ──────────────────────────────────────────
CAPTURE_MS   = 2000           # how long to record (ms)  ➜ ~8 000 samples at 4 kSa/s
FLUSH_MS     = 2500           # give the board time to reboot after the port opens
# ────────────────────────────────────────────────────────────────────

def main() -> None:
    raws1, raws2, avgs1, avgs2 = [], [], [], []

    try:
        with serial.Serial(PORT, BAUD, timeout=TIMEOUT_S) as ser:
            print(f'Opened {PORT} @ {BAUD} Bd')

            # ➊-- wait for the automatic reset to finish and discard start-up noise
            time.sleep(FLUSH_MS / 1000)
            ser.reset_input_buffer()

            print(f'Collecting for {CAPTURE_MS} ms …')
            t0 = time.time()

            while (time.time() - t0) * 1000 < CAPTURE_MS:
                line = ser.readline().decode(errors='ignore').strip()
                if not line:
                    continue                       # read timed out
                try:
                    f1, f2, f3, f4 = map(float, line.split(','))
                except ValueError:
                    continue                       # malformed line → skip it
                raws1.append(f1); raws2.append(f2)
                avgs1.append(f3); avgs2.append(f4)

    except serial.SerialException as e:
        sys.exit(f'⚠️ Serial error: {e}')

    if not raws1:
        sys.exit('⚠️ No data captured.')
    
    # drop the very first sample – the sketch sometimes spits out junk here
    for L in (raws1, raws2, avgs1, avgs2):
        del L[0]                     # remove index 0 in-place

    print(f'Captured {len(raws1)} samples.  Saving + plotting …')
    save_to_excel(raws1, raws2, avgs1, avgs2)
    plot_data   (raws1, raws2, avgs1, avgs2)


def save_to_excel(r1, r2, a1, a2):
    """
    Save the sensor data to an Excel file in the same directory as the plot.
    """
    fname = Path(__file__).with_suffix('')
    ts    = datetime.now().strftime('%Y%m%d_%H%M%S')
    xlsx  = Path(f'{fname}_{ts}.xlsx')
    df = pd.DataFrame({
        'raw1': r1,
        'raw2': r2,
        'avg1': a1,
        'avg2': a2
    })
    #df.to_excel(xlsx, index_label='Sample #')
    print(f'Data saved to {xlsx}')

def plot_data(r1, r2, a1, a2):
    x = range(len(r1))  # sample index
    plt.figure(figsize=(10, 5))
    plt.plot(x, r1, label='raw1')
    plt.plot(x, r2, label='raw2')
    plt.plot(x, a1, label='avg1')
    plt.plot(x, a2, label='avg2')
    plt.title('Hall-sensor burst')
    plt.xlabel('Sample #')
    plt.ylabel('Value')
    plt.legend()
    plt.tight_layout()

    # optional – save a timestamped PNG next to the .py file
    fname = Path(__file__).with_suffix('')
    ts    = datetime.now().strftime('%Y%m%d_%H%M%S')
    png   = Path(f'{fname}_{ts}.png')
    #plt.savefig(png, dpi=150)
    print(f'Plot saved to {png}')

    plt.show()

if __name__ == '__main__':
    main()
