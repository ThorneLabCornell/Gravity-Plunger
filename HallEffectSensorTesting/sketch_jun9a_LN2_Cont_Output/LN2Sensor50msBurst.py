#DMW - 2025/06/09
import time, datetime, serial, pandas as pd, numpy as np
import matplotlib.pyplot as plt
import pathlib, os          # ← add pathlib / os

# ───── user settings ────────────────────────────────────────────────
PORT        = 'COM8'
BAUD        = 115200
SAMPLE_US   = 500
BURST_MS    = 50
WARMUP_S    = 3.0
VREF        = 5.0
SIGMA_CUT   = 3
OUT_DIR     = pathlib.Path('sketch_jun9a_LN2_Cont_Output')   # sub-folder
# ────────────────────────────────────────────────────────────────────


def capture_burst():
    want = int(BURST_MS * 1000 / SAMPLE_US) + 1
    with serial.Serial(PORT, BAUD, timeout=0.05) as ser:
        ser.setDTR(False); ser.reset_input_buffer()
        time.sleep(WARMUP_S); ser.reset_input_buffer()

        counts = []
        while len(counts) < want:
            ln = ser.readline()
            if ln.endswith(b'\n'):
                try: counts.append(int(ln))
                except ValueError: pass

    volts = np.array(counts) * VREF / 1023.0
    t_ms  = np.arange(len(volts)) * (SAMPLE_US / 1000.0)
    return t_ms, volts


def reject_outliers(times, volts, k=SIGMA_CUT):
    med = np.median(volts)
    mad = np.median(np.abs(volts - med)) or 1e-6
    keep = np.abs(volts - med) < k * mad
    return times[keep], volts[keep]


def save_to_excel_png(times, volts):
    OUT_DIR.mkdir(exist_ok=True)            # create folder if absent
    stamp   = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    stem    = OUT_DIR / f'A0_burst_{BURST_MS}ms_{stamp}'

    # Excel
    pd.DataFrame({'time_ms': times, 'voltage_V': volts}) \
      .to_excel(f'{stem}.xlsx', index=False)

    # Plot & PNG
    plt.figure(figsize=(6,4))
    plt.plot(times, volts, '.-', lw=1.2, ms=4)
    plt.xlabel('Time (ms)'); plt.ylabel('raw0 voltage (V)')
    plt.title(f'{len(volts)} samples in {BURST_MS} ms')
    plt.grid(True); plt.tight_layout()
    plt.savefig(f'{stem}.png', dpi=150)
    plt.show()

    print(f'✅  Saved:\n   ├─ {stem}.xlsx\n   └─ {stem}.png')


if __name__ == '__main__':
    t, v        = capture_burst()
    t, v        = reject_outliers(t, v)
    save_to_excel_png(t, v)
