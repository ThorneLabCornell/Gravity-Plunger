import serial
import csv
import numpy as np
import time
import pandas as pd
import matplotlib.pyplot as plt

# Configuration
port = 'COM7'
baudrate = 115200
csv_filename = 'test.csv'
LUT_filename = r"C:\Users\natha\OneDrive\Documents\Thorne Lab\plungecooler\Thermocouple_LUT.txt"

# --- Serial Connection & Data Collection ---
def collect_data(port, baudrate, LUT):
    ser = serial.Serial(port, baudrate, timeout=10)
    thermoLog = []
    timeLog = []
    startTime = time.time_ns()

    while True:
        try:
            line = ser.readline().decode().rstrip()
            if line == 'complete':
                break

            voltage_raw = float(line)
            voltage = (voltage_raw * (3.3 / (pow(2, 12) - 1))) - 1.25
            temperatureC = np.interp([voltage], LUT[:, 2:3].flatten(), LUT[:, 0:1].flatten())
            thermoLog.append(temperatureC)
            timeLog.append(time.time_ns() - startTime) 
        except (ValueError, UnicodeDecodeError):
            print("Error processing serial data. Skipping line.")

    ser.close()
    return timeLog, thermoLog

# --- Data Processing & Output ---
def process_and_save(timeLog, thermoLog, filename):
    df = pd.DataFrame({'timeLog': timeLog, 'thermoLog': thermoLog})
    df['timeLog'] = df['timeLog'] - df['timeLog'].iloc[0]  # Start time at 0
    df.to_csv(filename, index=False)
    return df

def plot_results(df):
    plt.figure(figsize=(10, 6))
    plt.plot(df['timeLog'] / 1e9, df['thermoLog'])  # Convert time to seconds
    plt.xlabel('Time (seconds)')
    plt.ylabel('Temperature (Celsius)')
    plt.title('Temperature Over Time')
    plt.grid(True)
    plt.show()

# --- Main Execution ---
if __name__ == "__main__":
    LUT = np.genfromtxt(LUT_filename, delimiter=',')
    timeLog, thermoLog = collect_data(port, baudrate, LUT)
    df = process_and_save(timeLog, thermoLog, csv_filename)
    plot_results(df)
    print(f"Data logged to {csv_filename}")