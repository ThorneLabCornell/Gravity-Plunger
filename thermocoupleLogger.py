import serial
import csv
import datetime
import numpy as np

# Configuration
port = 'COM7'
baudrate = 115200
csv_filename = 'logged_data.csv'

# Establish serial connection
try:
    ser = serial.Serial(port, baudrate, timeout=10)  # Timeout for non-blocking reads
except serial.SerialException as e:
    print(f"Error connecting to serial port: {e}")
    exit(1)

thermoLog = []
timeLog = []
LUT = np.genfromtxt("C:\Users\natha\OneDrive\Documents\Thorne Lab\plungecooler\Thermocouple_LUT.txt", delimiter=',')
# Open CSV file for writing
with open(csv_filename, 'w', newline='') as csvfile:
    csv_writer = csv.writer(csvfile)
    csv_writer.writerow(['Timestamp', 'Temperature'])  # Header row

    while True:
        try:
            line = ser.readline().decode().rstrip()
            if line == 'complete':
                break
            # Log to lists
            voltage = (line * (3.3 / (pow(2, 12)-1))) - 1.25
            input = LUT[:, 2:3].flatten()
            temperatures = LUT[:, 0:1].flatten()
            temperatureC = np.interp([voltage], input, temperatures)
            thermoLog.append(temperatureC)
            timeLog.append(datetime.datetime.now())
        except UnicodeDecodeError:
            print("Error decoding serial data. Skipping line.")
            continue

    # Write logs to CSV
    for timestamp, temperature in zip(timeLog, thermoLog):
        csv_writer.writerow([timestamp.strftime("%H:%M:%S"), temperature])

# Close serial connection
ser.close()

print(f"Data logged to {csv_filename}")