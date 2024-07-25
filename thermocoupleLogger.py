import serial

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

# Open CSV file for writing
with open(csv_filename, 'w', newline='') as csvfile:

    while True:
        try:
            line = ser.readline().decode().rstrip()
            print(line)
        except UnicodeDecodeError:
            print("Error decoding serial data. Skipping line.")
            continue  # Skip to the next line on decoding errors

        if line == 'complete':
            break  # Exit loop when "complete" is received

        # Write data to CSV
        csvfile.write(line + '\n')

# Close serial connection
ser.close()

print(f"Data logged to {csv_filename}")