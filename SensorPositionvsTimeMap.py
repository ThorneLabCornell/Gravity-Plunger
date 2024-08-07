import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit


# Define your desired time point
timepoint = 50

# Time vs Position data collected for mapping
file_path = 'GravityPlunger Position vs Time Curve.xlsx'


# Read the Excel file into a DataFrame
df = pd.read_excel(file_path)

# Define the quadratic function
def quadratic_func(x, a, b, c):
    return a * x**2 + b * x + c

# Fit the quadratic function to the data
popt, _ = curve_fit(quadratic_func, df['Time'], df['Sensor Pos'])

# Extract the parameters
a, b, c = popt

# Generate y values for the fitted curve
y_fit = quadratic_func(df['Time'], a, b, c)

# Calculate the predicted position at the time point
sensor_position = quadratic_func(timepoint, a, b, c)
platform_position = sensor_position - 201


print(f'Desired Timepoint: {timepoint}ms')
print(f'Estimated Position of Sensor: {sensor_position:.0f}mm')
print(f'Platform Position: {platform_position:.0f}mm')


# # Create the scatter plot
# plt.figure(figsize=(10, 6))
# plt.scatter(df['Time'], df['Sensor Pos'], label='Data')

# # Plot the fitted curve
# plt.plot(df['Time'], y_fit, 'r-', label='Fitted Curve')

# # Add labels, title, and legend
# plt.xlabel('Time')
# plt.ylabel('Sensor Pos')
# plt.title('Quadratic Fit to Sensor Data')
# plt.legend()

# Display the plot
plt.show()

