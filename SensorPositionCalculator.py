import pandas as pd
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# MAX SENSOR HEIGHT: 574mm

# Define your desired time point
timepoint = 250

# Row Index at which the free fall starts
freeFallStart = 9

# Time vs Position data collected for mapping
file_path = 'GravityPlunger Position vs Time Curve.xlsx'


# Read the Excel file into a DataFrame
posTimeData = pd.read_excel(file_path)
servoMotion = posTimeData.iloc[:freeFallStart]  # Pos Time data for servo motion
freeFall = posTimeData.iloc[freeFallStart:]     # Pos Time data for free fall

# Define the quadratic function
def quadratic_func(x, a, b, c):
    return a * x**2 + b * x + c
print(posTimeData.iat[freeFallStart, posTimeData.columns.get_loc('Time')])

if timepoint <= posTimeData.iat[freeFallStart, posTimeData.columns.get_loc('Time')]:

    # Fit the quadratic function to the data
    popt, _ = curve_fit(quadratic_func, freeFall['Time'], freeFall['Sensor Pos'])

    # Extract the parameters
    a, b, c = popt

    # Generate y values for the fitted curve
    y_fit = quadratic_func(freeFall['Time'], a, b, c)

    # Calculate the predicted position at the time point
    sensor_position = quadratic_func(timepoint, a, b, c)
    # Create the scatter plot
    plt.figure(figsize=(10, 6))
    # Plot the fitted curve
    plt.plot(freeFall['Time'], y_fit, 'r-', label='Fitted Curve')

if timepoint > posTimeData.iat[freeFallStart, posTimeData.columns.get_loc('Time')]:
    # Fit the quadratic function to the data
    popt, _ = curve_fit(quadratic_func, servoMotion['Time'], servoMotion['Sensor Pos'])

    # Extract the parameters
    a, b, c = popt

    # Generate y values for the fitted curve
    y_fit = quadratic_func(servoMotion['Time'], a, b, c)

    # Calculate the predicted position at the time point
    sensor_position = quadratic_func(timepoint, a, b, c)
    # Create the scatter plot
    plt.figure(figsize=(10, 6))
    # Plot the fitted curve
    plt.plot(servoMotion['Time'], y_fit, 'r-', label='Fitted Curve')

platform_position = sensor_position - 201


print(f'Desired Timepoint: {timepoint}ms')
print(f'Estimated Position of Sensor: {sensor_position:.1f}mm')
print(f'Platform Position: {platform_position:.1f}mm')



# print(f'The equation of the fitted curve is: y = {a:.3f}x^2 + {b:.3f}x + {c:.3f}')

# Plot the full motion data
plt.scatter(posTimeData['Time'], posTimeData['Sensor Pos'], label='Data')

# Add labels, title, and legend
plt.xlabel('Time')
plt.ylabel('Sensor Pos')
plt.title('Timepoint vs Sensor Position')
plt.legend()

# Display the plot  
plt.show()