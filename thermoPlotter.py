import pandas as pd
import matplotlib.pyplot as plt
import re

# Read the CSV file
df1 = pd.read_csv(r'C:\Users\natha\OneDrive\Documents\Thorne Lab\Gravity-Plunger\control_4.csv')
df2 = pd.read_csv(r'C:\Users\natha\OneDrive\Documents\Thorne Lab\Gravity-Plunger\3750_4.csv')
df3 = pd.read_csv(r'C:\Users\natha\OneDrive\Documents\Thorne Lab\Gravity-Plunger\5220_4.csv')
# Function to extract the first numeric value from a string
def extract_first_numeric(value):
    match = re.search(r"[-+]?\d*\.?\d+", str(value))
    if match:
        return float(match.group())
    else:
        return None  # Or handle the case where no number is found

# Apply the extraction function to the 'thermoLog' column
df1['thermoLog'] = df1['thermoLog'].apply(extract_first_numeric)
df2['thermoLog'] = df2['thermoLog'].apply(extract_first_numeric)
df3['thermoLog'] = df3['thermoLog'].apply(extract_first_numeric)

# Plotting
plt.figure(figsize=(10, 6))
plt.plot(df1['timeLog'] / 1e6, df1['thermoLog'], label='Control')
plt.plot(df2['timeLog'] / 1e6, df2['thermoLog'], label='37.5mm Rod')
plt.plot(df3['timeLog'] / 1e6, df3['thermoLog'], label='55.2mm Rod')
plt.title('Time vs Thermo')
plt.xlabel('Time')
plt.ylabel('Thermo')
plt.grid(axis='y', alpha=0.75)
plt.legend()
plt.tight_layout()
plt.show()