# Gravity-Plunger

## Operation
- Operate the gravity plunger by flashing Arduino Uno R3 with 'Gravity_Plunger_Program.ino'
- For timepoint < 300ms
```sh
- set waitTime to 0
- move photo reflective sensor and platform to the desired position via iteration OR using 'SensorPositionCalculator.py'
- Ativate plunge and see the displayed reaction time, adjust position if necessary
```
- For timepoint > 300ms
```sh
- set waitTime to desired time in milliseconds
- Activate plunge and see the displayed reaction time, adjust waitTime if necessary
```

# Position Calibration (optional)
- Place photo reflective sensor below servo
- Activate plunge
- Record time point in ms and position of sensor mount from measuring stick in mm into 'GravityPlunger Position vs Time Curve.xlsx'
- Move photo reflective sensor down stem rod and repeat until satisfied
- Run 'SensorPositionCalculator.py' and enter row at which rod begins free fall after servo release into freeFallStart
- 'SensorPositionCalculator.py' will now use a second order curve fit to approximate position at which to place sensor mount and platform given a timepoint.
- NOTE: This script will only be effective with 0 wait time. For wait time > 0, iteratively run plunges until time point is reached.

# Temperature Characterization
- Temperature characterization was completed with butt-welded type K 150mm thermocouple.
- Experiment was done as follows
```sh
- Nucleo F411RE flashed with STM32_THERMOCOUPLE code
- Arduino flashed with Gravity_Plunger_Program_Thermocouple
- For each run, specify a csv file name and run 'thermocoupleLogger.py'
```
