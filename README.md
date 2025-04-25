# Hand-Gesture-Controlled-Drone
A hand gesture controlled drone built using Pixhawk flight controller and Arduino UNO along with MPU6050 Gyroscope and potentiometer, linked using telemetry.

# Operating Mechanism
- The Arduino Uno receives pitch and roll data from the MPU6050 gyroscope, and throttle input from a potentiometer. These raw sensor values are mapped to drone-compatible control signals in the range of 1000 to 2000, suitable for RC channel inputs.
- A Python script, using the MAVLink protocol, transmits the mapped control values from the Arduino to the Pixhawk flight controller via a telemetry link (connected to telem1 port on pixhawk), enabling real-time gesture-based control of the drone. Parameters were also modified accordingly to override RC control in Qgroundcontrol software.
- The connections for arduino are shown in the circuit_diagram.png file.
- All parameters to be changed are also given in the gesture.params file. (Refer NOTE)

# MAVLink
- An SiK Telemtry module is used to transmit data from arduino to pixhawk.
- Connect to telem1 port of pixhawk.
- Go to QGC > Analyze tools > MAVLink inspector > RC_CHANNELS. Once the python code runs, the change in channel 1 (Roll), channel 2 (Pitch), channel 3 (Throttle) values as per the gyroscope and potentiometer readings from arduino will be displayed, which in turn controls the motors.

# NOTE
1. Make sure to connect arduino circuit and telemetry transmitter in one PC, and the drone with pixhawk in another for calibration and parameter changes.
2. COM_POWER_COUNT must be set to 0 for connection with the pixhawk even after disconnecting from PC.
3. This project is only for throttle, pitch and roll. Yaw has not been implemented due to instability in gesture control.
