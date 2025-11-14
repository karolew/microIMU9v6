# MinIMU-9 v6 Tilt Compensated Compass

This is a micropython custom driver implementation for MinIMU-9 v6 compass with tilt compensation.

# Usage

1. Copy `imu9v6.py` to your project
2. Initialize I2C with specified pins. Example ESP32:</br>
`i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)`
3. Create compas instance, set calibrate to True (only for very first run).
Calibration time is hardcoded to 60s. Example:</br>
`compass = MinIMU9v6(i2c, calibrate=True)`
4. Get heading in degres:</br>
`heading = compass.get_tilt_compensated_heading()`