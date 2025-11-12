from machine import I2C, Pin
import time
import math
import json


class MinIMU9v6:

    CALIBRATION_FILE = "calibration.json"

    def __init__(self, i2c, lsm_addr=0x6B, lis_addr=0x1E, calibrate: bool = False) -> None:
        self.i2c = i2c
        self.lsm_addr = lsm_addr        # LSM6DSO (gyro/accel)
        self.lis_addr = lis_addr        # LIS3MDL (magnetometer)
        self.calibrate = calibrate      # Calibrate before use.

        # Calibration offsets.
        self.mag_offset = [0, 0, 0]
        self.mag_scale = [1, 1, 1]

        # Initialize sensors.
        self._init_lsm6dso()
        self._init_lis3mdl()

        # Calibrate and save calibration data.
        if self.calibrate:
            self.calibrate_magnetometer()

        # Load calibration data.
        self.load_calibration()

    def _init_lsm6dso(self) -> None:
        """
        Initialize LSM6DSO accelerometer and gyroscope
        See https://www.pololu.com/file/0J1899/lsm6dso.pdf
        """

        # Reset LSM6DSO.
        self.i2c.writeto_mem(self.lsm_addr, 0x12, b'\x01')
        time.sleep_ms(100)

        # Configure accelerometer: 104 Hz, +-2g, low-power mode.
        self.i2c.writeto_mem(self.lsm_addr, 0x10, b'\x40')  # CTRL1_XL register addr 10h.

        # Configure gyroscope: 104 Hz, 250 dps.
        self.i2c.writeto_mem(self.lsm_addr, 0x11, b'\x40')  # CTRL2_G register addr 11h.

        # Enable block data update.
        self.i2c.writeto_mem(self.lsm_addr, 0x12, b'\x44')  # CTRL3_C register addr 12h.

    def _init_lis3mdl(self) -> None:
        """
        Initialize LIS3MDL magnetometer.
        See https://www.pololu.com/file/0J1089/LIS3MDL.pdf
        """

        # Configure magnetometer.
        # CTRL_REG1: Temp enabled, high-performance XY, 80 Hz ODR, no self-test
        self.i2c.writeto_mem(self.lis_addr, 0x20, b'\x7C')

        # CTRL_REG2: +-4 gauss full scale.
        self.i2c.writeto_mem(self.lis_addr, 0x21, b'\x00')

        # CTRL_REG3: Continuous conversion mode.
        self.i2c.writeto_mem(self.lis_addr, 0x22, b'\x00')

        # CTRL_REG4: High-performance Z axis.
        self.i2c.writeto_mem(self.lis_addr, 0x23, b'\x0C')

        # CTRL_REG5: Block data update.
        self.i2c.writeto_mem(self.lis_addr, 0x24, b'\x40')

    def read_accel(self) -> tuple | None:
        """
        Read accelerometer data in g
        """
        try:
            # Read 6 bytes from OUTX_L_A (0x28).
            data = self.i2c.readfrom_mem(self.lsm_addr, 0x28, 6)

            # LSM6DSO: +-2g = 16384 LSB/g (sensitivity).
            ax = self._to_int16(data[1], data[0]) / 16384.0
            ay = self._to_int16(data[3], data[2]) / 16384.0
            az = self._to_int16(data[5], data[4]) / 16384.0

            return ax, ay, az
        except:
            return None

    def read_mag(self) -> tuple | None:
        """
        Read magnetometer data in gauss
        """
        try:
            # Check if data is ready (STATUS_REG 0x27, bit 3).
            status = self.i2c.readfrom_mem(self.lis_addr, 0x27, 1)[0]
            if not (status & 0x08):
                return None

            # Read 6 bytes from OUT_X_L (0x28).
            data = self.i2c.readfrom_mem(self.lis_addr, 0x28, 6)

            # LIS3MDL: ±4 gauss = 6842 LSB/gauss (sensitivity).
            mx = self._to_int16(data[1], data[0]) / 6842.0
            my = self._to_int16(data[3], data[2]) / 6842.0
            mz = self._to_int16(data[5], data[4]) / 6842.0

            # Apply calibration
            mx = (mx - self.mag_offset[0]) * self.mag_scale[0]
            my = (my - self.mag_offset[1]) * self.mag_scale[1]
            mz = (mz - self.mag_offset[2]) * self.mag_scale[2]

            return mx, my, mz

        except:
            return None

    def _to_int16(self, high: int, low: int) -> int:
        """
        Convert two bytes to signed 16-bit integer (little-endian)
        """
        value = (high << 8) | low
        if value >= 0x8000:
            value = -((65535 - value) + 1)
        return value

    def save_calibration(self) -> None:
        """
        Save calibration data to file.
        """
        try:
            cal_data = {
                'mag_offset': self.mag_offset,
                'mag_scale': self.mag_scale,
                'timestamp': time.time()
            }
            with open(self.CALIBRATION_FILE, 'w') as f:
                json.dump(cal_data, f)
            print(f"Calibration saved to {self.CALIBRATION_FILE}")
        except Exception as e:
            print(f"Failed to save calibration: {e}")

    def load_calibration(self) -> None:
        """
        Load calibration data from file.
        """
        try:
            with open(self.CALIBRATION_FILE, 'r') as f:
                cal_data = json.load(f)

            self.mag_offset = cal_data['mag_offset']
            self.mag_scale = cal_data['mag_scale']
        except OSError:
            print("No saved calibration found. Please calibrate the sensor.")
        except Exception as e:
            print(f"Failed to load calibration: {e}")

    def calibrate_magnetometer(self, duration_s: int = 60) -> bool:
        """
        Calibrate magnetometer by rotating device in all directions
        Duration: time in seconds to collect samples
        """

        mag_min = [float('inf')] * 3
        mag_max = [float('-inf')] * 3

        start_time = time.time()
        sample_count = 0
        last_print = 0

        print("\nCalibrating started", end="")

        while time.time() - start_time < duration_s:
            mag_data = self.read_mag()
            if mag_data:
                # Store raw uncalibrated values
                mx_raw = (mag_data[0] / self.mag_scale[0] + self.mag_offset[0]
                          if self.mag_scale[0] != 0
                          else mag_data[0])
                my_raw = (mag_data[1] / self.mag_scale[1] + self.mag_offset[1]
                          if self.mag_scale[1] != 0
                          else mag_data[1])
                mz_raw = (mag_data[2] / self.mag_scale[2] + self.mag_offset[2]
                          if self.mag_scale[2] != 0
                          else mag_data[2])

                # Update min/max
                mag_min[0] = min(mag_min[0], mx_raw)
                mag_min[1] = min(mag_min[1], my_raw)
                mag_min[2] = min(mag_min[2], mz_raw)

                mag_max[0] = max(mag_max[0], mx_raw)
                mag_max[1] = max(mag_max[1], my_raw)
                mag_max[2] = max(mag_max[2], mz_raw)

                sample_count += 1

                # Progress indicator
                current_time = time.time() - start_time
                if int(current_time) > last_print:
                    print(".", end="")
                    last_print = int(current_time)

            time.sleep_ms(10)

        print(f"\n\nCollected {sample_count} samples")

        if sample_count < 100:
            print("WARNING: Too few samples collected. Calibration may be inaccurate.")
            return False

        # Calculate offsets (hard iron distortion)
        self.mag_offset[0] = (mag_max[0] + mag_min[0]) / 2
        self.mag_offset[1] = (mag_max[1] + mag_min[1]) / 2
        self.mag_offset[2] = (mag_max[2] + mag_min[2]) / 2

        # Calculate scale factors (soft iron distortion)
        avg_delta = ((mag_max[0] - mag_min[0]) +
                     (mag_max[1] - mag_min[1]) +
                     (mag_max[2] - mag_min[2])) / 3

        self.mag_scale[0] = (avg_delta / (mag_max[0] - mag_min[0])
                             if (mag_max[0] - mag_min[0]) != 0
                             else 1.0)
        self.mag_scale[1] = (avg_delta / (mag_max[1] - mag_min[1])
                             if (mag_max[1] - mag_min[1]) != 0
                             else 1.0)
        self.mag_scale[2] = (avg_delta / (mag_max[2] - mag_min[2])
                             if (mag_max[2] - mag_min[2]) != 0
                             else 1.0)

        print("\nCalibration complete!")
        print(f"Min values: X={mag_min[0]:.3f}, Y={mag_min[1]:.3f}, Z={mag_min[2]:.3f}")
        print(f"Max values: X={mag_max[0]:.3f}, Y={mag_max[1]:.3f}, Z={mag_max[2]:.3f}")
        print(f"Offsets: X={self.mag_offset[0]:.3f}, Y={self.mag_offset[1]:.3f}, Z={self.mag_offset[2]:.3f}")
        print(f"Scales:  X={self.mag_scale[0]:.3f}, Y={self.mag_scale[1]:.3f}, Z={self.mag_scale[2]:.3f}")
        print("=" * 50)

        self.save_calibration()

        return True

    def get_tilt_compensated_heading(self) -> float | None:
        """
        Calculate tilt-compensated compass heading.
        Returns: heading in degrees (0-360), or None if data not available.
        """
        # Read sensors
        accel = self.read_accel()
        mag = self.read_mag()

        if mag is None:
            return None

        ax, ay, az = accel
        mx, my, mz = mag

        # Calculate roll and pitch from accelerometer.
        roll = math.atan2(ay, az)
        pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))

        # Tilt compensation.
        mag_x = mx * math.cos(pitch) + mz * math.sin(pitch)
        mag_y = (mx * math.sin(roll) * math.sin(pitch) +
                 my * math.cos(roll) -
                 mz * math.sin(roll) * math.cos(pitch))

        # Calculate heading.
        heading = math.atan2(mag_y, mag_x)
        heading = math.degrees(heading)

        # Normalize to 0-360 and return.
        return heading + 360 if heading < 0 else heading
