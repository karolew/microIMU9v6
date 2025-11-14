import time

from machine import I2C, Pin

from imu9v6 import MinIMU9v6


def main():
    # Initialize I2C (adjust pins if needed)
    i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)

    print("=" * 50)
    print("Demo usage of MinIMU-9 v6 Tilt-Compensated Compass")
    print("=" * 50)

    # Scan I2C bus
    devices = i2c.scan()
    print(f"I2C devices found: {[hex(d) for d in devices]}\n")

    try:
        sensor = MinIMU9v6(i2c, calibrate=True)

        # Run calibration
        print("\n*** CALIBRATION REQUIRED ***")
        print("The compass needs calibration for accurate readings.")
        print("You will need to rotate the sensor in all directions.")

        fail_count = 0

        while True:
            heading = sensor.get_tilt_compensated_heading()

            if heading is not None:
                print(f"Heading: {heading:6.2f}°\t", end="\r")
                fail_count = 0
            else:
                fail_count += 1
                if fail_count > 10:
                    print("No magnetometer data - check sensor!      ", end="\r")

            time.sleep_ms(100)

    except KeyboardInterrupt:
        print("\n\nCompass stopped")
    except Exception as e:
        print(f"\nError: {e}")
        import sys
        sys.print_exception(e)


if __name__ == "__main__":
    main()
