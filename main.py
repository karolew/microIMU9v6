import time
from machine import I2C, Pin

from imu9v6 import MinIMU9v6


class Button:
    def __init__(self, pin_num, callback, pin_mode=Pin.PULL_UP, irq_trigger=Pin.IRQ_FALLING, debounce_ms=20):
        self.callback = callback
        self.pin_mode = pin_mode
        self.irq_trigger = irq_trigger
        self.debounce_ms = debounce_ms
        self.pin = Pin(pin_num, Pin.IN, self.pin_mode)
        self.last_time = 0
        self.pin.irq(trigger=self.irq_trigger, handler=self._handler)

    def _handler(self, pin):
        current = time.ticks_ms()
        if time.ticks_diff(current, self.last_time) > self.debounce_ms:
            self.last_time = current
            if pin.value() == 0 if self.pin_mode == Pin.PULL_UP else 1:
                self.callback()

calibration = False

def handle_interrupt_for_compass_calibration() -> None:
    global calibration
    calibration = True
    print("Calibration...    \r")

compass_calibration_button = Button(19, handle_interrupt_for_compass_calibration)

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
        sensor = MinIMU9v6(i2c, calibrate=False)

        # Run calibration
        print("\n*** CALIBRATION REQUIRED ***")
        print("The compass needs calibration for accurate readings.")
        print("You will need to rotate the sensor in all directions.")

        fail_count = 0

        while True:
            # Handle heading.
            heading = sensor.get_tilt_compensated_heading()

            if heading is not None:
                print(f"Heading: {heading:6.2f}°\t", end="\r")
                fail_count = 0
            else:
                fail_count += 1
                if fail_count > 10:
                    print("No magnetometer data - check sensor!      ", end="\r")

            # Handle calibration
            global calibration
            if calibration:
                sensor.calibrate_magnetometer(5)
                calibration = False

            time.sleep_ms(50)

    except KeyboardInterrupt:
        print("\n\nCompass stopped")
    except Exception as e:
        print(f"\nError: {e}")
        import sys
        sys.print_exception(e)


if __name__ == "__main__":
    main()
