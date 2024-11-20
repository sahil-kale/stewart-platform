### Write data to the arduino controller
import serial
import struct
import time
import argparse
import numpy as np
import time


class PlatformController:
    def __init__(
        self,
        port,
        baudrate=921600,
        duty_cycle_min=500,
        duty_cycle_max=2300,
        debug: bool = False,
    ):
        # Min and max duty cycles translate to 0 to 180 degrees on the servo
        # gathered from empirical data primarily
        self.port = port
        self.ser = serial.Serial(port, baudrate)

        self.duty_cycle_min = duty_cycle_min
        self.duty_cycle_max = duty_cycle_max

        self.debug = debug

        # sleep for a bit to allow the serial port to open
        time.sleep(1)
        # Write that the controller is ready
        print("Platform controller ready.")

    def write_raw(self, data: bytearray):
        if self.debug:
            print(f"Writing to device: {data}")
        self.ser.write(data)

    def compute_duty_cycle_from_angle(self, servo_angle):
        # Map the desired angle between 0 to pi, and map that value to the
        # duty cycle based on min and max duty cycles
        percentage = servo_angle / np.pi

        # saturate the percentage to be between 0 and 1
        percentage = np.clip(percentage, 0, 1)

        duty_cycle = self.duty_cycle_min + percentage * (
            self.duty_cycle_max - self.duty_cycle_min
        )

        return int(duty_cycle)

    def write_duty_cycles(self, duty_cycle_1, duty_cycle_2, duty_cycle_3):
        bytes = bytearray()
        # Encode the duty cycles as 3x 2-byte unsigned integers, little-endian
        encoding_struct = struct.Struct("<HHH")

        bytes.extend(encoding_struct.pack(duty_cycle_1, duty_cycle_2, duty_cycle_3))

        # Calculate the checksum as the sum of the 3 duty cycles
        checksum = sum([duty_cycle_1, duty_cycle_2, duty_cycle_3])

        # Append the checksum as a 4-byte unsigned integer, little-endian
        bytes.extend(struct.pack("<I", checksum))

        self.write_raw(bytes)

    def read_serial_output(self):
        if self.ser.in_waiting > 0:
            # Read the serial data
            output = self.ser.readline().decode().strip()
            print(f"Received from device: {output}")

    def close_port(self):
        self.ser.close()


if __name__ == "__main__":
    # Argument parser to handle the port, as well as debug mode
    parser = argparse.ArgumentParser(description="Control the platform")
    parser.add_argument(
        "--port", help="The serial port to connect to", default="/dev/ttyACM0"
    )
    parser.add_argument(
        "--debug", action="store_true", help="Enable debug mode for additional output"
    )

    args = parser.parse_args()
    pc = PlatformController(args.port, debug=args.debug)
    while True:
        # Ask the user for the duty cycle (use the same one for all 3)
        try:
            servo_angle_deg = int(input("Enter the desired angle in degrees: "))
            servo_angle_rad = np.deg2rad(servo_angle_deg)
            duty_cycle = pc.compute_duty_cycle_from_angle(servo_angle_rad)
            # Write the duty cycle to the platform
            pc.write_duty_cycles(duty_cycle, duty_cycle, duty_cycle)
            time.sleep(0.1)  # Wait briefly for the microcontroller to respond
            pc.read_serial_output()  # Read and print the serial output
        except ValueError:
            print("Please enter a valid integer.")
        except KeyboardInterrupt:
            print("Exiting...")
            pc.close_port()
            break
