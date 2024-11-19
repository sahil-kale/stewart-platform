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
        degrees_per_step=1.8,
        microstep_factor=2,
        min_degree_rad=(5 * np.pi / 180),
        max_degree_rad=(85 * np.pi / 180),
        debug: bool = False,
    ):
        # Min and max duty cycles translate to 0 to 180 degrees on the servo
        # gathered from empirical data primarily
        self.port = port
        self.ser = serial.Serial(port, baudrate)

        self.degrees_per_step = degrees_per_step / microstep_factor
        self.min_degree_rad = min_degree_rad
        self.max_degree_rad = max_degree_rad

        self.debug = debug

        # sleep for a bit to allow the serial port to open
        time.sleep(1)
        # Write that the controller is ready
        print("Platform controller ready.")

    def write_raw(self, data: bytearray):
        if self.debug:
            print(f"Writing to device: {data}")
        self.ser.write(data)

    def compute_steps_from_angle(self, actuator_angle):
        # saturate the angle to the min 
        if actuator_angle < self.min_degree_rad:
            actuator_angle = self.min_degree_rad

        percentage = 1 - actuator_angle / np.pi
        total_steps = 180.0 / self.degrees_per_step
        return (int)(percentage * total_steps)

    def write_steps(self, steps_1, steps_2, steps_3):
        bytes = bytearray()
        # Encode the duty cycles as 3x 2-byte unsigned integers, little-endian
        encoding_struct = struct.Struct("<HHH")

        bytes.extend(encoding_struct.pack(steps_1, steps_2, steps_3))

        # Calculate the checksum as the sum of the 3 duty cycles
        checksum = sum([steps_1, steps_2, steps_3])

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
            actuator_angle_deg = int(input("Enter the desired angle in degrees: "))
            actuator_angle_rad = np.deg2rad(actuator_angle_deg)
            steps = pc.compute_steps_from_angle(actuator_angle_rad)
            # Write the duty cycle to the platform
            print(steps)
            pc.write_steps(steps, steps, steps)
            time.sleep(0.1)  # Wait briefly for the microcontroller to respond
            pc.read_serial_output()  # Read and print the serial output
        except ValueError:
            print("Please enter a valid integer.")
        except KeyboardInterrupt:
            print("Exiting...")
            pc.close_port()
            break
