import numpy as np
from platform_controller import PlatformController
from platform_kinematics_module import PlatformKinematicsModule
from camera import Camera
from point import Point
from servo_kinematics_module import ServoKinematicsModule
from servo_kinematics_feeder import ServoKinematicsFeeder
from kinematics_3d_plotter import Kinematics3dPlotter
from ball_controller import BallController
import argparse
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import time
import json

import os, pty


def create_fake_serial():
    master, slave = pty.openpty()
    s_name = os.ttyname(slave)
    return s_name, master


class MainControlLoop:
    def __init__(
        self,
        serial_port: str,
        camera_port: str,
        servo_offsets: list[float] = [0, 0, 0],  # degrees
        virtual: bool = False,
        camera_debug: bool = False,
        run_visualizer: bool = False,
        run_controller: bool = False,
    ):
        self.params = {
            "lh": 41 / 1000,
            "la": 51 / 1000,
            "platform_attachment_radius": 100 / 1000,
        }

        self.servo_offset_radians = [
            np.radians(servo_offsets[0]),
            np.radians(servo_offsets[1]),
            np.radians(servo_offsets[2]),
        ]

        attachment_points_structure = [
            [1.0, 0.0, 0.0],
            [-0.5, 0.86602540378, 0.0],
            [-0.5, -0.86602540378, 0.0],
        ]

        self.attachment_points = np.array(attachment_points_structure)
        self.attachment_points = (
            self.attachment_points * self.params["platform_attachment_radius"]
        )

        self.pk = PlatformKinematicsModule(self.attachment_points)
        self.sk = ServoKinematicsModule(self.params["lh"], self.params["la"])
        
        # The gains are just 
        self.ball_controller = BallController(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0)

        self.run_visualizer = run_visualizer
        self.run_controller = run_controller

        if self.run_visualizer:
            self.visualizer = Kinematics3dPlotter(
                self.attachment_points, self.params["lh"], self.params["la"]
            )
            self.create_sliders()

        self.virtual = virtual

        if self.virtual:
            serial_port = create_fake_serial()[0]

        self.pc = PlatformController(serial_port, debug=False)

        # Read camera parameters
        # Open the JSON file
        with open("data.json", "r") as file:
            data = json.load(file)  # Load JSON data as a dictionary

        self.cv_system = Camera(data["u"], data["v"], camera_port, camera_debug)

        self.current_position = Point(0, 0, 0)

        self.cv_system.open_camera()
        self.cv_system.calibrate()

        self.pause_period = 0.01

    def create_sliders(self):
        """Create sliders for pitch, roll, and height"""
        ax_pitch = plt.axes([0.15, 0.02, 0.65, 0.03], facecolor="lightgoldenrodyellow")
        ax_roll = plt.axes([0.15, 0.06, 0.65, 0.03], facecolor="lightgoldenrodyellow")
        ax_height = plt.axes([0.15, 0.10, 0.65, 0.03], facecolor="lightgoldenrodyellow")

        self.slider_pitch = Slider(ax_pitch, "Pitch", -14.0, 14.0, valinit=0.0)
        self.slider_roll = Slider(ax_roll, "Roll", -14.0, 14.0, valinit=0.0)
        self.slider_height = Slider(ax_height, "Height", 0.02, 0.1, valinit=0.06)

    def run(self):
        while True:
            # Get pitch, roll, and height from the sliders
            if self.run_controller:
                self.current_position = self.cv_system.get_ball_coordinates()
                pass
            else:
                pitch_rad = np.deg2rad(self.slider_pitch.val)
                roll_rad = np.deg2rad(self.slider_roll.val)
                height = self.slider_height.val

            # Compute the platform's new attachment points
            platform_points = self.pk.compute_transformed_platform_attachment_points(
                roll_rad, pitch_rad
            )
            skf = ServoKinematicsFeeder(height, self.attachment_points)
            servo_vectors, platform_points_in_base_frame = skf.compute_servo_vector(
                platform_points
            )

            # Compute the servo angles
            servo_angles = []
            duty_cycles = []
            for i in range(len(servo_vectors)):
                desired_servo_length = np.linalg.norm(servo_vectors[i])
                servo_angle = self.sk.compute_servo_angle(desired_servo_length)
                servo_angles.append(servo_angle)
                duty_cycle = self.pc.compute_duty_cycle_from_angle(
                    servo_angle[0] + self.servo_offset_radians[i]
                )
                duty_cycles.append(duty_cycle)

            # Send the servo angles to the platform
            if not self.virtual:
                self.pc.write_duty_cycles(
                    duty_cycles[0], duty_cycles[1], duty_cycles[2]
                )

            # Update visualization if enabled
            if self.run_visualizer:
                self.visualizer.update(platform_points_in_base_frame, servo_angles)
                # Redraw the figure
                plt.pause(self.pause_period)
            else:
                time.sleep(self.pause_period)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--visualize", action="store_true", help="Run the 3D visualization"
    )

    # add an argument for the port
    parser.add_argument(
        "--port",
        type=str,
        default="/dev/ttyACM0",
        help="The serial port for the platform controller",
    )

    parser.add_argument(
        "--camera_port",
        type=str,
        default="/dev/video4",
        help="The port for the camera",
    )

    # add an argument for virtual
    parser.add_argument(
        "--virtual",
        action="store_true",
        help="Run the control loop in virtual mode",
    )

    # add argument for CV/Camera debug
    parser.add_argument(
        "--camera_debug",
        type=bool,
        default=False,
        help="Flag for showing more camera info for debugging",
    )

    args = parser.parse_args()

    # for now, if visualize is not set, raise an error saying that the ball controller is not implemented yet and needs to run with the visualize flag
    if not args.visualize:
        raise NotImplementedError(
            "Ball controller not implemented yet. Please run with the --visualize flag."
        )

    # Current computed offsets are [0, 8, 10] for servo 0, 1, and 2 respectively
    servo_offsets = [0, 8, 10]

    mcl = MainControlLoop(
        args.port,
        args.camera_port,
        virtual=args.virtual,
        camera_debug=args.camera_debug,
        run_visualizer=args.visualize,
        servo_offsets=servo_offsets,
    )
    mcl.run()
