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
        run_controller: bool = True,
        tune_controller: bool = False,
    ):
        self.pause_period = 0.01
        self.saturate_angle = 14.0
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

        # The gains are just set arbitrarily for now
        self.ball_controller = BallController(
            1.0, 1.0, 1.0, 1.0, 1.0, 1.0, self.pause_period, self.saturate_angle
        )

        self.run_visualizer = run_visualizer
        self.run_controller = run_controller

        if self.run_visualizer:
            self.visualizer = Kinematics3dPlotter(
                self.attachment_points, self.params["lh"], self.params["la"]
            )
            self.create_kinematic_sliders()

        self.tune_controller = tune_controller
        if self.tune_controller:
            plt.figure()
            # create a GUI with tuning sliders for kp, ki, and kd
            # for now, even though there are 6 PID gains, we'll just tune the x-axis gains and apply them to y
            self.create_pid_tuning_sliders()

        self.virtual = virtual

        if self.virtual:
            serial_port = create_fake_serial()[0]

        self.pc = PlatformController(serial_port, debug=False)

        # get the current directory
        current_dir = os.path.dirname(os.path.realpath(__file__))
        # get the file camera.json with the current dir
        file_path = os.path.join(current_dir, "camera_params.json")

        with open(file_path, "r") as file:
            data = json.load(file)  # Load JSON data as a dictionary

        if self.virtual is False:
            self.cv_system = Camera(data["u"], data["v"], camera_port, camera_debug)
            self.cv_system.open_camera()
            self.cv_system.calibrate()

        self.current_position = Point(0, 0)

        self.pause_period = 0.01

    def create_kinematic_sliders(self):
        """Create sliders for pitch, roll, and height"""
        ax_pitch = plt.axes([0.15, 0.02, 0.65, 0.03], facecolor="lightgoldenrodyellow")
        ax_roll = plt.axes([0.15, 0.06, 0.65, 0.03], facecolor="lightgoldenrodyellow")
        ax_height = plt.axes([0.15, 0.10, 0.65, 0.03], facecolor="lightgoldenrodyellow")

        self.slider_pitch = Slider(
            ax_pitch, "Pitch", -self.saturate_angle, self.saturate_angle, valinit=0.0
        )
        self.slider_roll = Slider(
            ax_roll, "Roll", -self.saturate_angle, self.saturate_angle, valinit=0.0
        )
        self.slider_height = Slider(ax_height, "Height", 0.02, 0.1, valinit=0.06)

    def create_pid_tuning_sliders(self):
        """Create sliders for tuning kp, ki, and kd gains for the x-axis"""
        ax_kp = plt.axes([0.15, 0.25, 0.65, 0.03], facecolor="lightgoldenrodyellow")
        ax_ki = plt.axes([0.15, 0.30, 0.65, 0.03], facecolor="lightgoldenrodyellow")
        ax_kd = plt.axes([0.15, 0.35, 0.65, 0.03], facecolor="lightgoldenrodyellow")

        self.slider_kp = Slider(ax_kp, "Kp", 0.0, 5.0, valinit=1.0)
        self.slider_ki = Slider(ax_ki, "Ki", 0.0, 5.0, valinit=1.0)
        self.slider_kd = Slider(ax_kd, "Kd", 0.0, 5.0, valinit=1.0)

    def run(self):
        while True:
            # Get pitch, roll, and height from the sliders
            if self.run_controller:
                if self.virtual is False:
                    self.current_position = self.cv_system.get_ball_coordinates()
                else:
                    self.current_position = Point(0, 0)

                desired_position = Point(1, 1)
                output_angles = self.ball_controller.run_control_loop(
                    desired_position, self.current_position
                )

                if self.tune_controller:
                    # get slider values
                    kp_x = self.slider_kp.val
                    ki_x = self.slider_ki.val
                    kd_x = self.slider_kd.val

                    self.ball_controller.set_gains(kp_x, ki_x, kd_x, kp_x, ki_x, kd_x)

                # Get desired position from trajectory controller
                # Get current position from CV controller (mocked out for now)
                desired_position = Point(1.0, 2.0)
                output_angles = self.ball_controller.run_control_loop(
                    desired_position, self.current_position
                )

                pitch_rad = output_angles[0]
                roll_rad = output_angles[1]
                height = 0.060  # default height
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
            elif self.tune_controller:
                plt.pause(self.pause_period)
            else:
                time.sleep(self.pause_period)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--visualize", action="store_true", help="Run the 3D visualization"
    )

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

    parser.add_argument(
        "--inhibit_controller",
        action="store_true",
        help="Run the control loop",
    )
    parser.add_argument(
        "--tune_controller",
        action="store_true",
        help="Run the control loop in tuning mode",
    )

    args = parser.parse_args()

    run_controller = args.inhibit_controller == False

    # Current computed offsets are [0, 8, 10] for servo 0, 1, and 2 respectively
    servo_offsets = [0, 8, 10]

    mcl = MainControlLoop(
        args.port,
        args.camera_port,
        virtual=args.virtual,
        camera_debug=args.camera_debug,
        run_visualizer=args.visualize,
        servo_offsets=servo_offsets,
        tune_controller=args.tune_controller,
        run_controller=run_controller,
    )
    mcl.run()
