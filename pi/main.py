import numpy as np
from platform_controller import PlatformController
from platform_kinematics_module import PlatformKinematicsModule
from camera import Camera
from point import Point, lpf_point
from servo_kinematics_module import ServoKinematicsModule
from servo_kinematics_feeder import ServoKinematicsFeeder
from kinematics_3d_plotter import Kinematics3dPlotter
from ball_controller import BallController
import argparse
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from kalman_filter import KalmanFilter
from logger import Logger
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
        cam_color_mask_detect: bool = False,
        cam_calibration_images: bool = False,
    ):
        self.dt = 0.05
        self.saturate_angle = 25
        self.params = {
            "lh": 67.5 / 1000,
            "la": 122.2 / 1000,
            "platform_attachment_radius": 70 / 1000,
            "base_attachment_radius": 100 / 1000,
            "resting_height": 0.12,
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

        self.platform_attachment_points = np.array(attachment_points_structure)
        self.platform_attachment_points = (
            self.platform_attachment_points * self.params["platform_attachment_radius"]
        )

        self.base_attachment_points = np.array(attachment_points_structure)
        self.base_attachment_points = (
            self.base_attachment_points * self.params["base_attachment_radius"]
        )

        self.pk = PlatformKinematicsModule(self.platform_attachment_points)
        self.sk = ServoKinematicsModule(
            self.params["lh"],
            self.params["la"],
            self.params["base_attachment_radius"]
            - self.params["platform_attachment_radius"],
        )

        kp = 2.0
        ki = 0.0
        kd = 30
        self.ball_controller = BallController(
            kp,
            ki,
            kd,
            kp,
            ki,
            kd,
            self.dt,
            np.deg2rad(self.saturate_angle),
        )

        self.run_visualizer = run_visualizer
        self.run_controller = run_controller

        if self.run_visualizer:
            self.visualizer = Kinematics3dPlotter(
                self.base_attachment_points, self.params["lh"], self.params["la"]
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
        cv_params_file_path = os.path.join(current_dir, "camera_params.json")

        self.camera_debug = camera_debug

        with open(cv_params_file_path, "r") as file:
            data = json.load(file)  # Load JSON data as a dictionary

        if self.virtual is not True:
            self.cv_system = Camera(data["u"], data["v"], camera_port, camera_debug)
            self.cv_system.load_camera_params("pi/camera_calibration_data.json")

        self.kalman_filter = KalmanFilter(
            self.dt, Q=100, R=200
        )  # Use default params for now

        self.current_position = Point(0, 0)

        self.logger = Logger("logs/log_{time.strftime('%Y-%m-%d_%H-%M-%S')}.json")

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
        self.slider_height = Slider(
            ax_height, "Height", 0.02, 0.2, valinit=self.params["resting_height"]
        )

    def create_pid_tuning_sliders(self):
        """Create sliders for tuning kp, ki, and kd gains for the x-axis"""
        ax_kp = plt.axes([0.15, 0.25, 0.65, 0.03], facecolor="lightgoldenrodyellow")
        ax_ki = plt.axes([0.15, 0.30, 0.65, 0.03], facecolor="lightgoldenrodyellow")
        ax_kd = plt.axes([0.15, 0.35, 0.65, 0.03], facecolor="lightgoldenrodyellow")

        self.slider_kp = Slider(ax_kp, "Kp", 0.0, 2.0, valinit=1.0)
        self.slider_ki = Slider(ax_ki, "Ki", 0.0, 2.0, valinit=0.0)
        self.slider_kd = Slider(ax_kd, "Kd", 0.0, 2.0, valinit=0.0)

    def run(self):
        # create a new log file path for the current run with the current time
        # make a new directory "logs" if it doesn't exist
        if not os.path.exists("logs"):
            os.makedirs("logs")

        while True:
            # Get pitch, roll, and height from the sliders
            time_since_start = time.time()
            camera_valid = False
            desired_position = Point(0, 0)
            if self.run_controller:
                if self.virtual is False:
                    current_measurement = self.cv_system.get_ball_coordinates()

                    self.current_position = self.kalman_filter.predict()

                    if current_measurement is not None:
                        self.current_position = self.kalman_filter.update(
                            current_measurement
                        )
                        camera_valid = True
                        print(
                            f"Time: {time.time()} | Current position is: {self.current_position} | Measured position is: {current_measurement}"
                        )
                    else:
                        if self.camera_debug:
                            print("Ball not detected")

                else:
                    self.current_position = Point(0, 0)

                output_angles = self.ball_controller.run_control_loop(
                    desired_position, self.current_position
                )

                if self.tune_controller:
                    # get slider values
                    kp_x = self.slider_kp.val
                    ki_x = self.slider_ki.val
                    kd_x = self.slider_kd.val

                    self.ball_controller.set_gains(kp_x, ki_x, kd_x, kp_x, ki_x, kd_x)

                pitch_rad = output_angles[0]
                roll_rad = output_angles[1]
                height = self.params["resting_height"]
            else:
                # set the pitch and roll sliders to negative whatever it was before
                # self.slider_pitch.set_val(-self.slider_pitch.val)
                # self.slider_roll.set_val(-self.slider_roll.val)

                pitch_rad = np.deg2rad(self.slider_pitch.val)
                roll_rad = np.deg2rad(self.slider_roll.val)
                height = self.slider_height.val

            # Compute the platform's new attachment points
            platform_points = self.pk.compute_transformed_platform_attachment_points(
                roll_rad, pitch_rad
            )
            skf = ServoKinematicsFeeder(height, self.platform_attachment_points)
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

            # log the following data to the log file
            # time, current position, camera valid, desired position, pitch, roll, height, servo angles, duty cycles
            # ensure that all data is separated by commas and are converted to strictly numbers
            if current_measurement is None:
                current_measurement_x = None
                current_measurement_y = None
            else:
                current_measurement_x = current_measurement.x
                current_measurement_y = current_measurement.y
            data = [
                {
                    "time": time_since_start,
                    "camera_measured_x": current_measurement_x,
                    "camera_measured_y": current_measurement_y,
                    "camera_filtered_x": self.current_position.x,
                    "camera_filtered_y": self.current_position.y,
                    "desired_position_x": desired_position.x,
                    "desired_position_y": desired_position.y,
                    "pitch_rad": pitch_rad,
                    "roll_rad": roll_rad,
                    "height": height,
                    "servo_1_angle": servo_angle[0][0],
                    "servo_2_angle": servo_angle[1][0],
                    "servo_3_angle": servo_angle[2][0]
                }
            ]

            self.logger.log_new_data(data)

            time_elapsed = time.time() - time_since_start
            pause_time = self.dt - time_elapsed
            if pause_time < 0:
                print(
                    f"Loop is taking too long to run! {time_elapsed} seconds, {pause_time} seconds"
                )
                pause_time = 0
            # Update visualization if enabled
            if self.run_visualizer:
                self.visualizer.update(platform_points_in_base_frame, servo_angles)
                # Redraw the figure
                plt.pause(pause_time)
            elif self.tune_controller:
                plt.pause(pause_time)
            else:
                time.sleep(pause_time)


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
        default="/dev/video0",
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
        action="store_true",
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

    parser.add_argument(
        "--cam_color_mask_detect",
        action="store_true",
        help="Run the camera calibration",
    )

    args = parser.parse_args()

    run_controller = args.inhibit_controller == False

    servo_offsets = [0, 0, -10]

    mcl = MainControlLoop(
        args.port,
        args.camera_port,
        virtual=args.virtual,
        camera_debug=args.camera_debug,
        run_visualizer=args.visualize,
        servo_offsets=servo_offsets,
        tune_controller=args.tune_controller,
        run_controller=run_controller,
        cam_color_mask_detect=args.cam_color_mask_detect,
    )
    mcl.run()
